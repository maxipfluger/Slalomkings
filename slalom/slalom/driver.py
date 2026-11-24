"""Slalom driver = state machine (S2) + steering controller (S3) in one node.

Input : ConeArray (from the simulator, the OpenCV detector or the AI model)
Output: Twist on /cmd_vel  (linear.x = throttle, angular.z = steering)

States
  IDLE     : does nothing (manual driving with the controller is possible)
  APPROACH : drive to a point next to the nearest yellow cone,
             left/right alternating; a new cone is detected when the
             nearest cone suddenly is much farther away (the old one is passed)
  SEARCH   : no yellow cone visible -> drive slowly towards the cone line,
             red cone close -> steer right, blue cone close -> steer left
  DONE     : stopped in front of the pink cone

Safety
  - Watchdog: no cone message for `watchdog_s` -> stop
  - Controller: L1 or circle -> IDLE (stop), triangle -> start
  - When not driving, the node sends a few zero commands and then stays
    silent, so the PS controller can drive the car manually.

Steering convention inside this file: positive = LEFT (ROS standard).
The real car uses positive = RIGHT -> parameter steering_sign = -1.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from rclpy.qos import qos_profile_sensor_data
from slalom_interfaces.msg import Cone, ConeArray


class SlalomBrain:
    """Pure logic, no ROS -> can be tested without a car or ROS."""

    def __init__(self, wheelbase=0.33, turning_radius=0.63, offset=0.55,
                 first_side_left=True, stop_dist=0.7, new_cone_jump=0.8,
                 search_steer=0.4, wall_dist=1.0, pass_max_x=1.0, pass_cooldown=8, hold_frames=4, ignore_frames=12, clear_frames=25, pink_hold=15, pink_lost_stop=1.0, steer_gain=1.0, min_lookahead=0.0, clear_always=False, side_check=False, side_max_frames=25, pink_after=0):
        self.wheelbase = wheelbase
        self.max_steer = math.atan(wheelbase / turning_radius)  # rad
        self.offset = offset                # m, lateral distance when passing a cone
        self.first_side = 1 if first_side_left else -1
        self.stop_dist = stop_dist          # m, stop this far before the pink cone
        self.new_cone_jump = new_cone_jump  # m, jump in distance = old cone passed
        self.search_steer = search_steer    # -1..1, steering while searching
        self.wall_dist = wall_dist          # m, wall cones closer than this count
        self.pass_max_x = pass_max_x        # m, a cone only counts as passed if it was this close
        self.ignore_frames = ignore_frames  # updates after a pass in which close cones (the passed one) are ignored
        self.side_check = side_check        # LiDAR: only turn to the next side when that side is free
        self.pink_after = pink_after        # pink is only believed after this many passed yellow cones (red wall cones look pink sometimes)
        self.side_max_frames = side_max_frames  # max updates to wait for a free side (then turn anyway)
        self.blocked = {1: False, -1: False}    # set by the node from /scan: +1 = left, -1 = right
        self.clear_always = clear_always    # drive straight past EVERY cone that left the view, not only the last one
        self.steer_gain = steer_gain        # factor on the cone-following steering (human data: steer earlier)
        self.min_lookahead = min_lookahead  # m, aim point at least this far ahead (human data: ease off close to the cone)
        self.pink_lost_stop = pink_lost_stop  # m, pink seen closer than this and then lost (too close for the camera) -> stop
        self.clear_frames = clear_frames    # updates to drive straight past the last yellow cone before turning to pink
        self.pink_hold = pink_hold          # updates to wait for a flickering yellow cone before turning to pink
        self.hold_frames = hold_frames      # updates to keep steering if the yellow cone drops out briefly
        self.pass_cooldown = pass_cooldown  # updates (~11 Hz) before the next pass can count
        self.reset()

    def reset(self):
        self.state = 'IDLE'
        self.side = self.first_side         # +1 = pass on the left, -1 = right
        self.passed = 0
        self.last_x = None
        self.since_pass = 999
        self.missing = 0
        self.last_steer = 0.0
        self.clear = 0
        self.pink_recent = 0
        self.end_phase = False
        self.pink_close = 0
        self.last_y = 0.0
        self.side_wait = 0
        self.lost_dir = 0              # +1/-1 = cone lost before passing it, it was left/right

    def start(self):
        self.reset()
        self.state = 'APPROACH'

    def stop(self):
        self.state = 'IDLE'

    def pursuit(self, tx, ty):
        """Pure pursuit: steering (-1..1, positive = left) to reach point (tx, ty)."""
        ld = math.hypot(tx, ty)
        if ld < 1e-3:
            return 0.0
        alpha = math.atan2(ty, tx)
        delta = math.atan(2.0 * self.wheelbase * math.sin(alpha) / ld)
        return max(-1.0, min(1.0, delta / self.max_steer))

    def update(self, cones):
        """cones: list of (color, x, y). Returns steering -1..1 (positive = left) or None = stop."""
        if self.state in ('IDLE', 'DONE'):
            return None
        self.since_pass += 1
        can_pass = (self.last_x is not None and self.last_x < self.pass_max_x
                    and self.since_pass >= self.pass_cooldown)

        yellow = sorted([(x, y) for c, x, y in cones if c == Cone.YELLOW and x > 0.0])
        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0] if self.passed >= self.pink_after else []
        self.pink_recent = 30 if pink else max(0, self.pink_recent - 1)
        if pink and min(pink)[0] < self.pink_lost_stop:
            self.pink_close = 3             # pink is close ahead
        elif not pink and self.pink_close > 0:
            self.pink_close -= 1
            if self.pink_close == 0 and self.end_phase:   # close pink vanished (below the camera view) -> we are there
                self.state = 'DONE'
                return None
        if self.since_pass < self.ignore_frames:   # just passed -> do not chase the passed cone again
            yellow = [(x, y) for x, y in yellow if x > self.pass_max_x]

        # pink cone close and no yellow cone in front of it -> stop
        if pink:
            px, py = min(pink)
            if px < self.stop_dist and (not yellow or yellow[0][0] > px):
                self.state = 'DONE'
                return None

        if yellow:
            x, y = yellow[0]
            if self.last_x is not None and x > self.last_x + self.new_cone_jump and self.missing < self.hold_frames:
                # near cone vanished, only a far one is seen -> could be flicker, confirm over a few updates
                self.missing += 1
                return self.last_steer
            if can_pass and x > self.last_x + self.new_cone_jump:
                self.side = -self.side      # old cone passed -> other side for the next one
                self.passed += 1
                self.since_pass = 0
            self.last_x = x
            self.last_y = y
            self.lost_dir = 0
            self.state = 'APPROACH'
            self.missing = 0
            self.last_steer = max(-1.0, min(1.0, self.steer_gain * self.pursuit(max(x, self.min_lookahead), y + self.side * self.offset)))
            return self.last_steer

        # yellow cone dropped out for a moment (detection flicker) -> keep the last steering
        # far cone flickers out more often -> wait longer (pink_hold) before searching
        hold = self.hold_frames if (self.last_x is None or self.last_x < self.pass_max_x) else self.pink_hold
        if self.last_x is not None and self.missing < hold:
            self.missing += 1
            return self.last_steer

        if pink:                            # last yellow passed -> drive to the pink cone
            if self.last_x is not None and self.last_x < self.pass_max_x:
                # the last yellow cone just left the view -> count it and first drive straight past it
                self.side = -self.side
                self.passed += 1
                self.since_pass = 0
                self.last_x = None
                self.clear = self.clear_frames
                self.end_phase = True
            if self.last_x is not None and self.missing < self.pink_hold:
                # yellow cone still ahead but flickering -> keep going for it, do not turn to pink yet
                self.missing += 1
                return self.last_steer
            if self.clear > 0:
                self.clear -= 1
                self.state = 'APPROACH'
                return 0.0
            px, py = min(pink)
            self.state = 'APPROACH'
            return self.pursuit(px, py)

        # nothing to follow -> SEARCH
        if self.state == 'APPROACH' and self.last_x is not None:
            if can_pass:                    # the cone we followed has left the view = passed
                self.side = -self.side
                self.passed += 1
                self.since_pass = 0
                self.lost_dir = 0
                if self.pink_recent > 0 or self.clear_always:   # passed cone is still beside us -> first drive straight past it
                    self.clear = self.clear_frames
                    self.end_phase = True
            else:                           # lost it before passing -> turn back towards it
                self.lost_dir = 1 if self.last_y > 0 else -1
            self.last_x = None
        self.state = 'SEARCH'
        reds = [x for c, x, y in cones if c == Cone.RED and 0 < x < self.wall_dist]
        blues = [x for c, x, y in cones if c == Cone.BLUE and 0 < x < self.wall_dist]
        if reds and not blues:
            return -self.search_steer       # left wall close -> right
        if blues and not reds:
            return self.search_steer        # right wall close -> left
        if self.clear > 0:                  # just passed a cone that is still beside us -> straight, do not cut into it
            self.clear -= 1
            return 0.0
        if self.lost_dir:                   # cone left the view before we passed it -> look for it again
            return self.lost_dir * self.search_steer
        # we are on the -side of the line now -> steer back towards it
        if self.side_check and self.blocked.get(self.side) and self.side_wait < self.side_max_frames:
            self.side_wait += 1             # passed cone still beside us on that side -> straight first
            return 0.0
        self.side_wait = 0
        return self.side * self.search_steer


class TrackBrain(SlalomBrain):
    """Slalom with memory: every cone is kept in a small map (car frame) and moved
    with the car's own motion (dead reckoning from speed + steering). So a cone that
    left the camera view is still known -> the car keeps a smooth line around it and
    already steers to the next cone instead of searching blind with full lock."""

    def __init__(self, dr_speed=0.4, switch_x=0.35, gate=0.45, max_age=40,
                 fov_deg=25.0, unseen_drop=12, spacing=2.0, max_steer_cmd=0.8, lookahead=0.9, beside=0.3, min_pass_travel=1.0, virtual_max=3.0, near_range=2.2, **kw):
        self.dr_speed = dr_speed      # m/s real speed at the normal throttle (bag 23.11: ~0.4)
        self.switch_x = switch_x      # m, cone this close ahead (beside the car) = passed -> next cone
        self.gate = gate              # m, max distance to match a detection with a known cone
        self.max_age = max_age        # updates a cone is kept without seeing it
        self.fov_deg = fov_deg        # deg, half camera view (cones beyond it are not expected to be seen)
        self.unseen_drop = unseen_drop  # updates in view but not seen -> was a false detection, drop
        self.max_steer_cmd = max_steer_cmd
        self.near_range = near_range    # m, detections closer than this have a reliable distance
        self.virtual_max = virtual_max  # m, guess the next cone only this far after a pass (dead reckoning drifts)
        self.spacing = spacing        # m, cone spacing (used to guess the next cone when it is not seen yet)
        self.lookahead = lookahead    # m, pure pursuit distance along the planned path (bigger = smoother)
        self.min_pass_travel = min_pass_travel  # m driven before the next cone can count as passed
        self.beside = beside          # m, straight piece of path beside each cone (before and after it)
        super().__init__(**kw)

    def reset(self):
        super().reset()
        self.tracks = []              # dicts: color, x, y, age, hits, miss
        self.target = None
        self.last_u = 0.0
        self.pink_seen = False
        self.travel = 99.0            # m driven since the last pass
        self.tracks.append({'color': -1, 'x': 0.0, 'y': 0.0, 'age': 0, 'hits': 99, 'miss': 0, 'seen': True, 'near': 99})  # start point (car is aimed along the line)

    def move(self, dt):
        """Shift all known cones by the car's motion during dt (car frame: x forward, y left)."""
        if self.state in ('IDLE', 'DONE') or dt <= 0.0:
            return
        ds = self.dr_speed * dt
        self.travel += ds
        dyaw = ds / self.wheelbase * math.tan(self.last_u * self.max_steer)
        dx, dy = ds * math.cos(dyaw / 2), ds * math.sin(dyaw / 2)
        c, s = math.cos(-dyaw), math.sin(-dyaw)
        for t in self.tracks:
            px, py = t['x'] - dx, t['y'] - dy
            t['x'], t['y'] = c * px - s * py, s * px + c * py

    def associate(self, cones):
        for t in self.tracks:
            t['age'] += 1
            t['seen'] = False
        for color, x, y in cones:
            if color not in (Cone.YELLOW, Cone.PINK) or x <= 0.0:
                continue
            best, bd = None, self.gate
            for t in self.tracks:
                if t['color'] == color and not t['seen']:
                    dd = math.hypot(t['x'] - x, t['y'] - y)
                    if dd < bd:
                        best, bd = t, dd
            if best is None:
                self.tracks.append({'color': color, 'x': x, 'y': y, 'age': 0, 'hits': 1, 'miss': 0, 'seen': True,
                                    'near': 1 if math.hypot(x, y) < self.near_range else 0})
            else:
                dd = math.hypot(x, y)
                w = 0.7 if dd < 1.5 else (0.4 if dd < self.near_range else 0.15)   # far = inaccurate distance
                if dd < self.near_range:
                    best['near'] = best.get('near', 0) + 1
                best['x'] += w * (x - best['x'])
                best['y'] += w * (y - best['y'])
                best['age'] = 0
                best['hits'] += 1
                best['miss'] = 0
                best['seen'] = True
        keep = []
        for t in self.tracks:
            in_view = 0.4 < t['x'] < 1.8 and abs(math.degrees(math.atan2(t['y'], t['x']))) < self.fov_deg
            if not t['seen'] and in_view:
                t['miss'] += 1
            if t['color'] == -1:
                if t['x'] > -2.5:
                    keep.append(t)
                continue
            if t['x'] < -2.5 or (t['age'] > self.max_age and t['x'] > 0.0) or t['miss'] > self.unseen_drop:
                if t is self.target:
                    self.target = None
                continue
            keep.append(t)
        # merge duplicates (one cone seen as two)
        merged = []
        for t in sorted(keep, key=lambda q: -q['hits']):
            if any(m['color'] == t['color'] and math.hypot(m['x'] - t['x'], m['y'] - t['y']) < 0.35 for m in merged):
                if t is self.target:
                    self.target = None
                continue
            merged.append(t)
        self.tracks = merged

    def update(self, cones, dt=0.09):
        if self.state in ('IDLE', 'DONE'):
            return None
        self.move(dt)
        self.associate(cones)

        yellow = sorted([t for t in self.tracks if t['color'] == Cone.YELLOW and t['hits'] >= 2],
                        key=lambda t: t['x'])
        pink = sorted([t for t in self.tracks if t['color'] == Cone.PINK and t['hits'] >= 2],
                      key=lambda t: t['x'])
        pink_seen_now = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]
        if pink_seen_now:
            self.pink_seen = True

        # current target passed? (it is now beside the car)
        if self.target is not None and self.target['x'] < self.switch_x:
            if self.travel >= self.min_pass_travel:
                self.side = -self.side
                self.passed += 1
                self.since_pass = 0
                self.travel = 0.0
            for t in self.tracks:              # forget duplicates of the passed cone
                if t is not self.target and t['color'] == Cone.YELLOW and \
                        math.hypot(t['x'] - self.target['x'], t['y'] - self.target['y']) < 0.6:
                    t['x'] = min(t['x'], self.target['x'])
            self.target = None
        ahead_all = [t for t in yellow if t['x'] >= self.switch_x]
        ahead = [t for t in ahead_all if t.get('near', 0) >= 2]      # seen close enough to trust
        origin = [t for t in self.tracks if t['color'] == -1]
        behind = [t for t in yellow if t['x'] < self.switch_x]
        no_pink_ahead = not any(p['x'] > 0.3 for p in pink)
        guess_ok = bool(behind) and no_pink_ahead and self.travel < self.virtual_max and \
            (len(behind) >= 2 or bool(origin))
        if not ahead and ahead_all and not guess_ok:
            ahead = ahead_all                                          # only far cones known and no better guess
        yellow = [t for t in yellow if t['x'] < self.switch_x or t in ahead]   # path only through trusted cones
        if ahead:
            self.target = ahead[0]
        else:
            self.target = None
            # no trusted cone ahead -> guess it from the cones already passed (same line, same spacing)
            if guess_ok:
                a, b = (behind[-2] if len(behind) >= 2 else origin[0]), behind[-1]
                n = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                if 1.0 < n < 3.5:
                    gx = b['x'] + self.spacing * (b['x'] - a['x']) / n
                    gy = b['y'] + self.spacing * (b['y'] - a['y']) / n
                    if gx < self.switch_x and self.travel >= self.min_pass_travel:
                        # we drove past the guessed cone without ever seeing it -> count it, remember it
                        self.side = -self.side
                        self.passed += 1
                        self.since_pass = 0
                        self.travel = 0.0
                        self.tracks.append({'color': Cone.YELLOW, 'x': gx, 'y': gy, 'age': 0, 'hits': 2,
                                            'miss': 0, 'seen': True, 'near': 2})
                    elif gx >= self.switch_x:
                        yellow = yellow + [{'color': Cone.YELLOW, 'x': gx, 'y': gy, 'virtual': True}]
                        self.virtual = yellow[-1]

        # stop: pink close and no yellow cone in front of it
        if pink:
            p = pink[0]
            no_yellow_before = (self.target is None or self.target['x'] > p['x'])
            if p['x'] < self.stop_dist and no_yellow_before and self.passed > 0 and p.get('near', 0) >= 2:
                self.state = 'DONE'
                return None

        virtual = getattr(self, 'virtual', None)
        self.virtual = None
        if self.target is not None or virtual is not None:
            tx, ty = self.path_point(yellow, pink)
            u = self.steer_gain * self.pursuit(tx, ty)
            self.state = 'APPROACH'
        elif pink:
            p = pink[0]
            u = self.pursuit(max(p['x'], 0.3), p['y'])
            self.state = 'APPROACH'
        else:
            self.state = 'SEARCH'
            u = self.side * self.search_steer
            if self.side_check and self.blocked.get(self.side) and self.side_wait < self.side_max_frames:
                self.side_wait += 1
                u = 0.0
            else:
                self.side_wait = 0
        u = max(-self.max_steer_cmd, min(self.max_steer_cmd, u))
        self.last_u = u
        self.last_steer = u
        return u

    def path_point(self, yellow, pink):
        """Plan a path through the points beside the next cones (alternating sides),
        with a short straight piece beside each cone, and return the point on it
        `lookahead` metres away from the car (pure pursuit on a path)."""
        cones = [t for t in yellow if t['x'] >= self.switch_x][:2]
        behind = [t for t in yellow if t['x'] < self.switch_x]
        line = cones + (pink[:1] if len(cones) < 2 else [])
        if not line:
            return self.lookahead, 0.0
        # direction of the cone line
        if len(line) >= 2:
            ax, ay, bx, by = line[0]['x'], line[0]['y'], line[1]['x'], line[1]['y']
        elif behind:
            ax, ay, bx, by = behind[-1]['x'], behind[-1]['y'], line[0]['x'], line[0]['y']
        else:
            ax, ay, bx, by = 0.0, 0.0, line[0]['x'], line[0]['y']
        n = math.hypot(bx - ax, by - ay) or 1.0
        dx, dy = (bx - ax) / n, (by - ay) / n
        nx, ny = -dy, dx                       # left normal of the line
        pts = [(0.0, 0.0)]
        if behind and behind[-1]['x'] > -self.beside and not behind[-1].get('virtual'):
            q = behind[-1]                     # cone we just passed: first finish the straight piece beside it
            ps = -self.side
            ex, ey = q['x'] + ps * self.offset * nx + self.beside * dx, q['y'] + ps * self.offset * ny + self.beside * dy
            if ex > 0.05:
                pts.append((ex, ey))
        side = self.side
        for t in cones:
            px, py = t['x'] + side * self.offset * nx, t['y'] + side * self.offset * ny
            pts += [(px - self.beside * dx, py - self.beside * dy), (px + self.beside * dx, py + self.beside * dy)]
            side = -side
        if len(cones) < 2:
            if pink and (not cones or pink[0]['x'] > cones[-1]['x']):
                pts.append((pink[0]['x'], pink[0]['y']))
            else:
                lx, ly = pts[-1]
                pts.append((lx + 2.0 * dx, ly + 2.0 * dy))
        # walk along the path from the car
        ld = self.lookahead
        for (x1, y1), (x2, y2) in zip(pts[:-1], pts[1:]):
            if math.hypot(x2, y2) >= ld:
                # intersection of segment with circle radius ld (take the far solution)
                fx, fy = x2 - x1, y2 - y1
                a = fx * fx + fy * fy
                b = 2 * (x1 * fx + y1 * fy)
                c = x1 * x1 + y1 * y1 - ld * ld
                disc = max(0.0, b * b - 4 * a * c)
                tt = (-b + math.sqrt(disc)) / (2 * a) if a > 1e-9 else 1.0
                tt = max(0.0, min(1.0, tt))
                return x1 + tt * fx, y1 + tt * fy
        return pts[-1]


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        p = self.declare_parameter
        cones_topic = p('cones_topic', '/cones/opencv').value
        cmd_topic = p('cmd_topic', '/cmd_vel').value
        self.throttle = p('throttle', 0.3).value
        self.search_throttle = p('search_throttle', 0.3).value
        self.steering_sign = p('steering_sign', -1.0).value     # real car: positive = right
        self.steering_trim = p('steering_trim', 0.0).value      # real car pulls right -> about -0.05
        self.watchdog_s = p('watchdog_s', 0.5).value
        self.btn_start = p('button_start', 3).value             # triangle
        self.btn_stop = p('button_stop', 1).value               # circle
        self.btn_manual = p('button_manual', 9).value           # L1 = human takes over
        autostart = p('autostart', False).value                 # True for the simulator
        kw = dict(
            wheelbase=p('wheelbase', 0.33).value,
            turning_radius=p('turning_radius', 0.63).value,
            offset=p('offset', 0.55).value,
            first_side_left=p('first_side_left', True).value,
            stop_dist=p('stop_dist', 0.7).value,
            new_cone_jump=p('new_cone_jump', 0.8).value,
            search_steer=p('search_steer', 0.4).value,
            wall_dist=p('wall_dist', 1.0).value,
            pass_max_x=p('pass_max_x', 1.0).value,
            pass_cooldown=p('pass_cooldown', 8).value,
            hold_frames=p('hold_frames', 4).value,
            ignore_frames=p('ignore_frames', 12).value,
            clear_frames=p('clear_frames', 25).value,
            pink_hold=p('pink_hold', 15).value,
            pink_lost_stop=p('pink_lost_stop', 1.0).value,
            steer_gain=p('steer_gain', 1.0).value,
            min_lookahead=p('min_lookahead', 0.0).value,
            clear_always=p('clear_always', False).value,
            side_check=p('side_check', False).value,
            side_max_frames=p('side_max_frames', 25).value,
            pink_after=p('pink_after', 0).value)
        self.mode = p('mode', 'classic').value          # 'classic' = old logic, 'track' = cone map + path
        if self.mode == 'track':
            self.brain = TrackBrain(dr_speed=p('dr_speed', 0.4).value, switch_x=p('switch_x', 0.35).value,
                                    lookahead=p('lookahead', 0.8).value, beside=p('beside', 0.0).value,
                                    min_pass_travel=p('min_pass_travel', 1.0).value, virtual_max=p('virtual_max', 1.5).value,
                                    max_steer_cmd=p('max_steer_cmd', 0.8).value, **kw)
        else:
            self.brain = SlalomBrain(**kw)
        self.last_cones_t = None
        self.lidar_yaw = math.radians(p('lidar_yaw_deg', 5.0).value)   # same as fusion yaw_offset_deg
        self.box_back = p('side_box_back', 0.3).value     # m behind the LiDAR
        self.box_front = p('side_box_front', 0.4).value   # m ahead of the LiDAR
        self.box_in = p('side_box_in', 0.12).value        # m, inner edge (ignore own car)
        self.box_out = p('side_box_out', 0.6).value       # m, outer edge sideways
        self.side_min_pts = p('side_min_points', 3).value
        self.last_blocked = None
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self.on_scan, qos_profile_sensor_data)

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(ConeArray, cones_topic, self.on_cones, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.1, self.check_watchdog)
        self.last_msg_time = None
        self.zeros_to_send = 0
        self.last_state = None
        if autostart:
            self.brain.start()
        self.get_logger().info(f'Driver ready on {cones_topic} -> {cmd_topic}. '
                               f'{"Driving." if autostart else "Press triangle to start."}')

    # ---------- inputs ----------
    def on_joy(self, msg):
        def pressed(i):
            return 0 <= i < len(msg.buttons) and msg.buttons[i] == 1
        if pressed(self.btn_manual) or pressed(self.btn_stop):
            if self.brain.state not in ('IDLE', 'DONE'):
                self.get_logger().warn('STOP by controller')
                self.brain.stop()
                self.zeros_to_send = 5
        elif pressed(self.btn_start) and self.brain.state in ('IDLE', 'DONE'):
            self.get_logger().info('START by controller')
            self.brain.start()

    def on_scan(self, scan):
        left = right = 0
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
                continue
            a = scan.angle_min + i * scan.angle_increment - self.lidar_yaw
            x, y = r * math.cos(a), r * math.sin(a)
            if -self.box_back < x < self.box_front:
                if self.box_in < y < self.box_out:
                    left += 1
                elif -self.box_out < y < -self.box_in:
                    right += 1
        self.brain.blocked = {1: left >= self.side_min_pts, -1: right >= self.side_min_pts}
        b = (self.brain.blocked[1], self.brain.blocked[-1])
        if b != self.last_blocked:
            self.get_logger().info(f'LiDAR side: left {"BLOCKED" if b[0] else "free"} | right {"BLOCKED" if b[1] else "free"}')
            self.last_blocked = b

    def on_cones(self, msg):
        self.last_msg_time = self.get_clock().now()
        cones = [(c.color, c.x, c.y) for c in msg.cones]
        now = self.last_msg_time.nanoseconds * 1e-9
        dt = 0.09 if self.last_cones_t is None else min(0.5, max(0.0, now - self.last_cones_t))
        self.last_cones_t = now
        try:
            steer = self.brain.update(cones, dt) if self.mode == 'track' else self.brain.update(cones)
        except Exception as e:                       # never keep driving with a broken brain
            self.get_logger().error(f'BRAIN ERROR {e!r} -> stop')
            self.brain.stop()
            steer = None
        self.log_state()
        if steer is None:
            if self.zeros_to_send:
                self.send(0.0, 0.0)
                self.zeros_to_send -= 1
            return
        self.zeros_to_send = 5              # if we stop later, send a few zeros
        speed = self.search_throttle if self.brain.state == 'SEARCH' else self.throttle
        self.send(speed, steer)

    def check_watchdog(self):
        if self.brain.state in ('IDLE', 'DONE'):
            return
        now = self.get_clock().now()
        if self.last_msg_time is None or (now - self.last_msg_time).nanoseconds * 1e-9 > self.watchdog_s:
            self.get_logger().warn('WATCHDOG: no cone data -> stop', throttle_duration_sec=1.0)
            self.send(0.0, 0.0)

    # ---------- output ----------
    def send(self, speed, steer_left_positive):
        cmd = Twist()
        cmd.linear.x = float(speed)
        steer = self.steering_sign * steer_left_positive + (self.steering_trim if speed else 0.0)
        cmd.angular.z = float(max(-1.0, min(1.0, steer)))
        self.pub.publish(cmd)

    def log_state(self):
        b = self.brain
        if b.state != self.last_state:
            side = 'left' if b.side > 0 else 'right'
            self.get_logger().info(f'State {b.state} | passed {b.passed} | next side {side} | mode {self.mode}')
            self.last_state = b.state


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.send(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
