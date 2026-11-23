import sys
p = sys.argv[1]
s = open(p).read()
if "class TrackBrain" in s:
    print("driver.py hat TrackBrain schon"); sys.exit(0)
if "side_check" not in s:
    print("ZUERST fix_sidecheck.py anwenden!"); sys.exit(1)
TRACK = r'''
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


'''[1:]
reps = [
("class Driver(Node):", TRACK + "class Driver(Node):"),
("        self.brain = SlalomBrain(\n", "        kw = dict(\n"),
("            side_max_frames=p('side_max_frames', 25).value)\n",
 "            side_max_frames=p('side_max_frames', 25).value)\n"
 "        self.mode = p('mode', 'classic').value          # 'classic' = old logic, 'track' = cone map + path\n"
 "        if self.mode == 'track':\n"
 "            self.brain = TrackBrain(dr_speed=p('dr_speed', 0.4).value, switch_x=p('switch_x', 0.35).value,\n"
 "                                    lookahead=p('lookahead', 0.8).value, beside=p('beside', 0.0).value,\n"
 "                                    min_pass_travel=p('min_pass_travel', 1.0).value, virtual_max=p('virtual_max', 1.5).value,\n"
 "                                    max_steer_cmd=p('max_steer_cmd', 0.8).value, **kw)\n"
 "        else:\n"
 "            self.brain = SlalomBrain(**kw)\n"
 "        self.last_cones_t = None\n"),
("        self.box_in = p('side_box_in', 0.05).value", "        self.box_in = p('side_box_in', 0.12).value"),
("        steer = self.brain.update(cones)\n",
 "        now = self.last_msg_time.nanoseconds * 1e-9\n"
 "        dt = 0.09 if self.last_cones_t is None else min(0.5, max(0.0, now - self.last_cones_t))\n"
 "        self.last_cones_t = now\n"
 "        try:\n"
 "            steer = self.brain.update(cones, dt) if self.mode == 'track' else self.brain.update(cones)\n"
 "        except Exception as e:                       # never keep driving with a broken brain\n"
 "            self.get_logger().error(f'BRAIN ERROR {e!r} -> stop')\n"
 "            self.brain.stop()\n"
 "            steer = None\n"),
("            self.get_logger().info(f'State {b.state} | passed {b.passed} | next side {side}')",
 "            self.get_logger().info(f'State {b.state} | passed {b.passed} | next side {side} | mode {self.mode}')"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: TrackBrain (mode track) eingebaut")
