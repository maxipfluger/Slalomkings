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
  - Controller: L1 or circle -> IDLE + an immediate zero command, triangle -> start
  - Ctrl+C or a crash -> one last zero command before the node exits
  - When not driving, the node sends a few zero commands and then stays
    silent, so the PS controller can drive the car manually.

Steering convention inside this file: positive = LEFT (ROS standard).
The real car uses positive = RIGHT -> parameter steering_sign = -1.
"""
import math
import signal
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from slalom_interfaces.msg import Cone, ConeArray


class SlalomBrain:
    """Pure logic, no ROS -> can be tested without a car or ROS."""

    def __init__(self, wheelbase=0.33, turning_radius=0.63, offset=0.55,
                 first_side_left=True, stop_dist=0.7, new_cone_jump=0.8,
                 search_steer=0.4, wall_dist=1.0, pass_max_x=1.0, pass_cooldown=8, hold_frames=4, ignore_frames=12, clear_frames=25, pink_hold=15, pink_lost_stop=1.0):
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
        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]
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
            self.last_steer = self.pursuit(x, y + self.side * self.offset)
            return self.last_steer

        # yellow cone dropped out for a moment (detection flicker) -> keep the last steering
        if self.last_x is not None and self.missing < self.hold_frames:
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
                if self.pink_recent > 0:     # last yellow cone (pink ahead) is still beside us -> first drive straight past it
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
        return self.side * self.search_steer


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
        self.brain = SlalomBrain(
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
            pink_lost_stop=p('pink_lost_stop', 1.0).value)

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
                self.send(0.0, 0.0)         # stop NOW, even if no cone message comes anymore
                self.zeros_to_send = 5
        elif pressed(self.btn_start) and self.brain.state in ('IDLE', 'DONE'):
            self.get_logger().info('START by controller')
            self.brain.start()

    def on_cones(self, msg):
        self.last_msg_time = self.get_clock().now()
        cones = [(c.color, c.x, c.y) for c in msg.cones]
        steer = self.brain.update(cones)
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
            self.get_logger().info(f'State {b.state} | passed {b.passed} | next side {side}')
            self.last_state = b.state


def main():
    # Ctrl+C only raises a flag, checked by the loop below. Not the defaults, because:
    #  - rclpy's own handler shuts ROS down *before* our cleanup, so the final zero
    #    could not be published ("publisher's context is invalid");
    #  - a KeyboardInterrupt landing inside rclpy's message handling can turn into
    #    a RuntimeError ("Unable to convert call argument") and crash the node.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda signum, frame: stop_requested.set())
    node = Driver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        # The car keeps executing the last command -> always leave a zero behind,
        # also when the node stops because of an unexpected error.
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
