"""Minimal 2D simulator for the cone slalom.

The car is a simple bicycle model. The node listens to the same drive command
the real car gets and publishes the cones the camera would see, in exactly the
same format as our real detector (ConeArray on /cones/opencv).
So the state machine and the controller can be tested at home without the car.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from slalom_interfaces.msg import Cone, ConeArray

COLOR_NAMES = {Cone.YELLOW: 'gold', Cone.ORANGE: 'orange', Cone.PINK: 'magenta',
               Cone.RED: 'red', Cone.BLUE: 'blue'}


def make_course(n_cones, spacing, first_x, wall_offset, wall_spacing):
    """Yellow cones on a line along x, pink stop cone at the end, red wall left, blue wall right."""
    cones = [(Cone.YELLOW, first_x + i * spacing, 0.0) for i in range(n_cones)]
    end_x = first_x + n_cones * spacing
    cones.append((Cone.PINK, end_x, 0.0))
    x = 0.0
    while x <= end_x + 0.5:
        cones.append((Cone.RED, x, wall_offset))
        cones.append((Cone.BLUE, x, -wall_offset))
        x += wall_spacing
    return cones


def step(x, y, yaw, speed, steer, wheelbase, dt):
    """One time step of the bicycle model. steer in rad, positive = left."""
    x += speed * math.cos(yaw) * dt
    y += speed * math.sin(yaw) * dt
    yaw += speed / wheelbase * math.tan(steer) * dt
    return x, y, yaw


def visible_cones(cones, x, y, yaw, fov, max_range):
    """Cones inside the camera view, in car coordinates (x forward, y left)."""
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    for color, cx, cy in cones:
        dx, dy = cx - x, cy - y
        fx = c * dx + s * dy
        fy = -s * dx + c * dy
        if fx <= 0.05 or math.hypot(fx, fy) > max_range:
            continue
        if abs(math.atan2(fy, fx)) > fov / 2:
            continue
        out.append((color, fx, fy))
    return out


class Sim(Node):
    def __init__(self):
        super().__init__('sim')
        p = self.declare_parameter
        self.wheelbase = p('wheelbase', 0.33).value             # m
        self.turning_radius = p('turning_radius', 1.0).value    # m, from G4
        self.max_speed = p('max_speed', 1.0).value              # m/s at linear.x = 1
        self.fov = math.radians(p('fov_deg', 70.0).value)       # camera field of view
        self.max_range = p('max_range', 3.0).value              # m
        cmd_topic = p('cmd_topic', '/cmd_vel').value            # TODO after G3: real topic name
        cones_topic = p('cones_topic', '/cones/opencv').value
        self.plot = p('plot', True).value
        self.cones = make_course(p('n_cones', 5).value, p('spacing', 1.5).value,
                                 p('first_x', 1.5).value, p('wall_offset', 1.0).value,
                                 p('wall_spacing', 0.7).value)

        self.max_steer = math.atan(self.wheelbase / self.turning_radius)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.speed, self.steer = 0.0, 0.0
        self.trace = []
        self.dt = 0.05

        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.pub = self.create_publisher(ConeArray, cones_topic, 10)
        self.create_timer(self.dt, self.tick)
        self.ticks = 0
        if self.plot:
            self.setup_plot()
        self.get_logger().info(
            f'Sim running: listening on {cmd_topic}, publishing {cones_topic}, '
            f'max steer {math.degrees(self.max_steer):.1f} deg')

    def on_cmd(self, msg):
        # Assumption until G3: linear.x = throttle -1..1, angular.z = steering -1..1 (positive = left)
        self.speed = max(-1.0, min(1.0, msg.linear.x)) * self.max_speed
        self.steer = max(-1.0, min(1.0, msg.angular.z)) * self.max_steer

    def tick(self):
        self.x, self.y, self.yaw = step(self.x, self.y, self.yaw, self.speed,
                                        self.steer, self.wheelbase, self.dt)
        self.trace.append((self.x, self.y))
        for color, cx, cy in self.cones:
            if math.hypot(cx - self.x, cy - self.y) < 0.15:
                self.get_logger().warn(f'HIT a cone at ({cx:.1f}, {cy:.1f})', throttle_duration_sec=1.0)

        msg = ConeArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for color, fx, fy in visible_cones(self.cones, self.x, self.y, self.yaw, self.fov, self.max_range):
            cone = Cone()
            cone.color, cone.x, cone.y, cone.confidence = color, float(fx), float(fy), 1.0
            msg.cones.append(cone)
        self.pub.publish(msg)

        self.ticks += 1
        if self.plot and self.ticks % 4 == 0:
            self.update_plot()

    def setup_plot(self):
        import matplotlib.pyplot as plt
        self.plt = plt
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        for color, cx, cy in self.cones:
            self.ax.plot(cx, cy, '^', color=COLOR_NAMES[color], markersize=10)
        self.trace_line, = self.ax.plot([], [], 'k-', linewidth=1)
        self.car_arrow = None
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.5, max(c[1] for c in self.cones) + 1.0)
        self.ax.set_ylim(-2.0, 2.0)
        self.ax.grid(True)

    def update_plot(self):
        xs, ys = zip(*self.trace)
        self.trace_line.set_data(xs, ys)
        if self.car_arrow:
            self.car_arrow.remove()
        self.car_arrow = self.ax.arrow(self.x, self.y, 0.3 * math.cos(self.yaw), 0.3 * math.sin(self.yaw),
                                       head_width=0.1, color='green')
        self.plt.pause(0.001)


def main():
    rclpy.init()
    node = Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

