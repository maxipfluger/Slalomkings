import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from slalom_interfaces.msg import ConeArray
from std_msgs.msg import Bool


class ConeFusionNode(Node):
    def __init__(self):
        super().__init__('cone_fusion')

        self.declare_parameter('opencv_topic', '/cones/opencv')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('fused_topic', '/cones/fused')
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('invert_lidar', False)
        self.declare_parameter('search_window_deg', 4.0)
        self.declare_parameter('max_range_diff', 1.5)
        self.declare_parameter('safety_stop_dist', 0.30)
        self.declare_parameter('safety_fov_deg', 20.0)

        self.latest_scan = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.cone_sub = self.create_subscription(
            ConeArray,
            self.get_parameter('opencv_topic').value,
            self.cones_callback,
            10,
        )

        self.fused_pub = self.create_publisher(
            ConeArray,
            self.get_parameter('fused_topic').value,
            10,
        )
        self.stop_pub = self.create_publisher(Bool, '/safety_stop', 10)

        self.get_logger().info(
            f"Cone fusion active: {self.get_parameter('opencv_topic').value} + "
            f"{self.get_parameter('scan_topic').value} -> {self.get_parameter('fused_topic').value}"
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.check_safety_stop(msg)

    def check_safety_stop(self, scan: LaserScan):
        stop_dist = float(self.get_parameter('safety_stop_dist').value)
        fov_rad = math.radians(float(self.get_parameter('safety_fov_deg').value))
        yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))

        obstacle_close = False
        for i, r in enumerate(scan.ranges):
            if r <= scan.range_min or r >= scan.range_max or math.isnan(r) or math.isinf(r):
                continue
            beam_angle = scan.angle_min + i * scan.angle_increment - yaw_offset
            beam_angle = math.atan2(math.sin(beam_angle), math.cos(beam_angle))
            if abs(beam_angle) <= fov_rad and r < stop_dist:
                obstacle_close = True
                break

        stop_msg = Bool()
        stop_msg.data = obstacle_close
        self.stop_pub.publish(stop_msg)

    def cones_callback(self, msg: ConeArray):
        if self.latest_scan is None:
            self.fused_pub.publish(msg)
            return

        fused_msg = copy.deepcopy(msg)
        scan = self.latest_scan

        yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        invert = bool(self.get_parameter('invert_lidar').value)
        window_rad = math.radians(float(self.get_parameter('search_window_deg').value))
        max_diff = float(self.get_parameter('max_range_diff').value)

        for cone in fused_msg.cones:
            cam_angle = math.atan2(cone.y, cone.x)
            cam_dist = math.hypot(cone.x, cone.y)
            lidar_angle = (-cam_angle if invert else cam_angle) + yaw_offset

            valid_ranges = []
            for i, r in enumerate(scan.ranges):
                if r <= scan.range_min or r >= scan.range_max or math.isnan(r) or math.isinf(r):
                    continue
                beam_angle = scan.angle_min + i * scan.angle_increment
                angle_diff = math.atan2(
                    math.sin(beam_angle - lidar_angle),
                    math.cos(beam_angle - lidar_angle),
                )
                if abs(angle_diff) <= window_rad:
                    valid_ranges.append(r)

            if valid_ranges:
                best_r = min(valid_ranges)
                if best_r - cam_dist < max_diff:  # lidar nearer -> always trust lidar
                    cone.x = float(best_r * math.cos(cam_angle))
                    cone.y = float(best_r * math.sin(cam_angle))

        self.fused_pub.publish(fused_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
