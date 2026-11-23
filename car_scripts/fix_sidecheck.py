import sys
p = sys.argv[1]
s = open(p).read()
if "side_check" in s:
    print("driver.py hat side_check schon"); sys.exit(0)
reps = [
("from sensor_msgs.msg import Joy\n",
 "from sensor_msgs.msg import Joy, LaserScan\nfrom rclpy.qos import qos_profile_sensor_data\n"),
("min_lookahead=0.0, clear_always=False):",
 "min_lookahead=0.0, clear_always=False, side_check=False, side_max_frames=25):"),
("        self.clear_always = clear_always",
 "        self.side_check = side_check        # LiDAR: only turn to the next side when that side is free\n"
 "        self.side_max_frames = side_max_frames  # max updates to wait for a free side (then turn anyway)\n"
 "        self.blocked = {1: False, -1: False}    # set by the node from /scan: +1 = left, -1 = right\n"
 "        self.clear_always = clear_always"),
("        self.lost_dir = 0              # +1/-1",
 "        self.side_wait = 0\n        self.lost_dir = 0              # +1/-1"),
("        # we are on the -side of the line now -> steer back towards it\n        return self.side * self.search_steer\n",
 "        # we are on the -side of the line now -> steer back towards it\n"
 "        if self.side_check and self.blocked.get(self.side) and self.side_wait < self.side_max_frames:\n"
 "            self.side_wait += 1             # passed cone still beside us on that side -> straight first\n"
 "            return 0.0\n"
 "        self.side_wait = 0\n"
 "        return self.side * self.search_steer\n"),
("            clear_always=p('clear_always', False).value)",
 "            clear_always=p('clear_always', False).value,\n"
 "            side_check=p('side_check', False).value,\n"
 "            side_max_frames=p('side_max_frames', 25).value)\n"
 "        self.lidar_yaw = math.radians(p('lidar_yaw_deg', 5.0).value)   # same as fusion yaw_offset_deg\n"
 "        self.box_back = p('side_box_back', 0.3).value     # m behind the LiDAR\n"
 "        self.box_front = p('side_box_front', 0.4).value   # m ahead of the LiDAR\n"
 "        self.box_in = p('side_box_in', 0.05).value        # m, inner edge (ignore own car)\n"
 "        self.box_out = p('side_box_out', 0.6).value       # m, outer edge sideways\n"
 "        self.side_min_pts = p('side_min_points', 3).value\n"
 "        self.last_blocked = None\n"
 "        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self.on_scan, qos_profile_sensor_data)"),
("    def on_cones(self, msg):\n",
 "    def on_scan(self, scan):\n"
 "        left = right = 0\n"
 "        for i, r in enumerate(scan.ranges):\n"
 "            if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):\n"
 "                continue\n"
 "            a = scan.angle_min + i * scan.angle_increment - self.lidar_yaw\n"
 "            x, y = r * math.cos(a), r * math.sin(a)\n"
 "            if -self.box_back < x < self.box_front:\n"
 "                if self.box_in < y < self.box_out:\n"
 "                    left += 1\n"
 "                elif -self.box_out < y < -self.box_in:\n"
 "                    right += 1\n"
 "        self.brain.blocked = {1: left >= self.side_min_pts, -1: right >= self.side_min_pts}\n"
 "        b = (self.brain.blocked[1], self.brain.blocked[-1])\n"
 "        if b != self.last_blocked:\n"
 "            self.get_logger().info(f'LiDAR side: left {\"BLOCKED\" if b[0] else \"free\"} | right {\"BLOCKED\" if b[1] else \"free\"}')\n"
 "            self.last_blocked = b\n\n"
 "    def on_cones(self, msg):\n"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: side_check eingebaut")
