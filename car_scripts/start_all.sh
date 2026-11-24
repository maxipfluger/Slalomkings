#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
for f in $(find /home/projects -name setup.bash -path "*install*" -not -path "*/ros2_ws/*"); do source "$f"; done
source /home/projects/ros2_ws/install/setup.bash
mkdir -p /home/projects/logs
nohup ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py > /home/projects/logs/base.log 2>&1 &
sleep 8
nohup ros2 run slalom detector > /home/projects/logs/detector.log 2>&1 &
nohup ros2 run ldlidar_ros2 ldlidar_ros2_node --ros-args -p product_name:=LDLiDAR_LD06 -p laser_scan_topic_name:=scan -p point_cloud_2d_topic_name:=pointcloud2d -p frame_id:=base_laser -p port_name:=/dev/$(ls /sys/bus/usb-serial/drivers/cp210x/ | grep ttyUSB | head -1) -p serial_baudrate:=230400 -p laser_scan_dir:=true -p enable_angle_crop_func:=false -p angle_crop_min:=135.0 -p angle_crop_max:=225.0 -p range_min:=0.02 -p range_max:=12.0 > /home/projects/logs/lidar.log 2>&1 &
sleep 3
# 24.11.: fusion = distance ONLY from LiDAR, camera = colour + direction, LiDAR tracks cones outside the camera view
# TRACK_RANGE = how far the LiDAR may look (2.0 while it hits the floor at ~2.2 m; 3.5 when it is level)
TRACK_RANGE=${TRACK_RANGE:-2.0}
nohup ros2 run slalom fusion --ros-args -p lidar_track:=true -p track_yaw_deg:=-1.0 -p track_max_range:=$TRACK_RANGE -p yaw_offset_deg:=5.0 > /home/projects/logs/fusion.log 2>&1 &
nohup ros2 run slalom ai_detector > /home/projects/logs/ai.log 2>&1 &
sleep 3
grep -q "Opened joystick" /home/projects/logs/base.log && echo "Controller OK" || echo "ACHTUNG: Controller nicht gefunden"
grep -q "No devices found" /home/projects/logs/base.log && echo "ACHTUNG: Kamera nicht gefunden" || echo "Kamera OK"
echo "Gestartet. Logs: /home/projects/logs/"
