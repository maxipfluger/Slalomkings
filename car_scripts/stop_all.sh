#!/bin/bash
pkill -f "lib/slalom/driver"; pkill -f "lib/slalom/fusion"; pkill -f "lib/slalom/detector"
pkill -f ldlidar_ros2_node; pkill -f multi_cam; pkill -f vesc_twist; pkill -f joy_node; pkill -f teleop_twist; pkill -f "ros2 launch"
sleep 2
pkill -9 -f "lib/slalom"; pkill -9 -f ldlidar_ros2_node; pkill -9 -f multi_cam; pkill -9 -f vesc_twist; pkill -9 -f joy_node; pkill -9 -f teleop_twist
echo "Alles gestoppt. Noch aktiv:"
ps aux | grep -v grep | grep -E "multi_cam|vesc|joy_node|teleop|slalom|ldlidar" || echo "  nichts"
