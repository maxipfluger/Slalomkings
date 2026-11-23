#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
for f in $(find /home/projects -name setup.bash -path "*install*" -not -path "*/ros2_ws/*"); do source "$f"; done
source /home/projects/ros2_ws/install/setup.bash
ros2 run slalom driver --ros-args -p cones_topic:=/cones/fused -p steering_trim:=-0.07 -p first_side_left:=true -p pass_max_x:=1.5 -p search_steer:=0.8 -p offset:=0.45 -p min_lookahead:=0.7 -p clear_frames:=12 -p wall_dist:=0.0 -p new_cone_jump:=0.5 -p side_check:=false
