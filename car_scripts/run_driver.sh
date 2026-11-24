#!/bin/bash
# Start the slalom driver. The car drives only after TRIANGLE on the controller.
#   bash run_driver.sh                -> normal settings
#   bash run_driver.sh genai.yaml     -> settings from the GenAI tool (Yilin) override the normal ones
source /opt/ros/$ROS_DISTRO/setup.bash
for f in $(find /home/projects -name setup.bash -path "*install*" -not -path "*/ros2_ws/*"); do source "$f"; done
source /home/projects/ros2_ws/install/setup.bash
EXTRA=""
if [ -n "$1" ]; then
  echo "Lade Parameter aus $1:"; cat "$1"
  EXTRA="--params-file $1"
fi
# 24.11. LiDAR tracking: passed cone disappears beside the car (x<0) -> ignore_frames 0, offset = real distance now
ros2 run slalom driver --ros-args -p cones_topic:=/cones/fused -p steering_trim:=-0.07 -p first_side_left:=true \
  -p pass_max_x:=1.5 -p search_steer:=0.8 -p offset:=0.5 -p min_lookahead:=0.7 -p clear_frames:=6 \
  -p wall_dist:=0.0 -p new_cone_jump:=0.5 -p side_check:=false -p ignore_frames:=0 $EXTRA
