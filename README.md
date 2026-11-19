# Slalomkings – Team 13, MAE 148

Autonomous cone slalom (yellow cones, stop at pink cone, red/blue cone walls for orientation). Everything is ROS2. The car runs **ROS2 Jazzy**.

## Packages
| Package | What |
|---|---|
| `slalom_interfaces` | Our messages: `Cone`, `ConeArray` (do not change without telling the team!) |
| `slalom` | Our Python nodes: `sim`, `driver`, `save_images` |

## Topics (single source of truth)
| Topic | Type | Who publishes |
|---|---|---|
| `/cones/opencv` | `slalom_interfaces/ConeArray` | OpenCV detector (or the simulator) |
| `/cones/ai` | `slalom_interfaces/ConeArray` | our AI model |
| `/camera/color/image_raw` | `sensor_msgs/Image` | OAK-D camera (11 Hz, 1280x800, bgr8) |
| `/cmd_vel` | `geometry_msgs/Twist` | `driver` (or the PS controller) |
| `/joy` | `sensor_msgs/Joy` | PS controller |

Coordinates of cones: meters, `x` forward, `y` left (right = negative).
`/cmd_vel` on the real car: `linear.x` = throttle -1..1 (smooth from 0.3), `angular.z` = steering -1..1, **LEFT = NEGATIVE**.
The car keeps driving with the last command – always have the controller in your hand.

## Setup on the car (inside the robocar container)
    docker start robocar-team13
    docker exec -it robocar-team13 bash
    source_ros2
    cd /home/projects/ros2_ws/src/Slalomkings && git pull
    cd /home/projects/ros2_ws && colcon build --packages-select slalom_interfaces slalom
    source install/setup.bash

Do NOT use `robocar_docker` (it tries to create a new container). Never delete the container.

## Setup on a laptop – Option A: Linux VM with ROS2
    mkdir -p ~/ws/src && cd ~/ws/src
    git clone https://github.com/maxipfluger/Slalomkings.git
    cd ~/ws && colcon build
    source install/setup.bash

## Setup on a laptop – Option B: Docker (Windows PowerShell)
    git clone https://github.com/maxipfluger/Slalomkings.git
    cd Slalomkings
    docker run -it --rm -v "${PWD}:/ws/src/Slalomkings" ros:jazzy bash
    # inside the container:
    cd /ws && colcon build && source install/setup.bash

## Simulator + driver (no car needed)
Use a different domain id at home so you never talk to the real car:

    export ROS_DOMAIN_ID=42
    ros2 run slalom sim                                         # terminal 1
    ros2 run slalom driver --ros-args -p autostart:=true        # terminal 2

## Driving the real car
- Driving mode: `ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py`
- PS controller: hold **L1**, left stick = throttle, right stick = steering. Emergency stop: tap L1.
- Our driver: `ros2 run slalom driver` → **triangle = start**, **circle or L1 = stop**.

## Saving camera images (for labeling)
    ros2 run slalom save_images /home/projects/bilder/run1 3     # every 3rd frame, Ctrl+C to stop

Get them out of the container (on the Pi, outside the container):
`docker cp robocar-team13:/home/projects/bilder ~/bilder`, then drag `/home/pi/bilder` to your laptop in MobaXterm.

## Team rules
1. `git pull` before you start, `git push` when you stop.
2. Only push code that builds (`colcon build` without errors).
3. Each node = its own file. Don't edit someone else's node without telling them.
4. Message or topic changes only after telling the group chat.
5. ROS_DOMAIN_ID = 13 on the car and when your laptop talks to the car.
6. Results (topic list, steering calibration, turning radius) go into `docs/`.
7. No big files in git (rosbags, videos, datasets).
