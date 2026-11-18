# Slalomkings – Team 13, MAE 148

Autonomous cone slalom (yellow cones, stop at pink cone, red/blue cone walls for orientation). Everything is ROS2.

## Packages
| Package | What |
|---|---|
| `slalom_interfaces` | Our messages: `Cone`, `ConeArray` (do not change without telling the team!) |
| `slalom` | Our Python nodes (detector, controller, ...) |

## Topics (single source of truth)
| Topic | Type | Who publishes |
|---|---|---|
| `/cones/opencv` | `slalom_interfaces/ConeArray` | OpenCV detector |
| `/cones/ai` | `slalom_interfaces/ConeArray` | our AI model |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR (existing) |
| steering/throttle: TBD (G3) | | controller |

Coordinates: meters, `x` forward, `y` left (right = negative).

## Setup on the car (inside the robocar container)
    cd /home/projects/ros2_ws/src
    git clone https://github.com/maxipfluger/Slalomkings.git
    cd .. && colcon build --packages-select slalom_interfaces slalom
    source install/setup.bash

## Setup on a laptop – Option A: Linux VM with ROS2
    mkdir -p ~/ws/src && cd ~/ws/src
    git clone https://github.com/maxipfluger/Slalomkings.git
    cd ~/ws && colcon build
    source install/setup.bash

## Setup on a laptop – Option B: Docker (Windows PowerShell)
    git clone https://github.com/maxipfluger/Slalomkings.git
    cd Slalomkings
    docker run -it --rm -v "${PWD}:/ws/src/Slalomkings" ros:foxy bash
    # inside the container:
    cd /ws && colcon build && source install/setup.bash

## Quick test (no car needed)
    ros2 interface show slalom_interfaces/msg/ConeArray
    ros2 topic pub /cones/opencv slalom_interfaces/msg/ConeArray "{cones: [{color: 0, x: 1.5, y: 0.0, confidence: 1.0}]}"
    ros2 topic echo /cones/opencv      # in a second terminal

## Team rules
1. `git pull` before you start, `git push` when you stop.
2. Only push code that builds (`colcon build` without errors).
3. Each node = its own file. Don't edit someone else's node without telling them.
4. Message or topic changes only after telling the group chat.
5. ROS_DOMAIN_ID = 13 everywhere (car and laptops).
6. Results (topic list, steering calibration, turning radius) go into `docs/`.
7. No big files in git (rosbags, videos, datasets).
