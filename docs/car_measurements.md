# Car measurements (G3 / G4, Nov 18)

## Control (G3)
- ROS2 version on the car: **Jazzy**
- Drive topic: `/cmd_vel` (`geometry_msgs/Twist`, read by `vesc_twist_node`)
  - `linear.x`: -1..1, positive = forward. Smooth from about **0.25 on the stand / 0.3 on the floor** (below: jerky).
  - `angular.z`: -1..1, **negative = LEFT, positive = RIGHT**, proportional.
- The car **keeps driving with the last command** if no new commands arrive → our driver has a watchdog.
- Emergency stop: tap **L1** (teleop sends one zero command). Does not work while another node keeps publishing.
- Manual test without staying at the laptop:
  `ros2 topic pub -r 20 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`

## Changes made in the course packages (inside the car container)
- `ucsd_robocar_nav2_pkg/config/node_config.yaml` (src + install): `camera_nav: 0` (lane following off), `manual_joy_control_launch: 1`
- `ucsd_robocar_control2_pkg/config/manual_joy_control_config.yaml` (src + install): `enable_button: 9` (L1), `axis_angular yaw: 2`, `scale_angular yaw: -1.0`
- Jazzy joy uses SDL button numbers: L1 = 9, R1 = 10, cross = 0, circle = 1, square = 2, triangle = 3, share = 4

## Calibration (G4)
- Turning radius at full steering: **about 0.63 m** (circle diameter 125 cm)
- Steering 0 pulls **to the right** → `steering_trim` parameter in our driver (about -0.05 to -0.1, tune in T0)
- Cone spacing: 1.5 m is at the limit (full steering at every cone), **2 m recommended**

## Camera
- `/camera/color/image_raw`: 11 Hz, 1280x800, bgr8. No depth topic yet.
- ros2 bag: about 1.9 GB per minute. JPG via `save_images`: about 0.23 MB per image. About 88 GB free on the Pi.
