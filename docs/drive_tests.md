# Drive tests – checklists and result sheets

Fill this in during the car sessions; the results stay in this file (team rule 6).
Plan tasks: safety stop (must-have), T6 full run + backup video (Nov 28), T8 10 runs in a row (Nov 30).

## 0. Before every session (about 5 min)
- [ ] Car and controller batteries charged
- [ ] On the car: `git pull`, then `colcon build --packages-select slalom_interfaces slalom`
- [ ] Quick software check (about 25 s, cannot move the car): `colcon test --packages-select slalom && colcon test-result --verbose`
- [ ] `bash car_scripts/start_all.sh` prints **Controller OK** and **Kamera OK**
- [ ] Second person holds the controller: **tap L1 = emergency stop**

## 1. Safety stop – stand test (must-have "safety stop if sensor data is lost")
Car **on the stand, wheels up**. One yellow cone about 1.5 m in front of the camera.
Start the driver with `bash car_scripts/run_driver.sh`, then press **triangle**: the wheels spin.

| # | Test | Do | Expected | OK? |
|---|---|---|---|---|
| A | Stop button | press **circle** | wheels stop at once | ☐ |
| B | Ctrl+C | restart the driver, triangle, then **Ctrl+C** in the driver terminal | wheels stop, no error printed | ☐ |
| C | Sensor data lost | restart the driver, triangle, then in another terminal `pkill -f lib/slalom/fusion` | wheels stop within about 0.5 s, the driver log shows `WATCHDOG` | ☐ |

- Covering the camera is **not** a sensor-loss test: the detector keeps publishing (empty) cone lists, so the driver searches instead of stopping. Test C cuts the data by stopping the node.
- If the wheels do not stop: tap **L1**, then `bash car_scripts/stop_all.sh`.
- After test C: `bash car_scripts/stop_all.sh`, then `start_all.sh` again before driving.
- Film the three tests with a phone: that is the proof for the must-have.

## 2. T6 – full run + backup video
Course: 5 yellow cones 1.5 m apart, pink stop cone after the last one.
- [ ] Phone video from outside the track, and `ros2 bag record /cones/fused /cmd_vel` (the bag goes to Google Drive, not git)
- [ ] Repeat until one clean full run is on video
- [ ] Upload the video to the shared Google Drive right away

| Try | Cones passed | Stopped at pink? | Hit a cone? | Notes |
|---|---|---|---|---|
| 1 | /5 | | | |
| 2 | /5 | | | |
| 3 | /5 | | | |
| 4 | /5 | | | |
| 5 | /5 | | | |

## 3. T8 – 10 runs in a row
Same setup, **no setting changes between runs**. Success = all 5 cones passed on the correct side, no cone hit, stop in front of the pink cone.

| Run | Start (straight / angled) | Cones passed | Pink stop | Hit | Success | What went wrong |
|---|---|---|---|---|---|---|
| 1 | | /5 | | | | |
| 2 | | /5 | | | | |
| 3 | | /5 | | | | |
| 4 | | /5 | | | | |
| 5 | | /5 | | | | |
| 6 | | /5 | | | | |
| 7 | | /5 | | | | |
| 8 | | /5 | | | | |
| 9 | | /5 | | | | |
| 10 | | /5 | | | | |

**Success rate: \_\_ / 10** (target 8/10)

## 4. What the simulator says
Checked on Nov 23 with the settings of branch `t4-works-23-11-pm` (offset 0.45, min_lookahead 0.7, new_cone_jump 0.5, walls off), merged with main:
- all 14 automatic tests pass
- full course, 5 cones 1.5 m apart, straight start: all 6 checks pass (left/right alternation, no cone closer than 0.15 m, stop 0.4–1.0 m before the pink cone), about 33 s
- The simulator sees every cone perfectly (no flicker, no sunlight, no noise). On the track the same settings gave 4/5 (Nov 23 pm), so a failure on the car is most likely a **detection** problem (flicker, sunlight on pink/red, far cones) rather than the steering logic. Write it down under "What went wrong".
