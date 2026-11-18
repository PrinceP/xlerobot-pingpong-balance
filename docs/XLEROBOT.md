# Porting to XLeRobot: two arms, two paddles, two balls

Goal: each XLeRobot arm holds a table-tennis paddle by its handle and balances a ping-pong ball, both arms at the same time.

## Robot facts that matter here

| | Left arm | Right arm |
|---|---|---|
| Serial bus (on the Orin) | `/dev/ttyACM0`, motor IDs 1-6 (head motors 7-8 share the bus) | `/dev/ttyACM1`, motor IDs 1-6 (wheels 7-9 share the bus) |
| Wrist camera | `wrist_b` (`/dev/video4`, unconfirmed) | `wrist_a` (`/dev/video2`, confirmed) |

- Compute: Jetson Orin, conda env `xlerobot` (Python 3.12, lerobot 0.7.0, torch 2.x CPU). The controller and estimator are tiny MLPs, so CPU is enough.
- All three cameras share one USB 2.0 hub. 640x480 fails on one wrist camera; 320x240 MJPG works for all three. The pipeline resizes to 128x96
  anyway, so 320x240 costs nothing.
- Each arm process owns its serial port, so the XLeRobot bridge/host must be stopped while balancing (`make stop` in xlerobot-ops).

## Differences from the original setup, and the plan for each

1. **Arm calibration (blocker).** The current XLeRobot calibration has `shoulder_lift` and `elbow_flex` spanning the full encoder circle
   (0-4095), so the usable range ends at the folded rest pose. Recalibrate with the arms in the middle pose (`make calibrate` in xlerobot-ops)
   before anything else.
2. **Calibration format.** The original loop uses LeRobot `SOFollower` (one arm per port, joint values in degrees, calibration file per arm id).
   XLeRobot keeps one combined file (`left_arm_*`, `right_arm_*`, `head_*`, `base_*`). Plan: a small script that splits it into two
   `so_follower` calibration files (`xlerobot_left`, `xlerobot_right`) for motor IDs 1-6 on each port, so `run_real.py` works per arm with
   `--port`, `--id` and `--cam`, unchanged.
3. **Ball colour.** The detector looks for an orange ball. Ours is white (STAG). Options:
   - buy orange 40 mm balls (no code change, matches the tested detector), or
   - add a `--ball white` detector (low saturation, high value). Risk: white walls and the white printed arm parts behind the blade.
   The estimator only sees `[x, y, r, visible]`, so a different detector does not need retraining if its output matches.
4. **Paddle and grip.** Our paddles are standard shakehand paddles with wooden handles. The original clamps the handle in the gripper,
   blade edge-on to the wrist camera, squeezed by load (~10%). Same here. The sim paddle is 90 g. Weigh ours: a heavier paddle may need
   `--kp` tuning or a retrain with the real mass.
5. **Home pose per arm.** `hardware.pose_guide` once per arm (level blade), saved as `hardware/home_left.json` and `hardware/home_right.json`.
6. **Joint signs per arm.** Run `--jog` and `--tilt-test` on each arm. Both arms are the same SO-101 build, but check that neither is mirrored.
7. **Both at once.** Two processes, one per arm (separate port, camera and log). A wrapper starts both and stops both on Ctrl+C.
8. **Dependencies on the Orin.** `sim/env.py` (for `FdVel`) imports `mujoco` and `gymnasium`. Install them into the Orin env
   (`pip install mujoco==3.17.0 gymnasium`), or move `FdVel` to a module without the sim imports.
9. **LeRobot version.** Upstream pins `lerobot<0.7`, the Orin has 0.7.0. Check the `SOFollower` API (`position_p_coefficient`,
   `disable_torque_on_disconnect`, `max_relative_target`) on 0.7.0.

## Order of work

1. Recalibrate both arms (xlerobot-ops `make calibrate`).
2. Split the calibration into per-arm `so_follower` files; check `hardware.torque_off --id xlerobot_left --port /dev/ttyACM0`.
3. Ball detector: orange ball or `--ball white`; check detections on live wrist-camera frames.
4. One arm (right first, its wrist camera is confirmed): `--grip-only`, `pose_guide`, `--jog`, `--tilt-test`, then balance.
5. Left arm, the same steps.
6. Both arms together.

## Safety

The arm moves on its own and stays powered after the scripts exit. Keep clear, keep Ctrl+C ready, hold the arm before `torque_off`.
Never command the gripper fully closed on the handle (stall trips the servo overload protection).
