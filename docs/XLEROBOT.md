# Running the balancer on XLeRobot: what changed and why

Both XLeRobot arms hold a standard table-tennis paddle (wooden handle) and balance a white 40 mm ball, each with its own process, serial
port and wrist camera. zecice's controller and estimator weights are used unchanged. Result: both arms balanced at the same time for
about 4 minutes (left 249 s, right 245 s). These are the things that had to change, in the order we hit them.

## Robot

| | Left arm | Right arm |
|---|---|---|
| Serial bus | `/dev/ttyACM0`, motor IDs 1-6 (head motors 7-8 share the bus) | `/dev/ttyACM1`, motor IDs 1-6 (wheels 7-9 share the bus) |
| Wrist camera | USB port `2.4` (`/dev/v4l/by-path/...usb-0:2.4:1.0-video-index0`) | USB port `2.2` |
| Calibration id | `xlerobot_left` | `xlerobot_right` |

Robot computer: Jetson Orin, Python 3.12, lerobot 0.7.0, torch on CPU. The MLPs are tiny; CPU is plenty.

## 1. Calibration

- **Middle pose matters.** A first XLeRobot calibration done with the arms folded at rest gave `shoulder_lift` and `elbow_flex` a range of
  0-4095 (the whole encoder circle), so the usable range ended at the rest pose. Recalibrate with each arm in the middle pose (upper arm up,
  forearm forward, wrist straight).
- **One file per arm.** XLeRobot keeps one combined calibration file; the balancer uses LeRobot's `SOFollower` (one arm, IDs 1-6).
  `python -m hardware.xlerobot split-calib` copies the values unchanged into `so_follower/xlerobot_left.json` and `xlerobot_right.json`,
  so they match what is in the motors and no recalibration prompt appears.

## 2. Wrist cameras (Innomaker U20CAM, three on one USB 2.0 hub)

- All three report the same serial number: select them by USB port (`/dev/v4l/by-path`), not by id or `/dev/videoN`.
- **No 320x240 mode.** OpenCV silently falls back to 640x360 (16:9), which would distort the 4:3 image the estimator was trained on. Use
  640x480 MJPG.
- **Buffer size 1 halves the frame rate** on these cameras (15 fps). Buffer 2 gives 29.6 fps on both wrist cameras at once.
- Auto exposure's dynamic frame rate is switched off (`exposure_dynamic_framerate=0`) so room light cannot lower the frame rate.

The camera-to-paddle view matches zecice's sim closely once the paddle is clamped edge-on: blade as a vertical line near the image centre,
ball on the image-right side at about x +0.2, y -0.2.

## 3. White ball

The detector looks for an orange ball; ours is white. `--ball white` thresholds low saturation and high brightness, which also matches white
walls, the wooden handle, printed parts and skin. What made it work:

- **Region of interest:** only the strip right of the blade line and above the handle, where a ball on the blade can be.
- **Saturation up to 120:** the red rubber tints the ball pink. With a strict threshold (S<60) only a crescent of the ball was found, its
  size shrank, and the ball was "lost" while it sat near the centre: most short attempts ended that way.
- **Size calibration (`--r-corr`):** the estimator turns blob size into position along the blade (5-10% of size is ~1 cm). With the ball
  resting at the centre, choose `--r-corr` so the estimate reads (0, 0). Ours: left 1.15, right 1.2.

An orange ball would avoid all of this.

## 4. Paddle grip and home pose

- Clamp the handle deep in the jaws, blade edge-on to the wrist camera, the ball face on the camera's image-right side. Colour does not matter.
- The wrist must be lifted ~90 deg (`hardware.jog`) before inserting the paddle: XLeRobot's cart is in the way at rest.
- **Level the home pose with the ball.** Posed by eye, the blade was 9-10 deg off on both arms. Place the ball, nudge `wrist_flex` /
  `wrist_roll` 1-2 deg at a time (`hardware.jog --rel`, which closes the loop on the readings), save when the ball rests. Do this with the
  paddle clamped exactly as it will run: a re-clamp or an extra squeeze tips a round handle in the jaws by a degree or two.
- Start runs with `--grip-load 0` (no extra squeeze) once the paddle is clamped.

## 5. Servos

- **P gain 64.** At zecice's 32, `shoulder_lift` moved only 15% of a 3 deg upward step on our arm (paddle and longer lever); at 64 every joint
  tracks within 6% (`hardware.step_response`).
- **Heat.** A home pose with the upper arm far forward made the left shoulder reach 71 C and trip its overheat cut-out (status flag 4); the
  arm dropped. Keep the upper arm near upright (shoulder_lift near 0), release torque between runs, and let `run_real` stop at 65 C
  (`--max-temp`). `hardware.temps` shows temperatures and fault flags.
- `run_real` refuses to start if a joint cannot reach home within 5 deg (blocked, or a motor without torque).

## 6. Two arms at once

- One process per arm (`scripts/run_both.sh`). Each process ran ~20 busy torch threads; with two processes they starved each other.
  `run_real` now uses one thread.
- Control starts only when the ball rests near the centre for 0.5 s (`--start-frames/--start-radius/--start-speed`). Starting while the hand
  placing the ball was still in view gave a wrong first estimate, a jerk, and a ball thrown toward the tip.
- On start, `run_real` keeps the goals the arm already holds (with sag compensation), so it does not droop and roll a resting ball off.
- Logs and recordings are named per arm (`logs/real_left_*.npz`, `logs/real_right_*.mp4`).

## Not done yet

- The tilt test (`--tilt-test`) was inconclusive on our arm: with a 3 deg tilt the residual level error dominated. Balancing works, so the
  joint signs are right.
- The sim wrist camera is zecice's (fovy 62); it was not refitted to the Innomaker camera. The estimator works with the size correction.
- A retrain with our paddle mass and camera could improve the 2-3 cm steady offset seen on the right arm.
