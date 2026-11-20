# XLeRobot ping-pong ball balancing (two arms)

Both SO-101 arms of an [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) hold a table-tennis paddle each and balance a ping-pong ball
on it, at the same time. Each arm senses its ball only with its own wrist camera. The controller is zecice's PPO policy, trained in
MuJoCo and used here unchanged.

**Videos (on X):** [video 1](https://x.com/x_princep/status/2122936066957017416) ·
[video 2](https://x.com/x_princep/status/2122927733626024409)

![Left and right wrist cameras while both arms balance a ping-pong ball](assets/videos/both_arms_wrist.gif)

*What the controller sees: the left and right wrist cameras during the two-arm run, side by side (rotated so up is up; the paddle is
clamped edge-on, the ball sits on top of the blade). Full 25 s clip, from placing the balls to balancing:
[assets/videos/both_arms_wrist.mp4](assets/videos/both_arms_wrist.mp4).*

**Results on the real robot:** both arms balanced at the same time, each on its first attempt, for about 4 minutes (left 249 s,
right 245 s). Pretrained weights, no retraining.

> **Acknowledgement.** This project is built on [zecice/so101-pingpong-balance](https://github.com/zecice/so101-pingpong-balance)
> by **zecice** (MIT): the MuJoCo environment, PPO controller, ball detector + estimator, pretrained weights and the real-arm loop
> all come from that work, and its full commit history is kept here. Please star and cite the original.
> Additional asset credits: SO-101 model files by TheRobotStudio (Apache-2.0), paddle mesh by Samuel Galla and Tim Huber (MIT),
> see `assets/so101/README.md`.

## XLeRobot quick start

Runs on the robot computer (a Jetson Orin here, CPU only). Stop the XLeRobot host/bridge first: these scripts need the arms' serial ports.
Every hardware script takes `--arm left|right` (port, calibration, wrist camera, home file: see `hardware/xlerobot.py`).

```bash
python -m hardware.xlerobot split-calib                       # once per XLeRobot calibration: one SOFollower calibration per arm
python -m hardware.jog --arm right wrist_flex=-12             # lift the wrist clear of the cart
python -m hardware.jog --arm right wrist_roll=0 gripper=40    # centre the roll, open the gripper
python -m hardware.run_real --arm right --grip-only --home /nonexistent  # o, insert the paddle edge-on to the camera, Enter: squeeze
python -m hardware.pose_guide --arm right                     # arm goes limp: pose it upright with the paddle level, Ctrl+C saves home
python -m hardware.jog --arm right --rel wrist_flex=-1        # level with the ball: nudge until it rests, then --save-home
python -m hardware.run_real --arm right --ball white --kp 64 --r-corr 1.2 --grip-load 0   # balance
python -m hardware.torque_off --arm right                     # hold the arm: it goes limp
```
Both arms at once: `LEFT_ARGS="--r-corr 1.15" RIGHT_ARGS="--r-corr 1.2" scripts/run_both.sh --ball white --kp 64 --grip-load 0`.
Checks that move nothing: `hardware.ball_view` (detector on live frames), `hardware.temps` (motor temperatures and fault flags).

What had to change for XLeRobot, and why, is in [docs/XLEROBOT.md](docs/XLEROBOT.md). The original README follows.

---

## Original project: SO-101 ping-pong ball balancing

A low-cost [SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm balances a ping-pong ball on a table-tennis paddle. **The wrist camera is its
only sensor for the ball**; the rest is the arm's own joint readings. The controller is trained with PPO in MuJoCo and runs unchanged on the real arm.

![SO-101 arm balancing a ping-pong ball on a paddle in MuJoCo](assets/demo.gif)

*The trained controller in simulation, with two gentle kicks to the ball.* Real arm: [video on X](https://x.com/zecicc/status/2118380170662863229).

**Results:** 97.5% of episodes balanced for 10 s in randomized sim (latency 0–4 frames, soft servos, noisy ball state); 113/120 in sim with
the full camera pipeline; balances on the real arm.

## How it works

```
wrist camera (640x480 -> 128x96)
  -> detector     orange-blob colour threshold: x, y, radius, visible            perception/detector.py
  -> estimator    MLP on the last 3 detections -> ball position on the paddle    perception/estimator.py  (trained in sim)
  -> FdVel        velocity = filtered finite differences of the positions        sim/env.py
  -> controller   PPO MLP -> 5 joint-position deltas, 30 Hz                      sim/ppo.py, sim/train_controller.py
```

The paddle is clamped edge-on to the wrist camera, so the camera sees it from the side and the ball near one image column.

What made sim-to-real work: **train the controller with the real system's flaws from the start**.
- Control latency randomized over 0–4 frames (0–133 ms), with the last 3 actions in the observation.
- Soft, randomized servo gains (cheap servos sag and lag).
- The ball state the controller sees is noisy like the estimator's output, and its velocity comes from the **same finite-difference filter** the
  robot uses. A controller trained on exact velocity failed completely on camera-derived velocity.

What did **not** work: an end-to-end CNN policy on wrist images (behaviour cloning + DAgger from the PPO controller). It was fine in sim but
barely saw the real ball. A plain colour detector + a tiny estimator replaced it.

## Install

```bash
git clone https://github.com/PrinceP/xlerobot-pingpong-balance && cd xlerobot-pingpong-balance
uv sync                      # sim only
uv sync --extra hardware     # + LeRobot / Feetech drivers for the real arm
```
Uses [uv](https://docs.astral.sh/uv/); `uv sync` installs the exact tested versions from `uv.lock` (pip install is not set up).
Run everything from the repo root. Developed on Linux (laptop with an RTX A1000, 16 GB RAM); the code avoids Linux-only calls, but macOS and
Windows are untested. On macOS the live viewer must be started with `mjpython` instead of `python`.
Training needs ~5 GB of free RAM by default; see the note under training if you have less.

## Try it in sim (no arm needed)

```bash
uv run python -m sim.view_controller --rand                    # live viewer, pretrained controller
uv run python -m sim.eval_policy --episodes 40                 # controller success per latency (true ball state)
uv run python -m perception.estimator --eval weights/estimator.pt --latency 0,2,4 --workers 4 --servo-rand --episodes 40
                                                               # full camera pipeline in sim
```

### Train the controller (~29M steps, ~2 h: sim on 6 CPU cores, network on the GPU)

Default: 6 workers x 4 envs = 24 parallel sims x 128 steps = 3072 samples per iteration.
- Fewer CPU cores: keep the 24 envs on fewer workers (`--workers 3 --envs-per-worker 8`).
- Less RAM: use fewer envs and more steps, to keep the batch size (`--workers 3 --envs-per-worker 4 --steps 256`, ~2.5 GB).

Only the default settings were used for the released weights.

```bash
uv run python -m sim.train_controller --name my_controller --iters 9500 --act-hist 3 --grip-rot -90 --max-latency 4 --home-penalty 0.1 --servo-rand --ball-noise 1.0
```
Add `--wandb` for logging (after `uv sync --extra wandb`, plus `--extra hardware` if you also use the arm). `weights/controller.pt` is the best checkpoint of a run with this command.

### Train the estimator

```bash
uv run python -m perception.estimator --name my_estimator --frames 500000
```
Collects 500k frames of simulated wrist-camera detections while the controller balances with heavy action noise, then trains. The shipped
`weights/estimator.pt` was trained the same way with an earlier controller.

## Real arm

Hardware: SO-101 follower, wrist camera, a standard paddle clamped in the gripper, edge-on to the camera, an orange ball.

**Safety:** the arm moves on its own and stays powered after the scripts exit. Keep clear of it and be ready to press Ctrl+C. To release it, hold the arm and run `hardware.torque_off`.

```bash
uv run lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower_arm   # once, if not calibrated yet
uv run python -m hardware.run_real --grip-only   # type o: the arm lifts, the gripper opens, hold the paddle in the jaws, Enter -> it squeezes
uv run python -m hardware.pose_guide      # once: hold the arm, it goes limp; level the paddle, Ctrl+C -> hardware/home.json
uv run python -m hardware.run_real        # Enter keeps the paddle (o inserts it) -> it goes home, waits for the ball
uv run python -m hardware.torque_off      # arm goes limp (hold it!)
```
If your arm is already calibrated under another name, pass `--id <name>` to the hardware scripts.
The home pose depends on each arm's calibration, so everyone makes their own (`hardware/home_example.json` is mine, for the format).
`--port` defaults to `/dev/ttyACM0` (macOS: `/dev/tty.usbmodem...`, Windows: `COM3` etc.; `lerobot-find-port` tells you).
`--cam` defaults to `/dev/video4`; a camera index like `--cam 0` works on every OS.

Things that mattered on the real arm:
- **Servo P gain 32**, not LeRobot's default 16: at 16 the arm sags and ignores 4–7° commands. `run_real.py` sets it.
- **Level home pose.** A few degrees of tilt and the ball runs off: the policy has no integral action.
- **Gripper:** squeeze by load (~10%), never command "fully closed" on the paddle. A stall trips the Feetech overload protection and the
  motor drops off the bus until a power cycle.
- Measure your own latency with `uv run python -m hardware.measure_latency PORT CAMERA` (after `--grip-only`, paddle in): the controller is trained for 0-4 extra 33 ms steps.

## Layout

| Folder | |
|---|---|
| `sim/` | MuJoCo env, parallel envs, PPO, training, eval, viewer |
| `perception/` | ball detector + estimator (trained in sim, used in sim and on the arm) |
| `hardware/` | real-arm loop and utilities (LeRobot `SOFollower`) |
| `assets/so101/` | arm + paddle + scene MJCF and meshes (see its README for sources and licenses) |
| `weights/` | pretrained controller and estimator |

## License

Code and weights: MIT. SO-101 model files: Apache-2.0 (TheRobotStudio). Paddle mesh: MIT. See `assets/so101/README.md`.
