#!/usr/bin/env bash
# Both XLeRobot arms balance at once: one run_real process per arm, side by side in tmux session "balance".
#   scripts/run_both.sh [extra run_real args for both arms, e.g. --ball white --kp 64]
#   LEFT_ARGS="--r-corr 0.95" RIGHT_ARGS="" scripts/run_both.sh ...   # per-arm extras
# Each pane asks its own questions (paddle in? ...). Ctrl+C in a pane stops that arm; torque stays ON (hold the arm, then
#   python -m hardware.torque_off --arm left|right).
# Stop the XLeRobot bridge/host first: both arms' serial ports must be free.
set -euo pipefail
cd "$(dirname "$0")/.."
PY="${PY:-/home/orin/miniconda3/envs/xlerobot/bin/python}"
EXTRA="$*"
if pgrep -f '[x]lerobot_web_bridge|[x]lerobot_host' >/dev/null; then
  echo "The XLeRobot bridge/host is running and holds the serial ports. Stop it first (make stop)." >&2; exit 1
fi
for arm in left right; do
  [ -f "hardware/home_${arm}.json" ] || { echo "missing hardware/home_${arm}.json: run pose_guide --arm ${arm} first" >&2; exit 1; }
done
tmux kill-session -t balance 2>/dev/null || true
tmux new-session -d -s balance -n arms "sg dialout -c '$PY -m hardware.run_real --arm left $EXTRA ${LEFT_ARGS:-}'; echo 'left exited - Enter'; read -r _"
tmux split-window -h -t balance:arms "sg dialout -c '$PY -m hardware.run_real --arm right $EXTRA ${RIGHT_ARGS:-}'; echo 'right exited - Enter'; read -r _"
exec tmux attach -t balance
