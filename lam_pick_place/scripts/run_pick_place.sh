#!/usr/bin/env bash
set -euo pipefail

# One-shot pipeline runner for lam_pick_place.
# - Builds a ROS 2 workspace (or reuses existing)
# - Sources the workspace
# - Starts controller and manager via ros2 run
#
# Usage:
#   bash lam_pick_place/scripts/run_pick_place.sh \
#     --ws "~/ws_lam" \
#     --ros-setup "/opt/ros/humble/setup.bash" \
#     [--clean] \
#     [--num-runs 20] [--rate 50.0] [--log-dir trajectories] \
#     [--publish-topic /Left/joint_command] [--input-traj input_trajectory]
#
# Notes:
# - Ensure Isaac Sim 5.0 is running with omni.isaac.ros2_bridge enabled
#   and the Panda wired to the publish topic (default /Left/joint_command).
# - This script uses ros2 run directly (no libexec dependency).

WS_DIR=${WS_DIR:-"$HOME/ws_lam"}
ROS_SETUP=${ROS_SETUP:-"/opt/ros/humble/setup.bash"}
CLEAN=0
NUM_RUNS=20
RATE=50.0
LOG_DIR="trajectories"
PUBLISH_TOPIC="/Left/joint_command"
INPUT_TRAJ_TOPIC="input_trajectory"

print_usage() {
  cat <<EOF
Usage: $0 [options]
  --ws PATH                Workspace directory (default: $WS_DIR)
  --ros-setup PATH         ROS 2 setup.bash (default: $ROS_SETUP)
  --clean                  Remove build/install/log before building
  --num-runs N             Number of pick-place runs (default: $NUM_RUNS)
  --rate HZ                Publish/trajectory rate (default: $RATE)
  --log-dir PATH           Output CSV dir (default: $LOG_DIR)
  --publish-topic TOPIC    JointState topic to publish (default: $PUBLISH_TOPIC)
  --input-traj TOPIC       JointTrajectory input topic (default: $INPUT_TRAJ_TOPIC)
  -h, --help               Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws) WS_DIR="$2"; shift 2;;
    --ros-setup) ROS_SETUP="$2"; shift 2;;
    --clean) CLEAN=1; shift;;
    --num-runs) NUM_RUNS="$2"; shift 2;;
    --rate) RATE="$2"; shift 2;;
    --log-dir) LOG_DIR="$2"; shift 2;;
    --publish-topic) PUBLISH_TOPIC="$2"; shift 2;;
    --input-traj) INPUT_TRAJ_TOPIC="$2"; shift 2;;
    -h|--help) print_usage; exit 0;;
    *) echo "Unknown option: $1" >&2; print_usage; exit 1;;
  esac
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "Error: '$1' not found in PATH" >&2; exit 1; }
}

echo "[1/5] Checking prerequisites…"
require_cmd bash
require_cmd python3
require_cmd ros2
require_cmd colcon

echo "[2/5] Sourcing ROS 2 from: $ROS_SETUP"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Error: ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

PKG_SRC_DIR="$(cd "$(dirname "$0")/.." && pwd)"  # …/lam_pick_place
REPO_ROOT="$(cd "$PKG_SRC_DIR/.." && pwd)"

echo "[3/5] Preparing workspace: $WS_DIR"
mkdir -p "$WS_DIR/src"
if [[ ! -d "$WS_DIR/src/lam_pick_place" ]]; then
  echo "- Copying package into workspace…"
  rsync -a --delete "$PKG_SRC_DIR" "$WS_DIR/src/"
else
  echo "- Updating package in workspace…"
  rsync -a --delete "$PKG_SRC_DIR/" "$WS_DIR/src/lam_pick_place/"
fi

cd "$WS_DIR"
if [[ $CLEAN -eq 1 ]]; then
  echo "- Cleaning build/install/log"
  rm -rf build install log
fi

echo "[4/5] Building (ament_python)…"
colcon build --symlink-install --packages-select lam_pick_place

echo "- Sourcing workspace setup.bash"
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

echo "[5/5] Launching controller + manager (python -m, from source)…"

# Kill background processes on exit
cleanup() {
  echo "\nStopping background nodes…"
  jobs -p | xargs -r kill || true
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$WS_DIR/src/lam_pick_place:$PYTHONPATH"
echo "PYTHONPATH=$PYTHONPATH"
echo "- Verifying Python package import…"
python3 - <<'PYTEST'
import sys
print('sys.path[0]=', sys.path[0])
try:
    import lam_pick_place
    import importlib
    m = importlib.import_module('lam_pick_place.trajectory_controller_node')
    print('OK: lam_pick_place found at:', lam_pick_place.__file__)
except Exception as e:
    print('ERROR: cannot import lam_pick_place:', e)
    raise
PYTEST
set -x
python3 -m lam_pick_place.trajectory_controller_node \
  --ros-args \
  -p publish_topic:="$PUBLISH_TOPIC" \
  -p input_trajectory_topic:="$INPUT_TRAJ_TOPIC" \
  -p rate_hz:="$RATE" &

python3 -m lam_pick_place.pick_place_manager_node \
  --ros-args \
  -p arm_side:=Left \
  -p publish_topic:="$PUBLISH_TOPIC" \
  -p input_trajectory_topic:="$INPUT_TRAJ_TOPIC" \
  -p rate_hz:="$RATE" \
  -p max_joint_step:=0.05 \
  -p max_iterations:=5000 \
  -p goal_threshold:=0.03 \
  -p log_dir:="$LOG_DIR" \
  -p num_runs:="$NUM_RUNS" \
  -p vary_targets:=true
set +x

# Wait for foreground manager to finish; controller exits via trap
wait
