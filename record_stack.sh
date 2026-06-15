#!/bin/bash
# =============================================================================
#  CARLA Self-Driving Car Stack — Bag Recorder
#  Records LiDAR + IMU + TF + Odometry into a timestamped bag
# =============================================================================

ROS_SETUP="/opt/ros/humble/setup.bash"
BAG_DIR="$HOME/CARLA_PROJECT/bags"
CARLA_PORT=2000

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[RECORDER]${NC} $1"; }
ok()   { echo -e "${GREEN}[  OK  ]${NC} $1"; }
warn() { echo -e "${YELLOW}[ WARN ]${NC} $1"; }
fail() { echo -e "${RED}[ FAIL ]${NC} $1"; exit 1; }

# ── Checks ─────────────────────────────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || fail "ROS2 Humble not found at $ROS_SETUP"

# Check CARLA is running
if ! nc -z localhost $CARLA_PORT 2>/dev/null; then
    fail "CARLA is not running (port $CARLA_PORT unreachable). Start the stack first with ./launch_stack.sh"
fi

# ── Topics to record ───────────────────────────────────────────────────────────
TOPICS=(
    /carla/hero/lidar           # 32ch raw LiDAR point cloud
    /carla/hero/semantic_lidar  # Semantic LiDAR (GT labels: vehicle=10, ped=4, road=7...)
    /carla/hero/imu             # Accelerometer + Gyroscope
    /carla/hero/odometry        # Ground truth odometry
    /tf                         # Transform tree (required for RViz2 playback)
    /tf_static                  # Static transforms (sensor positions on car)
)

# ── Output folder ──────────────────────────────────────────────────────────────
mkdir -p "$BAG_DIR"
TIMESTAMP=$(date +"%Y_%m_%d-%H_%M_%S")
BAG_NAME="carla_recording_$TIMESTAMP"
BAG_PATH="$BAG_DIR/$BAG_NAME"

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}======================================================${NC}"
echo -e "${BOLD}   CARLA Stack — Bag Recorder                         ${NC}"
echo -e "${BOLD}======================================================${NC}"
echo ""
log "Recording to: ${BOLD}$BAG_PATH${NC}"
echo ""
echo -e "  Topics being recorded:"
for t in "${TOPICS[@]}"; do
    echo -e "    ${GREEN}+${NC} $t"
done
echo ""
warn "Make sure the car is driving before recording!"
warn "Press Ctrl+C to stop recording."
echo ""

# ── Record ─────────────────────────────────────────────────────────────────────
source "$ROS_SETUP"

ros2 bag record \
    -o "$BAG_PATH" \
    "${TOPICS[@]}"

# ── Done ───────────────────────────────────────────────────────────────────────
echo ""
ok "Recording stopped."
echo ""
log "Bag saved to: ${BOLD}$BAG_PATH${NC}"
echo ""
log "To inspect:  ros2 bag info $BAG_PATH"
log "To play:     ros2 bag play $BAG_PATH --loop"
log "To play 10x: ros2 bag play $BAG_PATH --loop --rate 10.0"
echo ""
