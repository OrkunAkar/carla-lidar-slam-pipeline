#!/bin/bash
# =============================================================================
#  CARLA Self-Driving Car Stack — Bag Player
#  Plays a recorded bag + opens RViz2 automatically
# =============================================================================

ROS_SETUP="/opt/ros/humble/setup.bash"
BAG_DIR="$HOME/CARLA_PROJECT/bags"

# ── Playback speed ─────────────────────────────────────────────────────────────
# 1.0 = real-time | 5.0 = 5x | 10.0 = 10x (recommended for low-FPS recordings)
RATE=1.0

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[PLAYER]${NC} $1"; }
ok()   { echo -e "${GREEN}[  OK  ]${NC} $1"; }
warn() { echo -e "${YELLOW}[ WARN ]${NC} $1"; }
fail() { echo -e "${RED}[ FAIL ]${NC} $1"; exit 1; }

# ── Checks ─────────────────────────────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || fail "ROS2 Humble not found at $ROS_SETUP"
[ -d "$BAG_DIR" ]   || fail "No bags folder found at $BAG_DIR. Record something first with ./record_stack.sh"

# ── Pick bag ───────────────────────────────────────────────────────────────────
# If a bag path is given as argument, use it. Otherwise use the latest recording.
if [ -n "$1" ]; then
    BAG_PATH="$1"
else
    BAG_PATH=$(ls -dt "$BAG_DIR"/carla_recording_* 2>/dev/null | head -1)
fi

[ -d "$BAG_PATH" ] || fail "Bag not found: $BAG_PATH"

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}======================================================${NC}"
echo -e "${BOLD}   CARLA Stack — Bag Player                           ${NC}"
echo -e "${BOLD}======================================================${NC}"
echo ""
log "Playing bag: ${BOLD}$BAG_PATH${NC}"
log "Speed: ${BOLD}${RATE}x${NC}"
echo ""

# Show bag info
source "$ROS_SETUP"
ros2 bag info "$BAG_PATH"
echo ""

warn "RViz2 will open automatically."
warn "Set Fixed Frame to 'map' and add your topics."
warn "Press Ctrl+C in this terminal to stop playback."
echo ""

# ── Open RViz2 in background ───────────────────────────────────────────────────
terminator -T "[PLAYER] RViz2" -e \
    "bash -c 'source $ROS_SETUP; echo \">>> RViz2 for bag playback\"; echo; echo \"  Fixed Frame : map\"; echo \"  Add > PointCloud2 > /carla/hero/lidar\"; echo \"  Add > Odometry   > /carla/hero/odometry\"; echo; rviz2; read'" &

sleep 3  # give RViz2 time to open

# ── Play bag ───────────────────────────────────────────────────────────────────
log "Starting playback at ${RATE}x ..."
ros2 bag play "$BAG_PATH"  --rate "$RATE"

echo ""
ok "Playback finished."
echo ""
