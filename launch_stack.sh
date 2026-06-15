#!/bin/bash
# =============================================================================
#  CARLA Self-Driving Car Stack Launcher
#  Automates all 6 terminals described in proc1.md
# =============================================================================

# ── Paths ─────────────────────────────────────────────────────────────────────
CARLA_ROOT="$HOME/CARLA_PROJECT/CARLA_0.9.15"
BRIDGE_INSTALL="$HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash"
APP_INSTALL="$HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"
CARLA_PORT=2000

# ── MODE ──────────────────────────────────────────────────────────────────────
# Set MODE=full    → original setup  (all sensors, CARLA window visible)
# Set MODE=lidar   → LiDAR only mode (no cameras/IMU/radar, CARLA runs headless)
MODE="lidar"

FULL_JSON="$HOME/CARLA_PROJECT/carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2/car_definition_file.json"
LIDAR_JSON="$HOME/CARLA_PROJECT/carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2/car_definition_lidar_only.json"

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

# ── Helpers ────────────────────────────────────────────────────────────────────
log()  { echo -e "${CYAN}[LAUNCHER]${NC} $1"; }
ok()   { echo -e "${GREEN}[  OK  ]${NC} $1"; }
warn() { echo -e "${YELLOW}[ WARN ]${NC} $1"; }
fail() { echo -e "${RED}[ FAIL ]${NC} $1"; exit 1; }

# ── Sanity checks ──────────────────────────────────────────────────────────────
[ -f "$CARLA_ROOT/CarlaUE4.sh" ] || fail "CARLA not found at $CARLA_ROOT"
[ -f "$BRIDGE_INSTALL" ]         || fail "Bridge not built. Run colcon build in ros2_bridge_ws/ros-bridge first."
[ -f "$APP_INSTALL" ]            || fail "App workspace not built. Run colcon build in carla_ros2_ws first."
command -v terminator &>/dev/null || fail "terminator not found. Install it with: sudo apt install terminator"

# ── Wait for CARLA port ────────────────────────────────────────────────────────
wait_for_carla() {
    log "Waiting for CARLA to be ready on port $CARLA_PORT ..."
    local attempts=0
    while ! nc -z localhost $CARLA_PORT 2>/dev/null; do
        sleep 2
        attempts=$((attempts + 1))
        echo -ne "  Attempt $attempts / 40 ...\r"
        if [ $attempts -ge 40 ]; then
            fail "CARLA did not start in time (80s). Check terminal 1."
        fi
    done
    ok "CARLA is ready!"
}

# ── Common ROS env block (used in every ROS terminal) ─────────────────────────
ros_env() {
    echo "source $ROS_SETUP && \
          source $BRIDGE_INSTALL && \
          export CARLA_ROOT=$CARLA_ROOT && \
          export PYTHONPATH=\$PYTHONPATH:\$CARLA_ROOT/PythonAPI/carla"
}

# =============================================================================
#  LAUNCH SEQUENCE
# =============================================================================

echo ""
echo -e "${BOLD}======================================================${NC}"
echo -e "${BOLD}   CARLA Self-Driving Car Stack — Launcher            ${NC}"
echo -e "${BOLD}======================================================${NC}"
echo ""

# ── TERMINAL 1 — CARLA Server ─────────────────────────────────────────────────
log "Starting CARLA server in MODE=$MODE (terminal 1) ..."
if [ "$MODE" = "lidar" ]; then
    # Headless: no render window, frees GPU memory and CPU rendering overhead
    # Cameras won't work in this mode but LiDAR runs fine
    terminator -T "[T1] CARLA Server (Headless)" -e \
        "bash -c 'echo \">>> Starting CARLA server HEADLESS ...\"; bash $CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen; echo \">>> CARLA exited. Press Enter to close.\"; read'" &
else
    terminator -T "[T1] CARLA Server" -e \
        "bash -c 'echo \">>> Starting CARLA server ...\"; bash $CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low; echo \">>> CARLA exited. Press Enter to close.\"; read'" &
fi

wait_for_carla

# ── TERMINAL 2 — CARLA ROS2 Bridge + Ego Vehicle ──────────────────────────────
log "Launching CARLA ROS2 bridge + ego vehicle (terminal 2) ..."
if [ "$MODE" = "lidar" ]; then
    VEHICLE_JSON="$LIDAR_JSON"
else
    VEHICLE_JSON="$FULL_JSON"
fi
terminator -T "[T2] ROS Bridge" -e \
    "bash -c '$(ros_env); echo \">>> Launching ROS2 CARLA bridge (json: $VEHICLE_JSON) ...\"; ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py objects_definition_file:=$VEHICLE_JSON; echo \">>> Bridge exited. Press Enter to close.\"; read'" &
sleep 8   # give bridge time to fully connect

# ── TERMINAL 3 — Waypoint Publisher ───────────────────────────────────────────
log "Launching waypoint publisher (terminal 3) ..."
terminator -T "[T3] Waypoint Publisher" -e \
    "bash -c '$(ros_env); echo \">>> Launching waypoint publisher ...\"; ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py; echo \">>> Waypoint publisher exited. Press Enter to close.\"; read'" &
sleep 4

# ── TERMINAL 4 — Local Planner (AD Agent) ─────────────────────────────────────
log "Launching local planner / AD agent (terminal 4) ..."
terminator -T "[T4] Local Planner" -e \
    "bash -c '$(ros_env); echo \">>> Launching local planner ...\"; ros2 run carla_ad_agent local_planner; echo \">>> Local planner exited. Press Enter to close.\"; read'" &
sleep 3

# ── TERMINAL 5 — Navigation HMI ───────────────────────────────────────────────
log "Launching Navigation HMI (terminal 5) ..."
terminator -T "[T5] Navigation HMI" -e \
    "bash -c '$(ros_env); source $APP_INSTALL; echo \">>> Launching Navigation HMI ...\"; ros2 run navigation navigation_hmi; echo \">>> Navigation HMI exited. Press Enter to close.\"; read'" &
sleep 2

# ── TERMINAL 6 — RViz2 ────────────────────────────────────────────────────────
log "Launching RViz2 (terminal 6) ..."
terminator -T "[T6] RViz2" -e \
    "bash -c 'source $ROS_SETUP; echo \">>> Launching RViz2 ...\"; echo; echo \"  RViz2 SETUP REMINDER:\"; echo \"  Add > MarkerArray  > /carla_road_network\"; echo \"  Add > MarkerArray  > /carla/markers\"; echo \"  Add > Path         > /carla/waypoints\"; echo \"  Add > PointCloud2  > /carla/hero/lidar\"; echo \"  Use 2D Goal Pose to set destination\"; echo; rviz2; echo \">>> RViz2 exited. Press Enter to close.\"; read'" &

echo ""
ok "All 6 terminals launched!"
echo ""
echo -e "${YELLOW}  To stop everything, close all terminal windows"
echo -e "  or run: ./stop_stack.sh${NC}"
echo ""
