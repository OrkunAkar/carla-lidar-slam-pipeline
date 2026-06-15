#!/bin/bash
# =============================================================================
#  CARLA Self-Driving Car Stack — Stop Script
# =============================================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'

log()  { echo -e "${CYAN}[LAUNCHER]${NC} $1"; }
ok()   { echo -e "${GREEN}[  OK  ]${NC} $1"; }

echo ""
echo -e "Stopping CARLA Self-Driving Car Stack ..."
echo ""

log "Killing ROS2 nodes ..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "ros2 run"    2>/dev/null
pkill -f "local_planner" 2>/dev/null
pkill -f "navigation_hmi" 2>/dev/null
pkill -f "carla_waypoint_publisher" 2>/dev/null
pkill -f "carla_ros_bridge" 2>/dev/null
ok "ROS2 nodes stopped."

log "Killing RViz2 ..."
pkill -f "rviz2" 2>/dev/null
ok "RViz2 stopped."

log "Killing CARLA server ..."
pkill -f "CarlaUE4" 2>/dev/null
pkill -f "CarlaUE4-Linux" 2>/dev/null
ok "CARLA stopped."

echo ""
ok "All processes stopped."
echo ""
