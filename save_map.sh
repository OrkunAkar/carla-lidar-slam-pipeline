#!/bin/bash
# =============================================================================
#  save_map.sh — Trigger LIO-SAM map export while pipeline is running
#  Saves GlobalMap.pcd, trajectory.pcd, CornerMap.pcd, SurfMap.pcd
#  to ~/CARLA_PROJECT/maps/town10hd/
# =============================================================================

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash"
DEST="/CARLA_PROJECT/maps/town10hd"   # relative to HOME (LIO-SAM prepends $HOME)
RESOLUTION=0.1                         # voxel downsample resolution (m). 0.0 = no downsampling

CYAN='\033[0;36m'; GREEN='\033[0;32m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
log()  { echo -e "${CYAN}[MAP SAVE]${NC} $1"; }
ok()   { echo -e "${GREEN}[  OK  ]${NC} $1"; }
fail() { echo -e "${RED}[ FAIL ]${NC} $1"; }

source "$ROS_SETUP"
source "$WS_SETUP"

echo ""
echo -e "${BOLD}======================================================${NC}"
echo -e "${BOLD}   LIO-SAM Map Export                                ${NC}"
echo -e "${BOLD}======================================================${NC}"
echo ""
log "Destination : ${BOLD}$HOME$DEST${NC}"
log "Resolution  : ${BOLD}${RESOLUTION}m${NC} voxel downsample (0.0 = full density)"
echo ""

# Check that LIO-SAM is actually running
if ! ros2 service list 2>/dev/null | grep -q "lio_sam/save_map"; then
    fail "lio_sam/save_map service not found."
    fail "Make sure LIO-SAM is running: ros2 launch lio_sam run.launch.py ..."
    exit 1
fi

log "Calling save_map service..."
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
    "{resolution: ${RESOLUTION}, destination: '${DEST}'}"

echo ""
if [ $? -eq 0 ]; then
    ok "Map saved to: ${BOLD}$HOME$DEST${NC}"
    echo ""
    echo -e "  Files written:"
    for f in GlobalMap.pcd CornerMap.pcd SurfMap.pcd trajectory.pcd transformations.pcd; do
        FPATH="$HOME$DEST/$f"
        if [ -f "$FPATH" ]; then
            SIZE=$(du -h "$FPATH" | cut -f1)
            echo -e "    ${GREEN}✓${NC} $f  (${SIZE})"
        else
            echo -e "    ${RED}✗${NC} $f  (not found)"
        fi
    done
    echo ""
    log "Open in CloudCompare: ${BOLD}cloudcompare $HOME$DEST/GlobalMap.pcd${NC}"
    log "Or run:               ${BOLD}python3 ~/CARLA_PROJECT/view_map.py $HOME$DEST/GlobalMap.pcd${NC}"
else
    fail "Service call failed — check LIO-SAM terminal for errors."
fi
echo ""
