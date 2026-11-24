#!/usr/bin/env bash
# DroneForge mapping from the GCS's O4 video + FC telemetry (the real drone, or the simulator through the same GCS paths): RTAB-Map's 2D map, shown in the GCS RViz pane.
#   ./map.sh start [--rectify]   o4_map_node (DA3 depth, FC odometry) + SuperPoint/LightGlue VO + RTAB-Map      --rectify: the real O4 lens (config/o4_camera.yaml)
#   ./map.sh stop | status | reset (clears the RTAB-Map map)
# Needs the GCS running (it shares the frames and the telemetry). Logs: ~/.local/state/droneforge/map/
set -u
DF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; ST="$HOME/.local/state/droneforge/map"; mkdir -p "$ST"
PATTERNS=('^\S*python3 \S*realmap/o4_map_node\.py' '^\S*python3 \S*vision/map_stream_node\.py --map' '^\S*python3 \S*vision/visual_odom\.py --map' '^\S*(python3 \S*ros2 run rtabmap_slam rtabmap --ros-args --params-file \S*rtabmap_o4|rtabmap_slam/rtabmap --ros-args --params-file \S*rtabmap_o4)')
running() { for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done 2>/dev/null | sort -u; }
src() { set +u; source /opt/ros/jazzy/setup.bash; set -u; export FASTRTPS_DEFAULT_PROFILES_FILE="$DF/gazebo/config/fastdds_shm.xml"; }
case "${1:-}" in
  start)
    [ -n "$(running)" ] && { echo "mapping already running"; exit 0; }
    src; RECT=""; [ "${2:-}" = "--rectify" ] && RECT="--rectify"
    setsid python3 "$DF/software/pc/realmap/o4_map_node.py" $RECT > "$ST/o4_map_node.log" 2>&1 < /dev/null &
    setsid python3 "$DF/gazebo/vision/visual_odom.py" --map > "$ST/visual_odom.log" 2>&1 < /dev/null &
    setsid ros2 run rtabmap_slam rtabmap --ros-args --params-file "$DF/gazebo/config/rtabmap_o4.yaml" -p delete_db_on_start:=true \
      -r rgb/image:=/droneforge/rgb/image -r depth/image:=/droneforge/depth/image -r rgb/camera_info:=/droneforge/rgb/camera_info > "$ST/rtabmap.log" 2>&1 < /dev/null &
    setsid python3 "$DF/gazebo/vision/map_stream_node.py" --map > "$ST/map_stream.log" 2>&1 < /dev/null &      # /map -> /map_thin + pose -> tcp 5765 -> the GCS map views
    sleep 3; echo "mapping started (logs $ST): $(running | wc -l) processes";;
  stop)
    P=$(running); [ -n "$P" ] && kill $P 2>/dev/null; sleep 1; P=$(running); [ -n "$P" ] && kill -9 $P 2>/dev/null; echo "mapping stopped";;
  reset) src; timeout 10 ros2 service call /rtabmap/reset std_srvs/srv/Empty > /dev/null 2>&1 && echo "map cleared" || echo "map reset failed (is the mapping running?)";;
  status) echo "mapping processes: $(running | wc -l)"; tail -1 "$ST/o4_map_node.log" 2>/dev/null;;
  *) echo "usage: ./map.sh start [--rectify] | stop | status | reset"; exit 2;;
esac
