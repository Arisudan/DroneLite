#!/usr/bin/env bash
# DroneForge indoor simulation: the AWS RoboMaker Small House in Gazebo Harmonic, the 3-inch drone (382.5 g, level centred O4 camera), the O4 feed with its latency,
# the Nimbus depth model (DA3) on that feed, RTAB-Map, SuperPoint+LightGlue visual odometry and the Kalman filters, and a camera-only explorer. SIMULATION ONLY.
#   ./run_house.sh --server http://127.0.0.1:8765 --link real --gui     explore the whole house (the depth server must be running)
#   --gui           Gazebo window (with the delayed O4 feed and its latency read-out)      --no-windows   no image windows (SSH without a screen)
#   --cam-latency MS / --cam-jitter MS   latency of the simulated O4 feed (default 150 / 25; set the value the GCS latency test measures)
#   --fw-defaults   keep the firmware core's stock take-off parameters (the simulation otherwise uses a slower climb / earlier hand-over)
#   --no-vo         switch the SuperPoint+LightGlue visual odometry off (default: on when exploring)
#   --hovertest [--control fc|pc] [--vo]    measure hop / shake / drift while hovering (writes worlds/last_hover_test.json)
#   --gcs           real-link mode: the GCS flies the sim over the simulated radio (controllers/sim_radio.py, CRSF on a pty linked at ~/.config/droneforge/sim_radio)
#                   and sees the camera as raw H.264 on udp port 3000 (vision/video_udp_out.py), like the O4. The explorer runs with --via-gcs: it steers only
#                   after TAKE OFF + EXPLORE in the GCS (sticks over udp 5766, nimbus_core/steering.py). Use ../sim.sh start.
#   --sysid         identify the tilt-to-motion plant (writes config/plant.json)         --map internal | --roam | --plan   older navigators
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; cd "$HERE"
source /opt/ros/jazzy/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/config/fastdds_shm.xml"      # whole camera frames through shared memory (30 fps without loss)
export GZ_SIM_RESOURCE_PATH="$HERE/models:$HERE/third_party/small_house_src/models:$GZ_SIM_RESOURCE_PATH"
GUI=0; SERVER=""; SHOW=1; MODE=explore; LINK=ideal; MAPMODE=rtabmap; VO=-1; HCTRL=fc; CAMLAT=150; CAMJIT=25; ARGS=()
# the feed latency measured on the real O4 (tools_scripts/o4_calibrate.py latency) becomes the default; --cam-latency / --cam-jitter override it
if [ -f config/o4_latency.json ]; then read -r CAMLAT CAMJIT < <(python3 -c "import json;d=json.load(open('config/o4_latency.json'));print(round(d['mean_ms']),round(d['std_ms']))" 2>/dev/null || echo "150 25"); fi
while [ $# -gt 0 ]; do case "$1" in
  --gui) GUI=1;; --no-windows) SHOW=0;; --plan) MODE=plan;; --roam) MODE=roam;; --sysid) MODE=sysid;; --hovertest) MODE=hovertest;; --gcs) MODE=gcs;; --vo) VO=1;; --no-vo) VO=0;; --fw-defaults) export DF_FW_DEFAULTS=1;;
  --server) case "$2" in http*) SERVER="$2"; shift;; *) echo "--server needs the address right after it, e.g. --server http://127.0.0.1:8765 --gui"; exit 1;; esac;;
  --link) LINK="$2"; shift;; --map) MAPMODE="$2"; shift;; --control) HCTRL="$2"; shift;; --cam-latency) CAMLAT="$2"; shift;; --cam-jitter) CAMJIT="$2"; shift;;
  *) ARGS+=("$1");; esac; shift; done
EXPLORE_LIKE=0; { [ "$MODE" = explore ] || [ "$MODE" = gcs ]; } && EXPLORE_LIKE=1      # gcs: the same perception, map and VO as exploring
GCS_MAP=0; [ "$MODE" = gcs ] && [ "${DF_MAP:-gcs}" = gcs ] && GCS_MAP=1      # 2026-11-23: the map + odometry come from the REAL pipeline fed by the GCS (../map.sh: O4 frames + FC telemetry), not from the simulator
[ $VO = -1 ] && { [ $EXPLORE_LIKE = 1 ] && VO=1 || VO=0; }
[ $GCS_MAP = 1 ] && VO=0
NEED_DEPTH=0; { [ $EXPLORE_LIKE = 1 ] || { [ "$MODE" = hovertest ] && [ $VO = 1 ]; }; } && [[ " ${ARGS[*]} " != *" --no-depth "* ]] && NEED_DEPTH=1
[ $NEED_DEPTH = 1 ] && [ -z "$SERVER" ] && [ ! -f vision/models/nimbus/da3_metric.weights.safetensors ] && { echo "needs the depth server: --server http://127.0.0.1:8765 (or the weights in vision/models/nimbus: python3 ../tools_scripts/get_models.py downloads them)"; exit 1; }
[ $VO = 1 ] && [ ! -f vision/models/superpoint_lightglue.onnx ] && { echo "visual odometry needs vision/models/superpoint.onnx and superpoint_lightglue.onnx (python3 ../tools_scripts/get_models.py downloads them; or use --no-vo to run without)"; exit 1; }
[ -f worlds/droneforge_house.sdf ] || python3 tools/make_house_world.py --camera 640x480@15
[ -f worlds/house_plan.npz ] || python3 tools/floorplan.py
python3 tools/make_gui_config.py >/dev/null 2>&1 || true
[ $GCS_MAP = 1 ] && NEED_DEPTH=0
USE_RTAB=0; [ $GCS_MAP = 0 ] && [ $EXPLORE_LIKE = 1 ] && [ "$MAPMODE" = rtabmap ] && USE_RTAB=1
KILL() { for n in sim_radio.py video_udp_out.py nimbus_perception.py map_stream_node.py visual_odom.py sim_tf_odom.py camera_feed_latency.py fc_v4_5_bridge.py house_explore.py house_nav.py house_nav_cam.py hover_test.py sysid_plant.py; do pkill -9 -f "controllers/$n|vision/$n|tools/$n" 2>/dev/null || true; done; pkill -9 parameter_bridg 2>/dev/null || true; pkill -9 -x image_bridge 2>/dev/null || true; pkill -9 -x rtabmap 2>/dev/null || true; pkill -9 -f "gz sim -s" 2>/dev/null || true; pkill -9 -f "gz sim -r" 2>/dev/null || true; pkill -9 -f "gz sim -g" 2>/dev/null || true; }
KILL; sleep 1                                                              # a leftover simulation publishes a second odometry
PIDS=(); cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; sleep 1; KILL; }
trap cleanup EXIT INT TERM
gz sim -s -r --headless-rendering worlds/droneforge_house.sdf >/dev/null 2>&1 &
PIDS+=($!); sleep 20
# --gui: a viewer window attached to the running server (physics stays headless and real time), with the delayed O4 feed panel
# QT_QPA_PLATFORM=xcb: the Gazebo GUI segfaults under Wayland (EGL "failed to create dri2 screen") on the 5090 laptop; X11 via XWayland works
if [ $GUI = 1 ]; then { [ -f config/gui_house.config ] && QT_QPA_PLATFORM=xcb gz sim -g --gui-config config/gui_house.config || QT_QPA_PLATFORM=xcb gz sim -g; } >/dev/null 2>&1 & PIDS+=($!); sleep 3; fi
ros2 run ros_gz_bridge parameter_bridge /droneforge/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators \
  /droneforge/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU /droneforge/sensors/rangefinder@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /droneforge/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /droneforge/camera/feed@sensor_msgs/msg/Image]gz.msgs.Image >/dev/null 2>&1 &
PIDS+=($!)
ros2 run ros_gz_image image_bridge /droneforge/camera/image_raw >/dev/null 2>&1 &      # the camera through the image bridge (the generic parameter_bridge dropped ~1/3 of the 30 fps frames)
PIDS+=($!); sleep 3
python3 controllers/fc_v4_5_bridge.py </dev/null >/dev/null 2>&1 & PIDS+=($!)
if [ "$MODE" != sysid ]; then
  # the O4 feed as the PC receives it: delayed, jittered; the annotated copy is what the Gazebo window shows
  python3 vision/camera_feed_latency.py --latency-ms "$CAMLAT" --jitter-ms "$CAMJIT" >/dev/null 2>&1 & PIDS+=($!)
  # the pose the way the real FC delivers it (sensors -> Kalman filters -> /odom + TF) and the camera mount transforms
  [ $GCS_MAP = 0 ] && python3 vision/sim_tf_odom.py --link "$LINK" $([ $VO = 1 ] && echo --vo) > /tmp/droneforge_tf.log 2>&1 & PIDS+=($!)
fi
if [ $NEED_DEPTH = 1 ]; then
  python3 vision/nimbus_perception.py $([ $SHOW = 1 ] && echo --show) ${SERVER:+--server "$SERVER"} --topic /droneforge/camera/image_delayed --feed-latency-ms "$CAMLAT" $([ $USE_RTAB = 1 ] || [ $VO = 1 ] && echo --ros-depth) 2>&1 | tee /tmp/droneforge_perception.log >/dev/null & PIDS+=($!)
  sleep $([ -n "$SERVER" ] && echo 5 || echo 20)
  [ $VO = 1 ] && { python3 vision/visual_odom.py > /tmp/droneforge_vo.log 2>&1 & PIDS+=($!); }
fi
if [ $USE_RTAB = 1 ]; then
  ros2 run rtabmap_slam rtabmap --delete_db_on_start --ros-args --params-file config/rtabmap_house.yaml -r rgb/image:=/droneforge/rgb/image -r depth/image:=/droneforge/depth/image -r rgb/camera_info:=/droneforge/rgb/camera_info > /tmp/droneforge_rtabmap.log 2>&1 &
  PIDS+=($!); sleep 3
  # 2D RTAB-Map map for the GCS: /map -> 1-pixel wall skeleton -> tcp port 5765 (the GCS Video page connects to the PC address by itself)
  python3 vision/map_stream_node.py > /tmp/droneforge_map_stream.log 2>&1 & PIDS+=($!)
  echo "map stream for the GCS on port 5765 (Video page: depth/map server = this PC)"
fi
if [ "$MODE" = gcs ]; then
  python3 controllers/sim_radio.py > /tmp/droneforge_sim_radio.log 2>&1 & PIDS+=($!)
  python3 vision/video_udp_out.py > /tmp/droneforge_video_out.log 2>&1 & PIDS+=($!)
  python3 controllers/house_explore.py --via-gcs --link "$LINK" --map "$MAPMODE" "${ARGS[@]}" > /tmp/droneforge_explore.log 2>&1 & PIDS+=($!)      # waits for GCS EXPLORE
  sleep 2; echo "SIM READY: GCS port ~/.config/droneforge/sim_radio, video udp://0.0.0.0:3000, map stream 127.0.0.1:5765"
fi
case $MODE in
  gcs) wait "${PIDS[0]}" 2>/dev/null || true; echo "the Gazebo server ended; stopping";;      # runs until ../sim.sh stop (closing the Gazebo window does not stop it)
  sysid) python3 tools/sysid_plant.py;;
  hovertest) python3 tools/hover_test.py --control "$HCTRL" "${ARGS[@]}";;
  plan) python3 controllers/house_nav.py "${ARGS[@]}";;
  roam) python3 controllers/house_nav_cam.py "${ARGS[@]}";;
  *) python3 controllers/house_explore.py $([ $SHOW = 1 ] && echo --show) --link "$LINK" --map "$MAPMODE" "${ARGS[@]}";;
esac
