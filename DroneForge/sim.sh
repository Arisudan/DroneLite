#!/usr/bin/env bash
# DroneForge simulation, real-link mode: the GCS flies the Gazebo drone through the same paths the real drone uses (SIMULATION ONLY, no hardware is touched).
#   ./sim.sh start [--no-gui] [--no-gcs]   Gazebo house + sim FC + simulated ELRS radio (CRSF on a pty) + O4-style video (raw H.264, udp 3000)
#                                          + depth model, visual odometry, RTAB-Map, 2D map stream; then opens the GCS connected to it
#   ./sim.sh stop                          closes everything the simulation started (Gazebo, ROS nodes, RTAB-Map, depth server, video encoder, the GCS it opened)
#   ./sim.sh status                        what is running
# Logs: ~/.local/state/droneforge/sim/ (sim.log, gcs.log) and /tmp/droneforge_*.log (one per node).
set -u
DF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ST="$HOME/.local/state/droneforge/sim"; mkdir -p "$ST"
RADIO="$HOME/.config/droneforge/sim_radio"
VIDEO_URL="udp://0.0.0.0:3000"
LINK_MODE="${DF_LINK:-real}"     # real = the pose the drone will have (flow + SuperPoint/LightGlue key-frame fixes in the Kalman filters, sim_tf_odom.py --vo); ideal = the simulator's exact pose
CAM_MS="${DF_CAM_MS:-30}"        # O4 feed delay seen by the autonomy (owner, 2026-11-21: <= 30 ms, 30 fps fixed); DF_CAM_MS=172 for the RTSP value measured 2026-11-20
# every program the simulation can start (matched on the command line; this file never contains a pattern on its own command line)
PATTERNS=('^(/usr/bin/)?bash \S*run_house\.sh' '^gz sim -[sg]' '^\S*(python3 \S*ros2 run ros_gz_(bridge|image)|ros_gz_bridge/parameter_bridge|ros_gz_image/image_bridge)' '^\S*(python3 \S*ros2 run rtabmap_slam|rtabmap_slam/rtabmap)'
          '^\S*python3 \S*(controllers|vision)/(fc_v4_5_bridge|sim_radio|house_explore|camera_feed_latency|sim_tf_odom|nimbus_perception|visual_odom|map_stream_node|video_udp_out|depth_server)\.py'
          '^\S*ffmpeg .*udp://[0-9.]*:3000' '^\S*python3 -m nimbus_gcs --port \S*sim_radio')     # anchored at the program name: a shell whose command line merely mentions them is never matched

alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }
readpid() { cat "$ST/$1" 2>/dev/null; }
ancestors() { local p=$$; while [ "$p" -gt 1 ] 2>/dev/null; do echo "$p"; p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null) || break; done; }
leftovers() { local skip; skip=$(ancestors | paste -sd'|'); for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done 2>/dev/null | sort -u | grep -Evx "${skip:-0}" || true; }

status() {
  local s g; s=$(readpid sim.pgid); g=$(readpid gcs.pid)
  alive "$s" && echo "simulation: running (process group $s)" || echo "simulation: stopped"
  alive "$g" && echo "GCS:        running (pid $g)" || echo "GCS:        not running (from sim.sh)"
  [ -L "$RADIO" ] && echo "radio:      $RADIO -> $(readlink "$RADIO")"
  local n; n=$(leftovers | wc -l); echo "processes:  $n simulation process(es) found"
}

stop() {
  "$DF/map.sh" stop > /dev/null 2>&1
  local g s; g=$(readpid gcs.pid); s=$(readpid sim.pgid)
  if alive "$g"; then kill -TERM "$g" 2>/dev/null; fi
  if alive "$s"; then kill -TERM -- "-$s" 2>/dev/null; fi
  for _ in 1 2 3 4 5 6; do [ -z "$(leftovers)" ] && ! alive "$g" && break; sleep 0.5; done
  local left; left=$(leftovers)
  [ -n "$left" ] && kill -TERM $left 2>/dev/null && sleep 1
  left=$(leftovers); [ -n "$left" ] && kill -KILL $left 2>/dev/null
  alive "$g" && kill -KILL "$g" 2>/dev/null
  rm -f "$ST/sim.pgid" "$ST/gcs.pid"; [ -L "$RADIO" ] && rm -f "$RADIO"
  sleep 0.5; left=$(leftovers)
  if [ -z "$left" ]; then echo "stopped: nothing of the simulation is running"; else echo "still running (pids): $left"; return 1; fi
}

start() {
  local GUI=--gui GCS=1
  for a in "$@"; do case "$a" in --no-gui) GUI="";; --no-gcs) GCS=0;; *) echo "unknown option $a"; exit 2;; esac; done
  if alive "$(readpid sim.pgid)"; then echo "the simulation is already running (./sim.sh stop first)"; exit 1; fi
  [ -n "$(leftovers)" ] && { echo "old simulation processes found: cleaning up first"; stop >/dev/null; }
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  : > "$ST/sim.log"
  rm -f "$ST/sim.pgid"
  setsid bash -c "echo \$\$ > '$ST/sim.pgid'; cd '$DF/gazebo' && exec ./run_house.sh --gcs --link $LINK_MODE --cam-latency $CAM_MS --cam-jitter 3 $GUI" > "$ST/sim.log" 2>&1 < /dev/null &      # own session: its pid = the process group stop kills
  for _ in 1 2 3 4 5 6 7 8 9 10; do [ -s "$ST/sim.pgid" ] && break; sleep 0.2; done
  local pid; pid=$(readpid sim.pgid)
  echo "starting the simulation (log $ST/sim.log); this takes about 60 s ..."
  local t=0
  until grep -q "SIM READY" "$ST/sim.log" 2>/dev/null; do
    alive "$pid" || { echo "the simulation stopped while starting; last lines:"; tail -5 "$ST/sim.log"; exit 1; }
    [ $t -ge 180 ] && { echo "not ready after 180 s; see $ST/sim.log"; exit 1; }
    sleep 1; t=$((t + 1))
  done
  echo "simulation ready after ${t} s: radio $RADIO, video $VIDEO_URL, 2D map 127.0.0.1:5765"
  if [ $GCS = 1 ]; then
    printf '{"kind": "network", "target": "%s", "analog": false}\n' "$VIDEO_URL" > "$ST/video.json"
    rm -f "$ST/gcs.pid"
    DRONEFORGE_VIDEO_STATE="$ST/video.json" setsid bash -c "echo \$\$ > '$ST/gcs.pid'; cd '$DF/software/pc' && exec python3 -m nimbus_gcs --port '$RADIO' --connect --sim --video '$VIDEO_URL'" > "$ST/gcs.log" 2>&1 < /dev/null &
    for _ in 1 2 3 4 5 6 7 8 9 10; do [ -s "$ST/gcs.pid" ] && break; sleep 0.2; done
    sleep 4
    if alive "$(readpid gcs.pid)"; then echo "GCS opened (pid $(readpid gcs.pid)), connected to the simulated radio. Stop everything with: ./sim.sh stop"
      [ "${DF_MAP:-gcs}" = gcs ] && "$DF/map.sh" start      # the map: the real pipeline (O4 frames + FC telemetry from the GCS) -> RTAB-Map -> the GCS RViz pane
    else echo "the GCS did not start; its last lines ($ST/gcs.log):"; grep -v "^QFont\|propagateSize" "$ST/gcs.log" | tail -5; fi
  else
    echo "GCS:  cd $DF/software/pc && python3 -m nimbus_gcs --port $RADIO --connect --sim --video $VIDEO_URL"
  fi
}

case "${1:-}" in
  start) shift; start "$@";;
  stop) stop;;
  status) status;;
  *) echo "usage: ./sim.sh start [--no-gui] [--no-gcs] | stop | status"; exit 2;;
esac
