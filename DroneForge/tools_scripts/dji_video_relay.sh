#!/bin/bash
# DJI O4 / CosmoStreamer raw H.264 or H.265 over UDP  ->  MPEG-TS on localhost, so the GCS (OpenCV/FFmpeg) can open it with the lowest latency.
# usage: tools_scripts/dji_video_relay.sh [h264|hevc] [in_port=3000] [out_port=5600]
# In the GCS VIDEO tab choose "DJI via CosmoStreamer (network URL)" and paste:  udp://127.0.0.1:5600   (default out_port)
# Raw streams have no timing: without -use_wallclock_as_timestamps ffplay assumes ~25 fps and plays a 50-60 fps stream at half speed (looks like lag).
# Check first with the picture only:  ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 -use_wallclock_as_timestamps 1 -f h264 -i "udp://0.0.0.0:3000"
CODEC=${1:-h264}; IN=${2:-3000}; OUT=${3:-5600}
exec ffmpeg -hide_banner -loglevel warning -fflags nobuffer -flags low_delay -probesize 32768 -analyzeduration 0 \
  -use_wallclock_as_timestamps 1 -f "$CODEC" -i "udp://0.0.0.0:${IN}?fifo_size=1000000&overrun_nonfatal=1" -c copy -f mpegts "udp://127.0.0.1:${OUT}?pkt_size=1316"
