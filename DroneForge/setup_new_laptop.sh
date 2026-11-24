#!/usr/bin/env bash
# DroneForge setup check for a new laptop (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, RTX 5090).
#   ./setup_new_laptop.sh            checks everything, changes nothing, tells you what is missing
#   ./setup_new_laptop.sh --install  also installs what is missing (asks sudo for apt; pip goes to your user)
# Run it from the DroneForge folder, wherever you put it. It also re-points the old /home/htic/Music/DroneForge paths in the active scripts to this folder.
set -u
DF="$(cd "$(dirname "$0")" && pwd)"; INSTALL=0; [ "${1:-}" = "--install" ] && INSTALL=1
ok(){ printf "  [ok]      %s\n" "$1"; }; miss(){ printf "  [MISSING] %s\n" "$1"; MISSING=$((MISSING+1)); }; MISSING=0
echo "DroneForge folder: $DF"

echo "1. paths"
OLD=/home/htic/Music/DroneForge
if [ "$DF" != "$OLD" ]; then
  # match the old path only as a whole path (followed by / quote space or line end), so a new path that starts with it is never rewritten again
  FILES=$(grep -rIlE "$OLD([/\"' ]|\$)" "$DF/software" "$DF/tools_scripts" "$DF/gazebo" "$DF/betaflight_configurator" --include=*.py --include=*.sh --include=*.yaml --include=*.json 2>/dev/null)
  for f in $FILES; do sed -i -E "s#$OLD([/\"' ]|\$)#$DF\\1#g" "$f"; done; echo "  re-pointed $(echo "$FILES" | grep -c .) files from $OLD to $DF"
else ok "same path as before, nothing to change"; fi

echo "2. system (ROS 2 Jazzy, Gazebo Harmonic, RTAB-Map)"
[ -f /opt/ros/jazzy/setup.bash ] && ok "ROS 2 Jazzy" || miss "ROS 2 Jazzy  (sudo apt install ros-jazzy-desktop)"
command -v gz >/dev/null && gz sim --version 2>/dev/null | head -1 | grep -q "version 8" && ok "Gazebo Harmonic (gz sim 8)" || miss "Gazebo Harmonic  (sudo apt install ros-jazzy-ros-gz)"
for p in ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-sync ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-cv-bridge ros-jazzy-tf2-ros; do
  dpkg -s $p >/dev/null 2>&1 && ok "$p" || miss "$p  (sudo apt install $p)"; done
command -v ffmpeg >/dev/null && ok "ffmpeg" || miss "ffmpeg  (sudo apt install ffmpeg)"

echo "3. python packages (system Python 3.12)"
for m in PyQt6 cv2 numpy scipy serial yaml websockets pytest onnxruntime; do
  python3 -c "import $m" 2>/dev/null && ok "$m" || miss "python module $m"; done

echo "4. GPU (the depth model, Depth Anything 3)"
if command -v nvidia-smi >/dev/null; then ok "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)"; else miss "NVIDIA driver (nvidia-smi not found)"; fi
PROV=$(python3 -c "import onnxruntime as o;print(','.join(o.get_available_providers()))" 2>/dev/null)
echo "$PROV" | grep -q CUDAExecutionProvider && ok "onnxruntime has CUDA ($PROV)" || miss "onnxruntime-gpu (providers now: ${PROV:-none}); the depth model would run on CPU, ~3 s per frame"

echo "5. models"
for m in gazebo/vision/models/nimbus/da3_metric.onnx gazebo/vision/models/nimbus/da3_metric.weights.safetensors gazebo/vision/models/superpoint.onnx gazebo/vision/models/superpoint_lightglue.onnx; do
  [ -s "$DF/$m" ] && ok "$m" || miss "$m  (unpack droneforge_models.tar into the DroneForge folder)"; done

if [ $INSTALL = 1 ] && [ $MISSING -gt 0 ]; then
  echo "installing what is missing..."
  sudo apt update && sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-rtabmap-ros ros-jazzy-cv-bridge ros-jazzy-tf2-ros ffmpeg python3-pip
  pip install --user --break-system-packages PyQt6 opencv-python numpy scipy pyserial PyYAML websockets pytest
  if command -v nvidia-smi >/dev/null; then
    pip uninstall -y --break-system-packages onnxruntime 2>/dev/null
    pip install --user --break-system-packages "onnxruntime-gpu[cuda,cudnn]"     # ORT >= 1.32 is built for CUDA 13; the extras pull the matching nvidia-* wheels
  else pip install --user --break-system-packages onnxruntime; fi
  echo "done; run this script again (without --install) to confirm."
elif [ $MISSING -gt 0 ]; then echo; echo "$MISSING item(s) missing. Run  ./setup_new_laptop.sh --install  or install them yourself."
else echo; echo "everything is in place."; fi
echo "Then: GCS = cd $DF/software/pc && python3 -m nimbus_gcs      depth model check = python3 $DF/gazebo/vision/depth_server.py (it prints 'providers: [CUDAExecutionProvider ...]')"
