# Setting up DroneForge from GitHub (https://github.com/Arisudan/DroneLite)

The repository holds the **code**. The model weights are not in it (they are downloaded, see step 3), nor are flight recordings / FC logs (`docs/`), `archive/`, `reference/`,
the third-party Gazebo house (`gazebo/third_party/small_house_src`), FC config dumps (they contain the receiver UID) and everything ELRS-binding related
(`README.md` shows `<BINDING_PHRASE>` / `<BINDING_UID>` placeholders; your real values stay local).

## New machine (Ubuntu 24.04), four commands
    git clone https://github.com/Arisudan/DroneLite ~/DroneLite && cd ~/DroneLite/DroneForge
    ./setup_new_laptop.sh --install        # 1) apt: ROS 2 Jazzy, Gazebo, RTAB-Map, ffmpeg   2) pip: requirements.txt (or requirements-gpu.txt when an NVIDIA GPU is found)   3) models
    ./setup_new_laptop.sh                  # run again: it should report only what you accept (e.g. no GPU)
    cd software/pc && python3 -m nimbus_gcs

Doing it by hand:

| What | Command |
|---|---|
| Python packages, no NVIDIA GPU | `pip install --user --break-system-packages -r requirements.txt` |
| Python packages, NVIDIA GPU (CUDA onnxruntime) | `pip uninstall -y --break-system-packages onnxruntime; pip install --user --break-system-packages -r requirements-gpu.txt` |
| Tests / doc tools | `pip install --user --break-system-packages -r requirements-dev.txt` |
| Models, required (~1.4 GB: depth + visual odometry) | `python3 tools_scripts/get_models.py` |
| Check the models only (size + sha256) | `python3 tools_scripts/get_models.py --check` |
| Face recognition model (optional, ~290 MB) | `python3 tools_scripts/get_models.py --optional` |
| No internet on that machine: models from a USB stick / another laptop | `python3 tools_scripts/get_models.py --from /media/usb/models` (the folder holds `nimbus/da3_metric.*`, `superpoint.onnx`, `superpoint_lightglue.onnx`) |
| What the models are and where they come from | `python3 tools_scripts/get_models.py --list` |

`get_models.py` resumes an interrupted download, verifies every file by size and sha256 and never replaces a good file. Where the files come from: Depth Anything 3 (the files the Nimbus app downloads
from its public bucket) and SuperPoint / LightGlue (LightGlue-ONNX v1.0.0 releases on GitHub). `detect.onnx` (optional YOLO for the GCS DETECT button) is yours to supply.
On an NVIDIA machine `python3 gazebo/vision/depth_server.py` must print `CUDAExecutionProvider`; otherwise the depth model runs on the CPU (~3 s per frame).

## Updating a machine that already has a full DroneForge folder (keeps your data, deletes nothing)
    cd ~/DroneLite && git pull
    rsync -a --exclude='/README.md' --exclude='.git*' ~/DroneLite/DroneForge/ ~/Music/DroneForge/
`--exclude='/README.md'` keeps your local README.md (with the real binding phrase). Run `./setup_new_laptop.sh` once if absolute paths changed.
