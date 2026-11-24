# DroneForge Gazebo Simulation (v4.5 Firmware Core)

This folder contains the complete Gazebo Harmonic simulation for DroneForge matching the exact physical airframe and sensors of the real drone.

## Specifications Modeled
- **All-Up-Weight (AUW)**: $0.3825\text{ kg}$ (~$382.5\text{ g}$)
- **Propulsion**: 3-inch 3-blade props, Quad-X configuration, $140\text{ mm}$ wheelbase.
- **Thrust-to-Weight**: $\sim 2.6\times$ (nominal $4\text{S}$ LiPo battery curve). Hover throttle at $\sim 62\%$.
- **Camera Sensor**: **DJI O4 Camera & Air Unit** ($1280 \times 720$ @ $30\text{ fps}$, $120^\circ$ H-FOV, $15^\circ$ forward uptilt).
- **Downward Sensor**: **MTF-02** downward TOF rangefinder ($0.02\text{ m} - 3.0\text{ m}$) & optical flow sensor.
- **Flight Controller**: **Betaflight v4.5** height hold and position hold control laws executing via `libhh.so`.

---

## Directory Structure
- `models/droneforge_3in/`: Detailed SDF model with inertia matrix, motor dynamics, DJI O4 camera, MTF-02, and IMU.
- `worlds/droneforge_testbench.sdf`: Indoor flight test arena with textured features and target landing pad.
- `controllers/fc_v4_5_bridge.py`: Firmware controller bridge interfacing with `libhh.so`.
- `viewers/camera_viewer.py`: Real-time OpenCV FPV HUD viewer rendering live DJI O4 video and flight telemetry.
- `run_sim.sh`: Single master launch script.

---

## Quick Start

To launch the full simulation with Gazebo 3D view and DJI O4 camera feed:

```bash
cd gazebo
./run_sim.sh
```

### Keyboard Flight Controls (Active in Camera Window)
- **`A`**: **Arm** (spins motors at idle)
- **`T`**: **Takeoff** (climbs with v4.5 creep to $0.50\text{ m}$)
- **`H`**: **Toggle Alt Hold / Pos Hold** (switches between altitude hold only and horizontal station-keeping)
- **`L`**: **Land** (cushioned touchdown and auto-disarm)
- **`D`**: **Disarm**
- **`Q`** or **`Esc`**: **Quit**

## Indoor house test (2026-11-19)
`./run_house.sh [--gui] [--no-depth]`: AWS RoboMaker Small House (MIT-0, `third_party/small_house_src`) + the 3-inch drone + Nimbus perception + room-to-room navigator.
* World: `tools/make_house_world.py --camera 640x360@15` (4 ms physics step = real time on this laptop; camera level, as Nimbus mounts it). Floor plan for routing: `tools/floorplan.py` -> `worlds/house_plan.png`.
* Models (the same files Nimbus ships): `vision/models/nimbus/da3_metric.onnx` + `.weights.safetensors` (Depth Anything 3 metric, 1.3 GB, from the public bucket `droneforge-assets-public`) and `yolov12n-face.onnx`. Node: `vision/nimbus_perception.py` (6x8 grid, 0.2-1.0 m obstacle test, as in the Nimbus analysis doc 06).
* Navigator: `controllers/house_nav.py` (A* on the floor plan, odometry for position, stick commands on `/droneforge/fc/command`).
* **Status:** `--no-depth` (floor plan + odometry, no camera): 5/5 legs (bedroom x2, kitchen, balcony alcove, home), 0 collisions, 43 m, ~131 s. With the camera: NOT working. DA3-L takes ~3 s per frame on this CPU (input is fixed 504x336), and its metric depth does not match the true distances in this render (camera 0.9-1.5 m where the plan shows 4.9 m free), so avoidance is wrong and slow.
* Bugs found and fixed in `controllers/fc_v4_5_bridge.py` while doing this: `pub_cmd` was never created (the bridge died on the first stick release), the odometry twist is in the body frame (it was rotated by yaw again) with the right-velocity sign flipped, the hold's position term fought the stick, and the yaw target could wind up.
* Always kill leftovers between runs (`pkill -9 parameter_bridg; pkill -9 -f "gz sim -s"`): a stray simulation publishes a second odometry.

## Real-link mode (2026-11-21): `../sim.sh start | stop | status`
The GCS flies the Gazebo drone through the same paths as the real drone: simulated ELRS radio (`controllers/sim_radio.py`, CRSF on a pty at `~/.config/droneforge/sim_radio`),
O4-style raw H.264 video on udp 3000 (`vision/video_udp_out.py`), RTAB-Map 2D map on tcp 5765, the explorer steering through the GCS EXPLORE button (`house_explore.py --via-gcs`, udp 5766).
In the GCS: ARM, TAKE OFF, wait for HOLDING, press EXPLORE; LAND or SAFE end it. `run_house.sh --gcs` is what `sim.sh` runs. Logs: `~/.local/state/droneforge/sim/` and `/tmp/droneforge_*.log`.
The rangefinder is a `gpu_lidar` (a `type="ray"` sensor is never created in Gazebo Harmonic).
