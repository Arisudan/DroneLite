#!/usr/bin/env bash
# DroneForge Gazebo Simulation Runner
# Launches Gazebo Harmonic with the 382g 3-inch drone, DJI O4 camera feed,
# ros_gz_bridge, Betaflight v4.5 FC bridge, and the real-time FPV camera viewer.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models:$GZ_SIM_RESOURCE_PATH"
export SDF_PATH="$SCRIPT_DIR/models:$SDF_PATH"

# Check for auto and tx flags
AUTO_FLAG=""
if [[ "$*" == *"--auto"* ]] || [[ "$*" == *"auto"* ]]; then
    AUTO_FLAG="--auto"
fi

ENABLE_TX=0
if [[ "$*" == *"--tx"* ]] || [ -e "/dev/ttyUSB0" ]; then
    ENABLE_TX=1
fi

# Source ROS 2 Jazzy environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found."
    exit 1
fi

echo "=========================================================="
echo "    DroneForge Gazebo Simulation (v4.5 Firmware Core)     "
echo "=========================================================="
echo "Drone: 3-inch Quad-X (~382.5 g AUW, 4S LiPo, Matek H743)"
echo "Sensors: DJI O4 FPV Camera, MTF-02 TOF Range + Optical Flow, IMU"
echo "World: droneforge_testbench.sdf"
if [ -n "$AUTO_FLAG" ]; then
    echo "Mode: AUTO-TAKEOFF (Will arm & hover at 0.50 m automatically)"
fi
if [ "$ENABLE_TX" -eq 1 ]; then
    echo "Hardware Bridge: ELRS TX (/dev/ttyUSB0) & FC VCP enabled"
fi
echo "=========================================================="

PIDS=()
cleanup() {
    echo ""
    echo "Stopping all simulation processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGTERM "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# 1. Start Gazebo Sim
echo "[1/4] Starting Gazebo Harmonic..."
gz sim -r "$SCRIPT_DIR/worlds/droneforge_testbench.sdf" &
PIDS+=($!)
sleep 3

# 2. Start ROS-GZ Parameter Bridge
echo "[2/5] Starting ROS-GZ Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /droneforge/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators \
    /droneforge/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /droneforge/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    /droneforge/sensors/rangefinder@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /droneforge/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry &
PIDS+=($!)
sleep 2

# 4. Start DJI O4 Camera Viewer & Telemetry HUD
echo "[3/5] Starting DJI O4 Camera Viewer..."
python3 "$SCRIPT_DIR/viewers/camera_viewer.py" &
PIDS+=($!)
sleep 1

# 5. Start Betaflight v4.5 FC Bridge
echo "[4/5] Starting Betaflight v4.5 Flight Controller Bridge..."
python3 "$SCRIPT_DIR/controllers/fc_v4_5_bridge.py" $AUTO_FLAG &
PIDS+=($!)
sleep 1

# 6. Start TX & Hardware Bridge if enabled
if [ "$ENABLE_TX" -eq 1 ]; then
    echo "[5/5] Starting ELRS TX & Hardware-in-Loop Bridge..."
    python3 "$SCRIPT_DIR/controllers/tx_sim_bridge.py" &
    PIDS+=($!)
fi

echo ""
echo "=========================================================="
echo "  SIMULATION RUNNING! How to control the drone:"
echo "    METHOD 1: In Camera Viewer window, press keys:"
echo "      [A] Arm | [T] Takeoff (0.50m) | [H] Hold | [L] Land | [D] Disarm"
echo "    METHOD 2: Or directly in this terminal, type:"
echo "      't' + Enter (Takeoff)  |  'l' + Enter (Land)  |  'd' + Enter (Disarm)"
echo "=========================================================="
echo ""

# Keep running until user terminates
wait "${PIDS[2]}" 2>/dev/null || wait "${PIDS[0]}" 2>/dev/null
