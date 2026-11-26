v4.8 = v4.7.4R1 + sooner lift-off + VO aid usable in the climb + shorter state frame.   BUILT 2026-11-26 10:35:37, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 26 2026 10:35:37" after a flash.

BEFORE FLASHING: update the GCS on EVERY laptop that will connect (this commit or later). An older GCS cannot read the shorter state frame: no height, no hold status.
Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-26_fc-hybrid-v4.8_BUILT_NOT-FLASHED/fc-hybrid-v4.8_matek-h743_betaflight-4.5.4.bin --debug-hold
BACK to what flies now: the same command with builds/2026-11-25_fc-hybrid-v4.7.4R_BUILT_NOT-FLASHED/*.bin
NOTE: the base is R1, so relative to the drone's current v4.7.4R this also brings R1's softer height gains (0.9923 / 0.10 / 0.04) and sideways kp 2.5 / ki 0.10.

CHANGES (v4.7.4R1_to_v4.8.patch)
 1. height_hold_core.c  takeoff_start_frac 0.82 -> 0.92 (creep starts at 0.61 instead of 0.54 at hover reference 0.66). Creep rates unchanged.
    Why: replaying 6 flights of 11-25 / 11-26 through the law, the drone was 8 cm up at throttle 0.72-0.73 every time.
    Simulation: lift-off and HOLDING about 2 s sooner, worst overshoot / climb speed about unchanged (0.95 was worse). The simulator does NOT model tip-over:
    v4.2 (start 0.85 with a faster creep) tipped over on the floor. FIRST FLIGHT TETHERED OR OVER A SOFT, CLEAR AREA.
 2. xy_hold_core.c/.h  climb_aid = 1: a fresh PC aid also acts in the climb. Inert unless the GCS is started with DF_GROUND_AID=1 (default off).
    Simulation: drift from the take-off spot 0.37 -> 0.23 m (median). The aid's sign has never been proven in flight: prove it with the aid from HOLD first.
 3. crsf_state_pack.c/.h, crsf_protocol.h  state frame 20 bytes on the wire instead of 26. Velocity x/y 1 mm/s (+-2.047 m/s), vz 0.01 m/s, roll/pitch 0.5 deg
    (+-63 deg), flow raw in steps of 2 (+-254), distance, yaw, flags, quality unchanged. Expected rate gain up to ~1.4x ONLY IF ELRS carries 10 bytes per telemetry
    packet in this mode (assumed, not confirmed). Check disarmed in DIAGNOSTICS (state frame Hz; v4.7: ~12-16 Hz).
NOT CHANGED: the take-off bobbing of 11-25. The replay shows the commanded throttle was still RISING while the drone sank from 0.30 to 0.02 m, so the fall was not
 commanded by the take-off law. Suspect thrust not following the throttle (motor / prop / ESC, M3 imbalance) or the range reading: needs the FC blackbox of such a flight.

BUILD: copy of ~/droneforge/bf_fixed/repo (rev1) + R1's src/ = rebuilt R1 identical to the shipped R1 .bin except build time/date/git hash (11 bytes).
 Work tree ~/droneforge/bf_v48/repo; toolchain tools/gcc-arm-none-eabi-10.3-2021.10 inside it. 807,941 bytes, 0 compiler warnings.
 make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config ARM_SDK_DIR=<toolchain> OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"
TESTS: tools_scripts/fc_hold_sim/test_v48.py; software/pc/nimbus_pc/tests/test_state_frame_v48.py. Simulator: DF_FW=v48.
