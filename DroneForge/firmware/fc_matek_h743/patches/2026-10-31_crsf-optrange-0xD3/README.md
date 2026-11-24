# Betaflight patch: send NimbusOS-style range + optical-flow telemetry (CRSF frame 0xD3)

`0001-crsf-add-optrange-frame-0xD3.patch` — a unified diff against Betaflight **master** (files as of 2026-10-31):
`src/main/rx/crsf_protocol.h`, `src/main/telemetry/crsf.c`, `src/main/drivers/rangefinder/rangefinder_lidarmt.{c,h}`.
Derivative of GPLv3 Betaflight code, so it stays GPLv3.

## What it does
- Adds frame type `0xD3` (payload 10 bytes) as a **timed** CRSF telemetry frame, 20 Hz (`CRSF_OPTRANGE_INTERVAL_US`).
- Layout (big-endian) is exactly what NimbusOS decodes, see `../../../docs/specs/crsf-link_protocol-spec.md` section 5c:
  distance (int16, mm, 0 if outside 1..5999), strength u8, precision u8, flow_x i16, flow_y i16 (0.01 rad/s),
  flow quality u8, flow status u8.
- The MT lidar driver now keeps the raw millimetre distance and the sensor's quality byte (Betaflight's rangefinder
  API otherwise reduces distance to centimetres).
- Frame is enabled only when the configured rangefinder is an MT sensor (`rangefinderTypeUsesMsp`), and is sent
  even when the sensor stalls: the PC watchdog detects a frozen sensor by seeing identical frames.

## Verified (host only)
`./check.sh` compiles the patched `crsfFrameOptrange()` against mocks with `gcc -Wall -Wextra` (no warnings) and decodes
its output with `nimbus_pc`: 10 scenarios (normal, close range, 5999/6000 mm limits, no data, invalid or unhealthy flow,
int16 and u8 clamping) all pass, including the CRC. The patch applies cleanly (`patch -p1 --dry-run`) to pristine master files.

## NOT verified — read before using
- **Never compiled as firmware and never flashed.** No ARM toolchain or Betaflight build was done here.
- **Your FC is not built from this source.** It reports `opticalflow_hardware = MTF02` and parses the sensor's native
  frames, while master uses `OPTICALFLOW_MT` fed by MSP sensor messages. Your build (4.5.4, "norevision", 2026-10-09,
  config rev 18ffb2a) comes from some other branch or PR. Names, accessors and the driver may differ; find the real source
  first, then re-base this patch (the logic is small).
- Sensor mode: master's MT driver expects the sensor to push MSP; your MTF-02P was measured sending native 0xEF frames.
- **Unknown choices:** whether Nimbus sends raw or gyro-compensated flow (this patch sends compensated), and the axis
  signs. Calibrate on the bench: move the drone forward and right and check the PC's `velocity_forward/right` signs.
- **Strength/precision:** precision is not available (always 0); strength is the lidar quality byte, probably not
  identical to what Nimbus sends.
- **Rate is a guess (20 Hz).** ELRS downlink telemetry bandwidth is small and shared with battery, attitude, link stats.
  Check the other telemetry still arrives.
- **Whether the ELRS receiver forwards type 0xD3** to the transmitter is untested.

## Using it
1. Identify the exact source your FC firmware was built from (ask the build's author, or the Betaflight Cloud build page
   for that build).
2. `git apply` / re-base the patch there, build for MATEKH743, flash.
3. With the radio link up, the PC sender decodes frames as `[telemetry] optrange: {...}`; add `--watch-optrange` to
   enable the sensor watchdog.
