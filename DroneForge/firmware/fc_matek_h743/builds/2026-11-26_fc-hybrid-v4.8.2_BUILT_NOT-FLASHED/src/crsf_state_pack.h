#pragma once
/* Compact CRSF state frame 0xD6 (NIMBUS_SEND_STATE): range + optical flow + EKF velocity + attitude in ONE frame.
 * v4.8 layout: 14 data bytes (20 on the wire; v4.7: 20 data / 26 on the wire), so ELRS needs fewer telemetry packets per frame.
 * Pure C (no Betaflight headers): the byte layout is tested on the PC against the Python decoder (nimbus_pc/crsf.py decode_state). */
#include <stdbool.h>
#include <stdint.h>

#define CRSF_STATE_DATA_BYTES 14
#define CRSF_STATE_PAYLOAD_BYTES 16   /* [dest 0x00][origin 0xC8] + the 14 data bytes (ELRS 4.x reads types >= 0x28 as addressed frames) */

/* data bytes (big-endian): 0-1 distance u16 mm | 2 flags | 3 flow quality | 4-6 velocity x, y as two signed 12-bit numbers, mm/s (+-2.047 m/s) |
 * 7 velocity z i8, 0.01 m/s | 8 roll i8, 0.5 deg | 9 pitch i8, 0.5 deg | 10-11 yaw i16 rad*1e4 | 12 flow x i8, raw/2 | 13 flow y i8, raw/2. Values outside a field saturate.
 * flags: bit0 = EKF velocity/attitude valid, bit1 = optical flow status ok, bit2 = FC height hold active, bit3 = FC height hold fault, bits 4-5 = hold mode (0 hold, 1 take-off, 2 land),
 * bit6 = velocity x/y are BODY-frame (forward, right), bit7 = the FC sideways loop switched itself off (runaway guard). holdBits carries bits 2-5 and 7 in place.
 * The inputs keep the v4.7 units (mm/s, rad*1e4, raw flow). Returns the payload size written to out[]. */
uint8_t crsfStatePack(uint8_t *out, uint16_t distRawMm, int16_t flowX, int16_t flowY, uint8_t flowQuality, bool ekfValid, uint8_t flowStatus, uint8_t holdBits,
                      int16_t velXmm, int16_t velYmm, int16_t velZmm, int16_t rollE4, int16_t pitchE4, int16_t yawE4);
