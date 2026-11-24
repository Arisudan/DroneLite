#pragma once
/* Compact CRSF state frame 0xD6 (NIMBUS_SEND_STATE): range + optical flow + EKF velocity + attitude in ONE frame, replacing the D3 + D4 + ATTITUDE frames.
 * Pure C (no Betaflight headers) so the byte layout is tested on the PC against the Python decoder (nimbus_pc/crsf.py decode_state). */
#include <stdbool.h>
#include <stdint.h>

#define CRSF_STATE_DATA_BYTES 20      /* big-endian: dist i16 mm, flow x i16, flow y i16, quality u8, flags u8, vel x/y/z i16 mm/s, roll/pitch/yaw i16 rad*1e4 */
#define CRSF_STATE_PAYLOAD_BYTES 22   /* [dest 0x00][origin 0xC8] + the 20 data bytes (ELRS 4.x reads types >= 0x28 as addressed frames) */

/* flags: bit0 = EKF velocity/attitude valid, bit1 = optical flow status ok. Returns the payload size written to out[]. */
uint8_t crsfStatePack(uint8_t *out, uint16_t distRawMm, int16_t flowX, int16_t flowY, uint8_t flowQuality, bool ekfValid, uint8_t flowStatus,
                      int16_t velXmm, int16_t velYmm, int16_t velZmm, int16_t rollE4, int16_t pitchE4, int16_t yawE4);
