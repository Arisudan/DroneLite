#include "crsf_state_pack.h"

static uint8_t *put16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)(v & 0xFF); return p; }

uint8_t crsfStatePack(uint8_t *out, uint16_t distRawMm, int16_t flowX, int16_t flowY, uint8_t flowQuality, bool ekfValid, uint8_t flowStatus,
                      int16_t velXmm, int16_t velYmm, int16_t velZmm, int16_t rollE4, int16_t pitchE4, int16_t yawE4) {
  uint8_t *p = out;
  *p++ = 0x00;                                   /* dest: broadcast */
  *p++ = 0xC8;                                   /* origin: flight controller */
  p = put16(p, distRawMm);
  p = put16(p, (uint16_t)flowX);
  p = put16(p, (uint16_t)flowY);
  *p++ = flowQuality;
  *p++ = (uint8_t)((ekfValid ? 1U : 0U) | (flowStatus ? 2U : 0U));
  p = put16(p, (uint16_t)velXmm);
  p = put16(p, (uint16_t)velYmm);
  p = put16(p, (uint16_t)velZmm);
  p = put16(p, (uint16_t)rollE4);
  p = put16(p, (uint16_t)pitchE4);
  p = put16(p, (uint16_t)yawE4);
  return (uint8_t)(p - out);
}
