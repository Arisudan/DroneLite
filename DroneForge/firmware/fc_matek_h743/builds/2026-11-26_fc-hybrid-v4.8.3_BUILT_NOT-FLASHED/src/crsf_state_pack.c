#include "crsf_state_pack.h"

static uint8_t *put16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)(v & 0xFF); return p; }
/* v / div rounded to the nearest (halves away from zero), clamped to +-lim */
static int32_t scaled(int32_t v, int32_t mul, int32_t div, int32_t lim) {
  const int32_t n = v * mul;
  const int32_t r = (n >= 0) ? (n + div / 2) / div : -((-n + div / 2) / div);
  return r > lim ? lim : (r < -lim ? -lim : r);
}

uint8_t crsfStatePack(uint8_t *out, uint16_t distRawMm, int16_t flowX, int16_t flowY, uint8_t flowQuality, bool ekfValid, uint8_t flowStatus, uint8_t holdBits,
                      int16_t velXmm, int16_t velYmm, int16_t velZmm, int16_t rollE4, int16_t pitchE4, int16_t yawE4) {
  uint8_t *p = out;
  *p++ = 0x00;                                   /* dest: broadcast */
  *p++ = 0xC8;                                   /* origin: flight controller */
  p = put16(p, distRawMm);
  *p++ = (uint8_t)((ekfValid ? 1U : 0U) | (flowStatus ? 2U : 0U) | (holdBits & 0xBCU) | 0x40U);          /* bit6: the velocity is in the drone's axes (forward, right) */
  *p++ = flowQuality;
  const uint32_t vx = (uint32_t)scaled(velXmm, 1, 1, 2047) & 0xFFFU;
  const uint32_t vy = (uint32_t)scaled(velYmm, 1, 1, 2047) & 0xFFFU;
  *p++ = (uint8_t)(vx >> 4);
  *p++ = (uint8_t)(((vx & 0xFU) << 4) | (vy >> 8));
  *p++ = (uint8_t)(vy & 0xFFU);
  *p++ = (uint8_t)(int8_t)scaled(velZmm, 1, 10, 127);
  *p++ = (uint8_t)(int8_t)scaled(rollE4, 1000, 87266, 127);       /* 0.5 deg = 87.266 * 1e-4 rad */
  *p++ = (uint8_t)(int8_t)scaled(pitchE4, 1000, 87266, 127);
  p = put16(p, (uint16_t)yawE4);
  *p++ = (uint8_t)(int8_t)scaled(flowX, 1, 2, 127);
  *p++ = (uint8_t)(int8_t)scaled(flowY, 1, 2, 127);
  return (uint8_t)(p - out);
}
