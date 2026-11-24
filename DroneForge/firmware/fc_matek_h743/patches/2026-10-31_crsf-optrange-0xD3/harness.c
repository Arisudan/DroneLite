// Host-side harness: compiles the patched crsfFrameOptrange() against mocks and prints the frame it builds.
// usage (stdin lines): <distance_mm> <quality> <healthy> <validX> <validY> <rateX rad/s> <rateY rad/s> <flowQuality>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

typedef struct { uint8_t *ptr; uint8_t *end; } sbuf_t;
static void sbufWriteU8(sbuf_t *d, uint8_t v) { *d->ptr++ = v; }
static void sbufWriteU16BigEndian(sbuf_t *d, uint16_t v) { sbufWriteU8(d, (uint8_t)(v >> 8)); sbufWriteU8(d, (uint8_t)v); }
#define constrain(a, lo, hi) ((a) < (lo) ? (lo) : ((a) > (hi) ? (hi) : (a)))
#define constrainf(a, lo, hi) constrain(a, lo, hi)
enum { CRSF_FRAME_OPTRANGE_PAYLOAD_SIZE = 10, CRSF_FRAME_LENGTH_TYPE_CRC = 2, CRSF_FRAMETYPE_OPTRANGE = 0xD3 };
typedef struct { float x, y; } vector2_t;
typedef struct { int16_t quality; vector2_t rawFlowRates, processedFlowRates; bool flowRateValid[2]; uint32_t timeStampUs; } opticalflow_t;
static int32_t g_mm; static uint8_t g_q; static opticalflow_t g_flow; static bool g_healthy;
static int32_t mtRangefinderGetLastDistanceMm(void) { return g_mm; }
static uint8_t mtRangefinderGetLastQuality(void) { return g_q; }
static const opticalflow_t *getOpticalFlowData(void) { return &g_flow; }
static bool isOpticalflowHealthy(void) { return g_healthy; }

#include "optrange_fn.inc"

static uint8_t crc8(const uint8_t *p, int n) {
    uint8_t c = 0;
    while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
    return c;
}
int main(void) {
    int mm, q, healthy, vx, vy, fq; float rx, ry;
    while (scanf("%d %d %d %d %d %f %f %d", &mm, &q, &healthy, &vx, &vy, &rx, &ry, &fq) == 8) {
        g_mm = mm; g_q = (uint8_t)q; g_healthy = healthy; g_flow.flowRateValid[0] = vx; g_flow.flowRateValid[1] = vy;
        g_flow.processedFlowRates.x = rx; g_flow.processedFlowRates.y = ry; g_flow.quality = (int16_t)fq;
        uint8_t buf[32]; sbuf_t d = { buf, buf + sizeof buf };
        sbufWriteU8(&d, 0xC8);                       // CRSF_SYNC_BYTE (crsfInitializeFrame)
        crsfFrameOptrange(&d);
        int n = (int)(d.ptr - buf);
        buf[n] = crc8(&buf[2], n - 2); n++;          // crsfFinalize: CRC over type + payload
        for (int i = 0; i < n; i++) printf("%02x", buf[i]);
        printf("\n");
    }
    return 0;
}
