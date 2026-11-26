#include "estimator_gates.h"
#include <math.h>

static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

void egFlatlineInit(egFlatline_t *f) { f->sig = 0; f->t = 0.0f; f->primed = false; }

bool egFlatlineUpdate(egFlatline_t *f, float dt, uint32_t signature) {
    if (!f->primed || signature != f->sig) { f->sig = signature; f->t = 0.0f; f->primed = true; return false; }
    f->t += dt;
    return f->t >= EG_FLATLINE_S;
}

uint32_t egPayloadSignature(int32_t distance, uint32_t strength, uint32_t precision, int32_t flow_x, int32_t flow_y, uint32_t quality) {
    uint32_t h = 2166136261u;                       /* FNV-1a over the six fields */
    const uint32_t v[6] = { (uint32_t)distance, strength, precision, (uint32_t)flow_x, (uint32_t)flow_y, quality };
    for (int i = 0; i < 6; i++) { h ^= v[i]; h *= 16777619u; h ^= h >> 15; }
    return h;
}

float egRangeVarianceScale(float base_scale, int strength, float dist_m) {
    if (strength >= EG_RANGE_STRENGTH_SAT || dist_m < EG_RANGE_NEAR_M) return base_scale * EG_RANGE_ZONE_SCALE;
    return base_scale;
}

float egFlowVarianceScale(float base_scale, float roll_rate_dps, float pitch_rate_dps) {
    const float w_dps = sqrtf(roll_rate_dps * roll_rate_dps + pitch_rate_dps * pitch_rate_dps);
    const float w = w_dps * 0.017453292f;
    float infl = 1.0f + 2.0f * w * w;
    if (w_dps > EG_FLOW_RATE_START_DPS) {
        const float t = clamp01((w_dps - EG_FLOW_RATE_START_DPS) / (EG_FLOW_RATE_FULL_DPS - EG_FLOW_RATE_START_DPS));
        const float target = infl > EG_FLOW_RATE_MAX_SCALE ? infl : EG_FLOW_RATE_MAX_SCALE;
        infl += t * t * (target - infl);
    }
    return base_scale * infl;
}

float egFlowYawScale(float yaw_rate_rad_s) {
    const float a = fabsf(yaw_rate_rad_s);
    if (a < EG_YAW_INFLATE_START) return 1.0f;
    return 1.0f + (EG_YAW_MAX_SCALE - 1.0f) * clamp01((a - EG_YAW_INFLATE_START) / (EG_YAW_SKIP - EG_YAW_INFLATE_START));
}

bool egSkipFlow(float yaw_rate_rad_s) { return fabsf(yaw_rate_rad_s) > EG_YAW_SKIP; }

void egFlowGateInit(egFlowGate_t *g) { g->rejected = 0; }

bool egFlowGate(egFlowGate_t *g, float mx, float my, float px, float py) {
    const bool bad = !(fabsf(mx - px) <= EG_FLOW_GATE_MS) || !(fabsf(my - py) <= EG_FLOW_GATE_MS);
    if (bad && g->rejected < EG_FLOW_GATE_MAX_REJECT) { g->rejected++; return false; }
    g->rejected = 0;
    return true;
}
