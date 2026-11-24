#pragma once
/* Pure C estimator helpers (no Betaflight includes): unit-tested on the PC. Numbers from docs/nimbus_breakdown/04_estimation_and_safety.md. */
#include <stdbool.h>
#include <stdint.h>

#define EG_RANGE_ZONE_SCALE 6.0f
#define EG_RANGE_STRENGTH_SAT 250
#define EG_RANGE_NEAR_M 0.35f
#define EG_FLOW_RATE_START_DPS 20.0f
#define EG_FLOW_RATE_FULL_DPS 80.0f
#define EG_FLOW_RATE_MAX_SCALE 25.0f
#define EG_YAW_INFLATE_START 0.75f
#define EG_YAW_SKIP 1.35f
#define EG_YAW_MAX_SCALE 51.0f
#define EG_FLOW_GATE_MS 1.0f
#define EG_FLOW_GATE_MAX_REJECT 5

/* Frozen range payload (Nimbus OptrangeFailsafe 'flatline'): the whole sample (distance, strength, precision, flow x/y, flow quality) identical for this long while flying = the sensor
 * is stuck. Nimbus's threshold is not recoverable from the binary (read from a config at start-up); 1.5 s of six identical fields is practically a frozen sensor. */
#define EG_FLATLINE_S 1.5f
typedef struct { uint32_t sig; float t; bool primed; } egFlatline_t;
void egFlatlineInit(egFlatline_t *f);
/* feed the payload signature once per control step; true once it has not changed for EG_FLATLINE_S */
bool egFlatlineUpdate(egFlatline_t *f, float dt, uint32_t signature);
uint32_t egPayloadSignature(int32_t distance, uint32_t strength, uint32_t precision, int32_t flow_x, int32_t flow_y, uint32_t quality);

float egRangeVarianceScale(float base_scale, int strength, float dist_m);
float egFlowVarianceScale(float base_scale, float roll_rate_dps, float pitch_rate_dps);
float egFlowYawScale(float yaw_rate_rad_s);
bool egSkipFlow(float yaw_rate_rad_s);

typedef struct { int rejected; } egFlowGate_t;
void egFlowGateInit(egFlowGate_t *g);
/* true = accept the sample. Per-axis |meas - pred| > 1.0 m/s rejects, but the 6th consecutive rejection is accepted (no lock-out). */
bool egFlowGate(egFlowGate_t *g, float meas_x, float meas_y, float pred_x, float pred_y);
