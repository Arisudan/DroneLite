#include <math.h>
#include <string.h>
#include "xy_hold_core.h"

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void xyDefaults(xyParams_t *p)
{
    p->kv = 5.0f; p->kp = 2.5f; p->ki = 0.10f; p->max_deg = 4.0f; p->min_z = 0.06f; p->test_z = 0.08f;
    p->test_deg = 2.5f; p->test_s = 0.8f; p->test_min_dv = 0.08f; p->calm_speed = 0.50f; p->calm_s = 0.5f;
    p->runaway_speed = 0.8f; p->runaway_s = 1.0f; p->runaway_pos = 1.5f;
}

void xyStart(xyState_t *s) { memset(s, 0, sizeof(*s)); s->active = true; }
void xyStop(xyState_t *s) { s->active = false; s->out[0] = s->out[1] = 0.0f; }

void xyUpdate(xyState_t *s, const xyParams_t *p, float dt, float forward, float right, float z, bool hold, float *roll_deg, float *pitch_deg)
{
    *roll_deg = 0.0f; *pitch_deg = 0.0f;
    if (!s->active || s->disabled || z < p->min_z) { s->out[0] = s->out[1] = 0.0f; return; }
    /* the speed measurement is noisy (flow + EKF): the sign test works on a low-passed copy (tau 0.15 s) */
    const float a_lp = dt / (0.15f + dt);
    s->vf[0] += (forward - s->vf[0]) * a_lp; s->vf[1] += (right - s->vf[1]) * a_lp;
    const float v[2] = { forward, right };          /* index 0 = pitch axis (forward), 1 = roll axis (right) */
    const float spd = sqrtf(forward * forward + right * right);

    /* runaway guard */
    s->t_run = spd > p->runaway_speed ? s->t_run + dt : (s->t_run > dt ? s->t_run - dt : 0.0f);
    const float dist = sqrtf(s->p[0] * s->p[0] + s->p[1] * s->p[1]);
    if (s->t_run > p->runaway_s || dist > p->runaway_pos) { s->disabled = true; s->out[0] = s->out[1] = 0.0f; return; }

    /* speed 0.6 s ago, for the drift acceleration just before a test */
    s->t_hist += dt;
    if (s->t_hist >= p->test_s) { s->v_hist[0] = s->vf[0]; s->v_hist[1] = s->vf[1]; s->t_hist = 0.0f; }

    /* self-test of the signs, once per flight, on both axes at the same time (they are independent): wait until calm and high enough, tilt both axes by a small angle for
     * a moment and look at the speed change. Until it is done the loop gives no output (an untested sign could be positive feedback). */
    float test_tilt[2] = { 0.0f, 0.0f };
    bool in_test = false;
    if (s->phase < 5) {
        const bool calm = spd < p->calm_speed && z >= p->test_z && hold;
        if (s->phase == 0) {
            s->t_calm = calm ? s->t_calm + dt : 0.0f;
            if (s->t_calm >= p->calm_s) {
                for (int ax = 0; ax < 2; ax++) {
                    s->a_pre[ax] = (s->vf[ax] - s->v_hist[ax]) / (p->test_s > 0.0f ? p->test_s : 1.0f);
                    s->v_start[ax] = s->vf[ax];
                }
                s->phase = 1; s->t_phase = 0.0f;
            }
        }
        if (s->phase == 1) {
            test_tilt[0] = test_tilt[1] = p->test_deg; in_test = true; s->t_phase += dt;
            if (s->t_phase >= p->test_s) {
                for (int ax = 0; ax < 2; ax++) {
                    const float dv = (s->vf[ax] - s->v_start[ax]) - s->a_pre[ax] * s->t_phase;
                    if (dv > p->test_min_dv) s->sign[ax] = 1.0f;
                    else if (dv < -p->test_min_dv) s->sign[ax] = -1.0f;
                    else { s->sign[ax] = 0.0f; s->axis_off[ax] = true; }
                }
                s->phase = 5; in_test = false;
            }
        }
    }
    /* position terms: displacement since the loop started (no yaw change), its integral learns a constant level error */
    if (hold && !in_test) {
        s->p[0] += forward * dt; s->p[1] += right * dt;
        s->i[0] += s->p[0] * dt; s->i[1] += s->p[1] * dt;
        const float lim = p->max_deg / (p->ki > 1e-6f ? p->ki : 1e-6f);
        s->i[0] = clampf_(s->i[0], -lim, lim); s->i[1] = clampf_(s->i[1], -lim, lim);
    }
    float out[2] = { 0.0f, 0.0f };
    for (int ax = 0; ax < 2; ax++) {
        if (in_test) { out[ax] = test_tilt[ax]; continue; }
        if (s->axis_off[ax]) continue;
        float sg = s->sign[ax];
        if (sg == 0.0f) {                           /* not tested yet: gentle speed damping with the nominal sign (measured speed grows with +tilt), low authority, no position terms; the self-test starts as soon as it is calm */
            out[ax] = -clampf_(p->kv * v[ax], -1.5f, 1.5f);
            continue;
        }
        const float kp = hold ? p->kp : 0.0f, ki = hold ? p->ki : 0.0f;
        out[ax] = -sg * (p->kv * v[ax] + kp * s->p[ax] + ki * s->i[ax]);
        out[ax] = clampf_(out[ax], -p->max_deg, p->max_deg);
    }
    s->out[0] = out[1]; s->out[1] = out[0];
    *pitch_deg = out[0]; *roll_deg = out[1];
}
