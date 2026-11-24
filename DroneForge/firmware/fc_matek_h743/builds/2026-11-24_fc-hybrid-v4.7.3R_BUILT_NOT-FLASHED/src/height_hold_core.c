#include <string.h>
#include "height_hold_core.h"

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float absf_(float v) { return v < 0.0f ? -v : v; }

/* reading -> true height above the ground (points from the PC's range_cal, 0.07 m ground reading: (0.29,0.20), (0.41,0.32), (0.45,0.45) -> above the ground); identity above 0.38 m and below 0 */
float hhCorrectHeight(float z)
{
    static const float zr[] = {0.00f, 0.22f, 0.34f, 0.38f};
    static const float zt[] = {0.00f, 0.13f, 0.25f, 0.38f};
    if (z <= 0.0f || z >= zr[3]) return z;
    for (int i = 1; i < 4; i++) {
        if (z <= zr[i]) return zt[i - 1] + (zt[i] - zt[i - 1]) * (z - zr[i - 1]) / (zr[i] - zr[i - 1]);
    }
    return z;
}

void hhDefaults(hhParams_t *p)
{
    p->kp_z = 1.40f; p->kv_z = 0.15f; p->ki_z = 0.06f; p->i_max = 0.20f;      /* v4.7.3R: ki 0.06 as v4.6-rev1 (v4.7.1 / v4.7.2: 0.15) */      /* v4.6 tighter height hold (was kp 0.9923, kv 0.10, ki 0.04): stiffer tracking and firmer vertical damping */
    p->vz_lim = 0.4581f; p->climb_peak_frac = 0.45f; p->descent_peak_frac = 0.45f;      /* v4.7.3R: 0.45 as v4.6-rev1 (v4.7.1 / v4.7.2: 0.35) */
    p->thr_idle = 0.05f; p->thr_max = 0.85f; p->drop_max = 0.30f;
    p->overshoot_m = 0.10f; p->overshoot_vz = 0.40f; p->overshoot_cut = 0.08f;
    p->height_min = 0.2f; p->height_max = 2.5f;
    p->base_tau = 1.5f; p->base_follow_max = 0.06f; p->lift_vz = 0.05f; p->lift_m = 0.15f;
    p->takeoff_start_frac = 0.82f; p->takeoff_creep_per_s = 0.010f; p->takeoff_fast_per_s = 0.03f; p->takeoff_cap_frac = 1.40f; p->takeoff_fail_s = 3.0f;
    p->land_rate = 0.16f; p->land_slow_rate = 0.07f; p->land_slow_below = 0.15f;
    p->touch_m = 0.07f; p->touch_s = 0.30f; p->idle_ramp_per_s = 1.5f; p->takeoff_ramp_per_s = 0.35f; p->cap_land_s = 1.5f; p->lift_base_boost = 0.0f; p->takeoff_vz_max = 0.30f; p->takeoff_vz_back = 0.05f; p->takeoff_handover_margin = 0.05f;
    p->takeoff_closed = 0.0f;      /* v4.7.3R: 0 = the v4.6-rev1 open-loop take-off creep (v4.7 / v4.7.2: 1) */
    p->az_ff_gain = 0.0f;          /* v4.7.3R: 0 = no acceleration feed-forward (the v4.7.1 / v4.7.2 value is 1) */
}

void hhEngage(hhState_t *s, const hhParams_t *p, float z, float f0, float vbat)
{
    memset(s, 0, sizeof(*s));
    s->active = true; s->mode = HH_MODE_HOLD;
    s->z_ref = z; s->target = z;
    s->f_base = clampf_(f0, p->thr_idle, p->thr_max);
    s->v0 = vbat > 6.0f ? vbat : 6.0f;
    s->f_out = s->f_base;
}

void hhDisengage(hhState_t *s) { s->active = false; s->trans = false; s->vz_ff = 0.0f; }

void hhTakeoffBegin(hhState_t *s, const hhParams_t *p, float f_hover, float target, float vbat)
{
    memset(s, 0, sizeof(*s));
    s->active = true; s->mode = HH_MODE_TAKEOFF;
    s->f_base = clampf_(f_hover, p->thr_idle, p->thr_max);
    s->v0 = vbat > 6.0f ? vbat : 6.0f;
    s->f_start = clampf_(p->takeoff_start_frac * s->f_base, p->thr_idle, p->thr_max);
    s->f_cur = p->thr_idle;                                    /* start AT IDLE and ramp: a step to f_start would be amplified by the throttle boost into a full-throttle burst */
    s->target_pending = clampf_(target, p->height_min, p->height_max);
    s->f_out = s->f_cur;
}

void hhSetTarget(hhState_t *s, const hhParams_t *p, float h)
{
    h = clampf_(h, p->height_min, p->height_max);
    if (s->mode == HH_MODE_TAKEOFF) { s->target_pending = h; return; }
    if (s->mode == HH_MODE_LAND) return;
    if (absf_(h - s->target) < 0.02f) return;                  /* ignore RC quantisation jitter */
    s->target = h;
    const float dz = absf_(h - s->z_ref);
    const float peak = (h > s->z_ref ? p->climb_peak_frac : p->descent_peak_frac) * p->vz_lim;
    float T = 1.875f * dz / peak;
    if (T < 1.5f) T = 1.5f;
    s->trans = true; s->t_start = s->t; s->t_dur = T; s->z_from = s->z_ref; s->z_to = h; s->v_from = 0.0f;
}

/* v4.7: a transition that STARTS at the drone's current climb speed v0 (quintic from (z, v0, 0) to (h, 0, 0); monotonic for v0 * T <= 2 * distance) */
static void hhSetTargetFromSpeed(hhState_t *s, const hhParams_t *p, float h, float v0)
{
    hhSetTarget(s, p, h);
    if (!s->trans) return;
    const float dz = s->z_to - s->z_from;
    if (dz <= 0.0f || v0 <= 0.0f) return;
    if (v0 * s->t_dur > 2.0f * dz) v0 = 2.0f * dz / s->t_dur;
    s->v_from = v0;
}

void hhLand(hhState_t *s, const hhParams_t *p)
{
    (void)p;
    if (!s->active || s->mode == HH_MODE_LAND) return;
    if (s->mode == HH_MODE_TAKEOFF) { s->f_out = s->f_cur; }
    s->mode = HH_MODE_LAND; s->t_mode = 0.0f; s->t_touch = 0.0f; s->trans = false; s->landed = false;
}

void hhFollowBase(hhState_t *s, const hhParams_t *p, float dt, float f_hover, float vbat)
{
    if (!s->active || s->mode != HH_MODE_HOLD || s->base_learned || f_hover < 0.2f || f_hover > 0.9f) return;
    if (f_hover - s->f_base > p->base_follow_max || s->f_base - f_hover > p->base_follow_max) return;   /* the PC number disagrees a lot: it is probably wrong (voltage reading), keep ours */
    const float a = clampf_(dt / p->base_tau, 0.0f, 1.0f);
    s->f_base += (clampf_(f_hover, p->thr_idle, p->thr_max) - s->f_base) * a;
    if (vbat > 6.0f) s->v0 += (vbat - s->v0) * a;              /* the PC's number already is "at this voltage" */
}

float hhUpdate(hhState_t *s, const hhParams_t *p, float dt, float z, float vz, float vbat)
{
    z = hhCorrectHeight(z);
    s->t += dt; s->t_mode += dt;
    const float vb = vbat > 6.0f ? vbat : s->v0;

    if (s->mode == HH_MODE_TAKEOFF) {
        const float cap = clampf_(p->takeoff_cap_frac * s->f_base, p->thr_idle, p->thr_max);
        if (s->f_cur < s->f_start) { s->f_cur += p->takeoff_ramp_per_s * dt; if (s->f_cur > s->f_start) s->f_cur = s->f_start; }
        else {
            float rate = z > 0.08f ? p->takeoff_creep_per_s : p->takeoff_fast_per_s;      /* v4.5 rev3: fast until it is 8 cm up, then the slow creep (the lift-off throttle is 0.57-0.69 on this drone; simulator sweep of 6 plants: worst time to hand-over 12 s instead of 36 s, overshoot 0.60 m, vz 0.51 m/s; 0.05/s overshoots to 0.73 m) */                   /* the climb is open loop (throttle creeps up): a speed governor keeps it from becoming a burst once the drone leaves the ground-effect cushion */
            if (z > 0.25f) {                                     /* only above 0.25 m: near the floor the EKF speed reads up to +0.2 m/s with the motors spinning (measured), which would stall the creep */
                if (vz > p->takeoff_vz_max) rate = -p->takeoff_vz_back;
                else if (vz > 0.6f * p->takeoff_vz_max) rate = 0.0f;
            }
            s->f_cur += rate * dt;
        }
        if (s->f_cur >= cap) { s->f_cur = cap; s->t_cap += dt; if (s->t_cap > p->takeoff_fail_s) s->fail = true; }
        /* the throttle at which it really started to rise: a rise (z above 4 cm and vz above lift_vz) that lasts 0.3 s. One noisy speed sample at 0.5 throttle must not become the hover base (it left the hold 0.15 too low and stuck in ground effect). */
        if (!s->lifted) {
            if (z > 0.04f && vz > p->lift_vz) s->t_liftdet += dt; else s->t_liftdet = 0.0f;
            if (s->t_liftdet >= 0.3f) { s->lifted = true; s->f_lift = s->f_cur; }
        }
        /* the range over-reads by ~9 cm at 6-30 cm, so a drone still at 11 cm real height can read 0.2 m */
        /* the take-off CLIMBS to the target height; only there does the hold take over (bumpless): the drone is out of ground effect, so the throttle that got it up is the hover base */
        const bool closed = p->takeoff_closed > 0.5f && s->lifted;              /* v4.7: the hold flies the rest of the climb */
        if (z > p->lift_m && (closed || z >= s->target_pending - p->takeoff_handover_margin)) {
            /* the hover base is what PHYSICALLY lifted the drone (not the PC's reference, which depends on a voltage reading and a hover calibration) */
            float f_b = (s->lifted ? s->f_lift : s->f_cur) + p->lift_base_boost;
            /* v4.5: the output must not step at the hand-over. base + integral is set to the creep throttle f_cur, and the integral can only carry i_max: if the lift-off was detected early (a low
             * f_lift) the old base left base + i_max far below f_cur and the throttle fell by ~0.12-0.15 right after the hand-over (real flights 2026-11-16 16:48, 15:03: 0.731 -> 0.58, drone dropped
             * 0.45 -> 0.08 m). The base is therefore never lower than f_cur - i_max / 2. */
            if (f_b < s->f_cur - 0.5f * p->i_max) f_b = s->f_cur - 0.5f * p->i_max;
            s->f_base = clampf_(f_b, p->thr_idle, p->thr_max); s->v0 = vb; s->base_learned = true;
            const float base = s->f_base;
            s->mode = HH_MODE_HOLD; s->t_mode = 0.0f; s->z_ref = z; s->vz_ff = 0.0f; s->trans = false; s->overshoot = false;
            s->integ = clampf_(s->f_cur - (base - p->lift_base_boost), -p->i_max, p->i_max);       /* the output jumps by the ground-effect margin (the base includes it, the integral does not cancel it) */
            s->target = z;
            if (closed) hhSetTargetFromSpeed(s, p, s->target_pending, vz > 0.0f ? vz : 0.0f);   /* v4.7 */
            else hhSetTarget(s, p, s->target_pending);
        } else {
            s->f_out = s->f_cur;
            return s->f_out;
        }
    }

    if (s->mode == HH_MODE_LAND) {
        if (s->t_mode == 0.0f && s->z_ref > z) s->z_ref = z;                               /* immediate descent from current height */
        if (s->landed) {
            s->f_out -= p->idle_ramp_per_s * dt; if (s->f_out < p->thr_idle) s->f_out = p->thr_idle;
            return s->f_out;
        }
        const float rate = z < p->land_slow_below ? p->land_slow_rate : p->land_rate;
        s->z_ref -= rate * dt; if (s->z_ref < -0.3f) s->z_ref = -0.3f;
        s->vz_ff = -rate;
        if (z < p->touch_m) s->t_touch += dt; else s->t_touch = 0.0f;
        if (s->t_touch >= p->touch_s) { s->landed = true; return s->f_out; }
    } else if (s->trans) {
        float u = clampf_((s->t - s->t_start) / s->t_dur, 0.0f, 1.0f);
        const float u3 = u * u * u, u4 = u3 * u, u5 = u4 * u, u2 = u * u;
        const float V = s->v_from * s->t_dur, D = (s->z_to - s->z_from) - V;                       /* v4.7: quintic with a start speed; V = 0 is the old 10/-15/6 curve */
        const float c3 = 10.0f * D + 4.0f * V, c4 = -15.0f * D - 7.0f * V, c5 = 6.0f * D + 3.0f * V;
        s->z_ref = s->z_from + V * u + c3 * u3 + c4 * u4 + c5 * u5;
        s->vz_ff = (V + 3.0f * c3 * u2 + 4.0f * c4 * u3 + 5.0f * c5 * u4) / s->t_dur;
        s->az_ff = (6.0f * c3 * u + 12.0f * c4 * u2 + 20.0f * c5 * u3) / (s->t_dur * s->t_dur);
        if (u >= 1.0f) { s->trans = false; s->vz_ff = 0.0f; s->az_ff = 0.0f; s->z_ref = s->z_to; }
    }
    const float e = s->z_ref - z;
    /* sank back onto the floor in the hold (handed over too early, or the throttle base was too low): go back to the take-off creep from the current throttle instead of holding on the floor forever */
    if (s->mode == HH_MODE_HOLD) {
        if (z < 0.04f && e > 0.15f && absf_(vz) < 0.10f) s->t_floor += dt; else s->t_floor = 0.0f;
        if (s->t_floor >= 1.5f) {
            s->mode = HH_MODE_TAKEOFF; s->t_mode = 0.0f; s->t_floor = 0.0f; s->t_cap = 0.0f; s->t_satur = 0.0f;
            s->lifted = false; s->base_learned = false; s->trans = false; s->vz_ff = 0.0f; s->integ = 0.0f;
            s->target_pending = clampf_(s->target, p->height_min, p->height_max);
            s->f_cur = s->f_out; s->f_start = s->f_out;
            return s->f_out;
        }
    }
    const float vz_ref = clampf_(p->kp_z * e + s->vz_ff, -p->vz_lim, p->vz_lim);
    /* thrust ~ V^2 * u^2  =>  u - idle scales with V0/V */
    float scale = s->v0 / vb;
    if (scale > 1.6f) scale = 1.6f;
    const float base = p->thr_idle + (s->f_base - p->thr_idle) * scale;
    const float f_unsat = base + p->kv_z * (vz_ref - vz) + s->integ;
    /* integrate only if not saturated against the error (anti wind-up); never on the floor */
    const bool sat_hi = f_unsat >= p->thr_max && e > 0.0f;
    const bool sat_lo = f_unsat <= base - p->drop_max && e < 0.0f;
    /* near the floor the integrator may only grow (so a drone that settled back on the floor can lift again), bounded by i_max */
    if ((z > 0.05f || e > 0.0f) && !sat_hi && !sat_lo) s->integ = clampf_(s->integ + p->ki_z * e * dt, -p->i_max, p->i_max);
    float f = base + p->kv_z * (vz_ref - vz) + s->integ;
    /* v4.7.1: acceleration feed-forward while a height change runs (thrust ~ u^2, so du = (f_base - idle) * a / 2g). Bench 2026-11-24: step overshoot 42 -> 21 mm */
    if (s->mode == HH_MODE_HOLD && s->trans) f += p->az_ff_gain * (s->f_base - p->thr_idle) * s->az_ff / (2.0f * 9.81f);
    if (s->mode == HH_MODE_HOLD) {
        if (z > 0.05f && (z > s->z_ref + p->overshoot_m || vz > p->overshoot_vz)) {
            /* v4.4: brake relative to what is actually holding the drone (base + integral), not to the bare base. On the real drone the base is the lift-off throttle in ground effect (~0.65)
             * and the integral carries the rest (~+0.09): capping at base - cut and zeroing the integral dropped the throttle by 0.12 right after the hand-over and the drone fell back to the floor
             * (flights 2026-11-16 15:03 and 15:02). The integral is kept; the brake is still overshoot_cut below the hover estimate. */
            const float cap = base + (s->integ > 0.0f ? s->integ : 0.0f) - p->overshoot_cut;
            if (f > cap) f = cap;
            s->overshoot = true;
        } else if (z < s->z_ref + p->overshoot_m * 0.5f && vz < p->overshoot_vz * 0.5f) {
            s->overshoot = false;
        }
    }
    f = clampf_(f, base - p->drop_max, p->thr_max);
    /* thrust limit: the throttle is pinned at the cap and the drone is not climbing (weak pack, heavy drone, something holding it): a stronger demand cannot help, so land gently */
    if (s->mode == HH_MODE_HOLD && p->cap_land_s > 0.0f) {
        if (f >= p->thr_max - 0.005f && vz < 0.10f) s->t_satur += dt; else { s->t_satur -= dt; if (s->t_satur < 0.0f) s->t_satur = 0.0f; }
        if (s->t_satur >= p->cap_land_s) { s->cap_landing = true; hhLand(s, p); }
    }
    s->f_out = f;
    return f;
}
