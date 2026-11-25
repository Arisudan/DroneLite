#pragma once
/*
 * FC-side sideways hold (pure C, no Betaflight dependencies: unit-tested on the PC against a simulated plant).
 * Input: the drone's own speed (forward, right) from the EKF in m/s and the true height. Output: roll and pitch ANGLE offsets in degrees that are added to the ANGLE-mode target
 * (positive pitch = nose down = forward acceleration, positive roll = right: the Betaflight convention, see gps_rescue.c).
 * Law (Nimbus-like, lower gains): tilt = -(Kv * speed + Kp * displacement + Ki * integral of displacement), limited to max_deg. The integral learns a constant level error in flight.
 * Until its sign is known an axis only damps speed with the nominal sign (limited to max_deg; v4.4, before: +-1.5 deg). Before it is trusted, each axis tests its own sign: a small tilt is applied for a moment and the speed change is measured. A wrong sign is inverted, no response disables that axis.
 * A runaway guard switches the loop off for the flight if speed or distance keep growing.
 */
#include <stdbool.h>

typedef struct {
    float kv, kp, ki;               /* 5.0 deg/(m/s), 2.5 deg/m, 0.10 deg/(m*s) */
    float max_deg;                  /* 4.0 */
    float min_z;                    /* 0.06 m: no action below this height */
    float test_z;                   /* 0.08 m: the sign self-test needs this height */
    float test_deg, test_s;         /* 2.5 deg for 0.8 s */
    float test_min_dv;              /* 0.08 m/s: smaller response = no usable response */
    float calm_speed, calm_s;       /* 0.50 m/s for 0.5 s before the test starts (a drift against a level error counts as calm) */
    float runaway_speed, runaway_s, runaway_pos;   /* 0.8 m/s for 1.0 s, or 1.5 m */
    float skip_test;
    /* v4.7 ST (appended: older fields keep their order for the PC tools) */
    float stick_db;                 /* 0.05 (fraction of full stick): above this the pilot/PC flies the axis in pure ANGLE mode, the hold adds nothing */
    float brake_speed, brake_s;     /* 0.15 m/s for 0.2 s after the sticks centre: then the hold re-anchors at that spot (speed damping only while braking) */
    float aid_max_m;                /* 3.0 m: a position aid (the PC's visual odometry) larger than this is ignored */
    float climb_trim;               /* 3.0 deg/m: during the take-off climb (no position hold yet) the level trim is learned from the drift speed: trim += climb_trim * speed * dt,
                                       so the hold starts with the level error already compensated (v4.6-rev1: the first ~0.4 m of drift after the hand-over came from it) */                /* v4.5 rev2: > 0.5 = no sign self-test, the nominal sign is used from the start. The test leans the drone 2.5 deg on both axes for 0.8 s (a visible move forward and right at take-off) and the nominal sign has been confirmed on the real drone in many flights; the runaway guard still switches the loop off if it were wrong */
} xyParams_t;

typedef struct {
    bool  active;
    bool  disabled;                 /* runaway: off for the flight */
    float p[2], i[2];               /* displacement and its integral: [0] forward, [1] right */
    float sign[2];                  /* +1 / -1 measured sign of (speed change per tilt), 0 = not tested yet */
    bool  axis_off[2];              /* no usable response on this axis */
    int   phase;                    /* 0 waiting for calm, 1 sign test (both axes), 5 running */
    float t_phase, t_calm, t_run;
    float vf[2];                    /* low-passed speed */
    float v_start[2], a_pre[2];     /* speed at the start of a test and the drift acceleration measured just before it */
    float v_hist[2], t_hist;        /* speed 0.6 s ago for the pre-test drift */
    float out[2];                   /* last output: roll, pitch in degrees */
    bool  stick_on, braking;        /* v4.7 ST: a stick is deflected / braking after its release */
    float t_brake;
    bool  aid_used;                 /* v4.7: the last update used the PC position aid */
} xyState_t;

void xyDefaults(xyParams_t *p);
void xyStart(xyState_t *s);
void xyStop(xyState_t *s);
void xyReposition(xyState_t *s);   /* v4.7: clear the displacement only: the integral (the learned level trim) is kept; xyStart clears it (new flight) */
void xyBrakeThenHold(xyState_t *s); /* v4.7: damp the speed first, then re-anchor where the drone has stopped (hand-over and stick release) */
/* one step. forward/right in m/s, z = true height in m, hold = position terms allowed (take-off climb and hold); outputs roll_deg and pitch_deg */
void xyUpdate(xyState_t *s, const xyParams_t *p, float dt, float forward, float right, float z, bool hold, float *roll_deg, float *pitch_deg);
/* v4.7 ST: the same step with the RC sticks (pitch forward +, roll right +, -1..1) and an optional position aid (aid_valid: the drone's displacement from its hold point
 * in its own axes, metres: forward, right; from the PC's visual odometry). xyUpdate() = this with centred sticks and no aid. */
void xyUpdateST(xyState_t *s, const xyParams_t *p, float dt, float forward, float right, float z, bool hold, float stick_pitch, float stick_roll,
                bool aid_valid, float aid_fwd, float aid_right, float *roll_deg, float *pitch_deg);
