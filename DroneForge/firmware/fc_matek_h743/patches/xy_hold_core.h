#pragma once
/*
 * FC-side sideways hold (pure C, no Betaflight dependencies: unit-tested on the PC against a simulated plant).
 * Input: the drone's own speed (forward, right) from the EKF in m/s and the true height. Output: roll and pitch ANGLE offsets in degrees that are added to the ANGLE-mode target
 * (positive pitch = nose down = forward acceleration, positive roll = right: the Betaflight convention, see gps_rescue.c).
 * Law (Nimbus-like, lower gains): tilt = -(Kv * speed + Kp * displacement + Ki * integral of displacement), limited to max_deg. The integral learns a constant level error in flight.
 * Until its sign is known an axis only damps gently (nominal sign, +-1.5 deg). Before it is trusted, each axis tests its own sign: a small tilt is applied for a moment and the speed change is measured. A wrong sign is inverted, no response disables that axis.
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
} xyState_t;

void xyDefaults(xyParams_t *p);
void xyStart(xyState_t *s);
void xyStop(xyState_t *s);
/* one step. forward/right in m/s, z = true height in m, hold = position terms allowed (take-off climb and hold); outputs roll_deg and pitch_deg */
void xyUpdate(xyState_t *s, const xyParams_t *p, float dt, float forward, float right, float z, bool hold, float *roll_deg, float *pitch_deg);
