#pragma once
/*
 * FC-side vertical flight: the pure control law (no Betaflight dependencies, so it is unit-tested on the PC against the flight simulator).
 * Modes: TAKEOFF (throttle creeps up from below hover until the drone is airborne), HOLD (height reference on a minimum-jerk curve with a velocity
 * feed-forward, outer loop kp_z on height, inner loop kv_z on vertical speed, a bounded integrator, an overshoot damper, a hover base that follows the
 * PC's live estimate), LAND (reference sinks at a limited rate, touchdown detection, throttle to idle).
 * All throttle numbers are STICK FRACTIONS 0..1 (the same unit as the PC's "hover throttle 0.70").
 */
#include <stdbool.h>

typedef struct {
    float kp_z, kv_z, ki_z, i_max;       /* 0.9923, 0.0538, 0.02, 0.12 */
    float vz_lim;                        /* 0.4581 m/s */
    float climb_peak_frac, descent_peak_frac;   /* 0.45, 0.45 of vz_lim for height changes */
    float thr_idle;                      /* 0.05 */
    float thr_max;                       /* absolute cap: 0.85 (real flights: lift-off at 0.78-0.80, free-air hover 0.69-0.75; the old 0.81 left no margin) */
    float drop_max;                      /* the output never goes more than this below the base throttle (0.30) */
    float overshoot_m, overshoot_vz, overshoot_cut;   /* 0.12 m, 0.45 m/s, 0.08 */
    float height_min, height_max;        /* 0.2 .. 2.5 m */
    /* appended (keep the order: the PC simulator mirrors this struct) */
    float base_tau;                      /* 1.5 s: time constant with which the hover base follows the PC's live hover estimate (only when it was NOT learned at lift-off and agrees within base_follow_max) */
    float base_follow_max;               /* 0.06: the PC estimate is ignored when it differs from the base by more than this (wrong voltage reading etc.) */
    float lift_vz;                       /* 0.05 m/s: rising faster than this near the floor = lifted off */
    float lift_m;                        /* 0.15 m: airborne above this height */
    float takeoff_start_frac;            /* 0.70: creep starts at this fraction of the hover throttle */
    float takeoff_creep_per_s;           /* 0.010 throttle fraction per second */
    float takeoff_cap_frac;              /* 1.15: creep never goes above this fraction of the hover throttle (and never above thr_max) */
    float takeoff_fail_s;                /* 3.0: at the cap without lifting for this long: fault */
    float land_rate, land_slow_rate, land_slow_below;   /* 0.12 m/s, 0.06 m/s below 0.30 m */
    float touch_m, touch_s;              /* 0.07 m for 0.4 s = touched down */
    float idle_ramp_per_s;               /* 1.0: throttle fraction per second ramped to idle after touchdown */
    float takeoff_ramp_per_s;            /* 0.35: throttle fraction per second from idle up to the creep start level (NEVER a step: Betaflight's throttle boost turns a throttle step into a full-throttle burst) */
    float cap_land_s;  /* appended */              /* 1.5: in HOLD with the throttle pinned at thr_max and the drone not climbing for this long = the thrust limit is reached: land gently instead of fighting (0 = off) */
    float lift_base_boost;               /* 0.04: added to the throttle that lifted the drone to get the hover base (the lift-off happens in ground effect, ~0.09 below the free-air hover on the real drone) */
    float takeoff_vz_max;                /* 0.30 m/s: during the take-off climb the creep stops above 0.6x this and backs off above it (a speed governor: the climb is open loop otherwise) */
    float takeoff_vz_back;               /* 0.05: throttle fraction per second removed while above takeoff_vz_max */
    float takeoff_handover_margin;       /* 0.05 m: the hold takes over this close below the target height (take-off CLIMBS to the target, then holds there) */
    float takeoff_fast_per_s;            /* 0.03 (v4.5 rev3): creep speed until the drone is 8 cm up (corrected); then takeoff_creep_per_s. The slow creep alone needed ~25 s to reach the lift-off throttle */
    float takeoff_closed;                /* v4.7: > 0.5 = the height hold takes over as soon as the lift-off is detected and the drone is above lift_m; the rest of the climb is the hold's smooth
                                            trajectory, starting at the drone's climb speed. 0 = v4.6: the open-loop creep climbs to 5 cm below the target (it overshot: real flights 0.6 -> 0.78 m,
                                            simulation of the drone 0.80 -> 1.08 m, past the 1.0 m ceiling) */
} hhParams_t;

enum { HH_MODE_HOLD = 0, HH_MODE_TAKEOFF = 1, HH_MODE_LAND = 2 };

typedef struct {
    bool  active;
    float z_ref, vz_ff;                  /* current height reference (m) and its feed-forward speed (m/s) */
    float az_ff;                         /* v4.7.1: the reference's acceleration (m/s^2) while a height change runs */
    float integ;
    float f_base, v0;                    /* hover throttle (at pack voltage v0) */
    float target;                        /* where the reference is heading (m) */
    bool  trans;
    float t_start, t_dur, z_from, z_to;  /* minimum-jerk transition */
    float v_from;                        /* v4.7: speed at the start of the transition (m/s; 0 = from rest, the old minimum-jerk curve) */
    float t;                             /* own clock (s) */
    float f_out;                         /* last output */
    bool  overshoot;
    /* appended */
    int   mode;
    float f_cur;                         /* takeoff creep throttle */
    float t_mode;                        /* time in the current mode (s) */
    float t_cap;                         /* time spent at the creep cap (s) */
    float target_pending;                /* height requested while still on the ground */
    float t_touch;                       /* time below touch_m while landing (s) */
    bool  fail;                          /* takeoff did not lift */
    bool  landed;                        /* touched down, throttle at idle */
    float f_lift;                        /* throttle at which the drone physically started to rise (learned during the take-off creep) */
    bool  lifted;
    bool  base_learned;                  /* the hover base came from the physical lift-off: the PC's estimate is not followed any more */
    float f_start;                       /* creep start level (reached by the ramp) */
    float t_satur;                       /* time with the throttle at the cap and no climb (s) */
    float t_liftdet;                     /* time the rise has lasted (s) */
    float t_floor;                       /* time in the hold on the floor with the target above (s): sank back */
    bool  cap_landing;                   /* the current landing was started by the thrust-limit rule */
} hhState_t;

/* the MTF-02P over-reads by ~9 cm between ~20 and ~40 cm (measured; the PC controller corrects it with the same curve): height above the learned ground as READ -> true height above the ground */
float hhCorrectHeight(float z_read);
void  hhDefaults(hhParams_t *p);
/* start holding: z = current height (m), f0 = hover throttle fraction (base), vbat in V */
void  hhEngage(hhState_t *s, const hhParams_t *p, float z, float f0, float vbat);
void  hhDisengage(hhState_t *s);
/* start a take-off from the floor: f_hover = the PC's hover reference, target = height to hold after lift-off */
void  hhTakeoffBegin(hhState_t *s, const hhParams_t *p, float f_hover, float target, float vbat);
/* new target height (m), clamped to height_min..height_max; the reference glides there (during take-off it is remembered for the lift-off) */
void  hhSetTarget(hhState_t *s, const hhParams_t *p, float h);
/* land from where it is */
void  hhLand(hhState_t *s, const hhParams_t *p);
/* the PC's live hover estimate (stick fraction at pack voltage vbat): the base follows it slowly while hovering */
void  hhFollowBase(hhState_t *s, const hhParams_t *p, float dt, float f_hover, float vbat);
/* one control step: dt (s), height z (m), vertical speed vz (m/s), pack voltage; returns the throttle fraction */
float hhUpdate(hhState_t *s, const hhParams_t *p, float dt, float z, float vz, float vbat);
