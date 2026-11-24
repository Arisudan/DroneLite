/* Flat C API around the FLASHED v3 height_hold_core.c (source snapshot 2026-11-13_hybrid-v3) for the PC simulation. Not part of the firmware. */
#include <string.h>
#include "height_hold_core.h"
static hhParams_t P; static hhState_t S;
void hh_defaults(void) { hhDefaults(&P); memset(&S, 0, sizeof(S)); }
float *hh_params(void) { return (float *)&P; }
void hh_engage(float z, float f0, float vbat) { hhEngage(&S, &P, z, f0, vbat); }
void hh_takeoff(float f_hover, float target, float vbat) { hhTakeoffBegin(&S, &P, f_hover, target, vbat); }
void hh_set_target(float h) { hhSetTarget(&S, &P, h); }
void hh_land(void) { hhLand(&S, &P); }
void hh_follow(float dt, float fh, float vbat) { hhFollowBase(&S, &P, dt, fh, vbat); }
float hh_update(float dt, float z, float vz, float vbat) { return hhUpdate(&S, &P, dt, z, vz, vbat); }
int hh_mode(void) { return S.mode; }
int hh_active(void) { return S.active; }
int hh_fail(void) { return S.fail; }
int hh_landed(void) { return S.landed; }
float hh_integ(void) { return S.integ; }
float hh_base(void) { return S.f_base; }
float hh_zref(void) { return S.z_ref; }
int hh_cap_landing(void) { return S.cap_landing; }
