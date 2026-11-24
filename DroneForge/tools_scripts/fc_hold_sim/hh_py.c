/* Flat C API around firmware height_hold_core.c for the PC simulation (ctypes). Not part of the firmware. */
#include <string.h>
#include "height_hold_core.h"
#include "xy_hold_core.h"
#include "ground_hold_gate.h"
#include "estimator_gates.h"
static hhParams_t P; static hhState_t S;
void hh_defaults(void) { hhDefaults(&P); memset(&S, 0, sizeof(S)); }
float *hh_params(void) { return (float *)&P; }
int hh_nparams(void) { return (int)(sizeof(P) / sizeof(float)); }
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

/* sideways loop */
static xyParams_t XP; static xyState_t XS;
void xy_reposition(void) { xyReposition(&XS); }
void xy_defaults(void) { xyDefaults(&XP); xyStart(&XS); }
float *xy_params(void) { return (float *)&XP; }
void xy_update(float dt, float fwd, float right, float z, int hold, float *out) { float r, p; xyUpdate(&XS, &XP, dt, fwd, right, z, hold != 0, &r, &p); out[0] = r; out[1] = p; }
void xy_update_st(float dt, float fwd, float right, float z, int hold, float sp, float sr, int aid, float af, float ar, float *out) { float r, p; xyUpdateST(&XS, &XP, dt, fwd, right, z, hold != 0, sp, sr, aid != 0, af, ar, &r, &p); out[0] = r; out[1] = p; }   /* v4.7 ST */
int xy_braking(void) { return XS.braking; }
void xy_brake(void) { xyBrakeThenHold(&XS); }
int xy_aid_used(void) { return XS.aid_used; }
float xy_i(int ax) { return XS.i[ax]; }
int xy_disabled(void) { return XS.disabled; }
int xy_phase(void) { return XS.phase; }
float xy_sign(int ax) { return XS.sign[ax]; }
int xy_axis_off(int ax) { return XS.axis_off[ax]; }
float xy_p(int ax) { return XS.p[ax]; }

/* frozen-range detector */
static egFlatline_t FL;
void fl_init(void) { egFlatlineInit(&FL); }
int fl_update(float dt, unsigned int sig) { return egFlatlineUpdate(&FL, dt, sig); }
unsigned int fl_sig(int d, unsigned int st, unsigned int pr, int fx, int fy, unsigned int q) { return egPayloadSignature(d, st, pr, fx, fy, q); }

float gh_cap(float i, float cap) { return ghIntegratorCap(i, cap); }
