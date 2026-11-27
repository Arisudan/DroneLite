#pragma once
/* Pure C ground-hold gate (no Betaflight includes): unit-tested on the PC. Nimbus-style: low-passed height, release dwell, latched release. */
#include <stdbool.h>

#define GH_RELEASE_M 0.05f        /* filtered height above this ... */
#define GH_RELEASE_DWELL_S 0.15f  /* ... continuously for this long releases the hold */
#define GH_REENGAGE_M 0.02f       /* after release: filtered height below this ... */
#define GH_REENGAGE_DWELL_S 1.0f  /* ... continuously for this long re-engages the hold (disarm always does) */
#define GH_FILTER_TAU_S 0.10f     /* first-order low-pass time constant of the height input */

typedef struct {
    bool inited;
    bool held;
    float filt;
    float timer;
} groundHoldGate_t;

void ghInit(groundHoldGate_t *g);                                 /* disarmed state: held, filter empty */
bool ghUpdate(groundHoldGate_t *g, float height_m, float dt_s, bool armed);  /* returns true while the hold (integrator at zero) is active */

/* v4.3: while the FC creeps up from the floor (hold active, drone within 5 cm of the floor) the roll and pitch integrator is no longer held at zero but limited to +-cap (PID units):
 * the P term alone cannot carry the left-right thrust difference (about 0.12 of the motor range on this drone) and the drone leans before it clears the floor. A cap of 100 covers the
 * ~55-70 the integrator settled at in the clean creeps; the 2026-11-05 wind-up (hundreds of units, one motor saturated) cannot happen. cap <= 0 means "hold at zero" (the old behaviour). */
float ghIntegratorCap(float i, float cap);
