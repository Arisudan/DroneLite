#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_EKF

#include "common/maths.h"

#include "drivers/time.h"

#include "fc/rc.h"
#include "flight/imu.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/height_hold.h"
#include "flight/height_hold_core.h"
#include "flight/xy_hold_core.h"
#include "flight/ground_hold_gate.h"
#include "flight/estimator_gates.h"
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#include "flight/kinematic_estimator.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#define HH_RC_CHANNEL        8       /* rcData[8] = CH9 = AUX5: switch + target height */
#define HH_RC_MODE_CHANNEL   10      /* rcData[10] = CH11 = AUX7: flight mode. 1100..1399 us = ALT HOLD (sideways loop paused); anything else, including minimum (988), the 1500 us idle value of an unset channel and no signal, = POS HOLD (v4.2) */
#define HH_US_ALT_LO         1100
#define HH_US_ALT_HI         1400
#define HH_RC_BASE_CHANNEL   9       /* rcData[9] = CH10: the PC's hover-reference throttle (same scale as the throttle channel); below 1100 us = not provided */
#define HH_US_OFF            1100
#define HH_US_TARGET_START   1150
#define HH_US_LAND           1960    /* CH9 at or above this = LAND (FC lands by itself) */
#define HH_MAX_TILT_DECI     350     /* take-off aborts above 35 deg of roll/pitch */
#define HH_TAKEOFF_MAX_THR_US 1300   /* the PC sends zero throttle while it asks for an FC take-off */
#define HH_MAX_RANGE_AGE_US  250000  /* the range sample must be fresher than this, or the hold lets go */
#define HH_MIN_AIRBORNE_M    0.15f
#define HH_GROUND_LEARN_US   1300    /* the ground offset is learned while armed with the throttle stick below this (still on the floor) */
#define HH_GROUND_HOLD_M     0.05f   /* the attitude integrator is held at zero while the drone is within this height of the floor */
#define HH_CEILING_M         2.8f    /* above this the hold lets go (a fault): the PC lands */

float xyHoldAngle[2] = { 0.0f, 0.0f };
static xyParams_t xyParams;
static xyState_t xyState;
static bool xyStarted = false;
static bool xyPaused = false;     /* ALT HOLD: the sideways loop is paused (AUX7) */
static hhParams_t hhParams;
static hhState_t hhState;
static bool hhFault = false;
static bool hhLandLatch = false;   /* landed: no new take-off until the land command / the hold request goes away */
static int hhLastMode = -1;
static int16_t hhRcThrottle = 1000;
static float groundZ = 0.0f;
static uint16_t groundN = 0;
static bool wasArmed = false;
static timeUs_t lastRunUs = 0;
static egFlatline_t flatline;
static bool flatlineInit = false;
static groundHoldGate_t ghGate;
static bool ghGateInit = false;
static timeUs_t ghLastUs = 0;
#define HH_AID_MAX_AGE_US    300000  /* v4.7: the PC position aid counts as fresh this long */
static float aidFwd = 0.0f, aidRight = 0.0f;
static bool aidValid = false;
static timeUs_t aidUs = 0;

/* v4.7.2: the same aid on RC channels (2026-11-24): MSP over ELRS delivered ~1 write in 10 (bench), so the aid was almost always older than HH_AID_MAX_AGE_US.
 * RC channels ride in every ELRS packet. Needs a full-resolution ELRS packet rate ("100Hz Full": 10-bit, ~1 us); the switch modes (Wide/Hybrid) send CH13+ not at all
 * and CH6-12 in 15 us steps. CH13 = forward, CH14 = right: 1500 us = 0, 1 us = 1 mm, valid 1100..1900 us (+-0.40 m); CH15 > 1900 us = the PC says the aid is valid.
 * An unset channel (1500 us or the minimum) never counts as an aid. Ignored without an RC signal and in failsafe (failsafe HOLDS the AUX channels at their last value). */
#define HH_RC_AID_FWD_CH     12      /* rcData[12] = CH13 = AUX9  */
#define HH_RC_AID_RIGHT_CH   13      /* rcData[13] = CH14 = AUX10 */
#define HH_RC_AID_FLAG_CH    14      /* rcData[14] = CH15 = AUX11 */
#define HH_RC_AID_FLAG_US    1900.0f
#define HH_RC_AID_MIN_US     1100.0f
#define HH_RC_AID_MAX_US     1900.0f

static void hhReadRcAid(timeUs_t now)
{
    if (!rxIsReceivingSignal() || FLIGHT_MODE(FAILSAFE_MODE) || rcData[HH_RC_AID_FLAG_CH] <= HH_RC_AID_FLAG_US) return;
    const float f = rcData[HH_RC_AID_FWD_CH], r = rcData[HH_RC_AID_RIGHT_CH];
    if (f < HH_RC_AID_MIN_US || f > HH_RC_AID_MAX_US || r < HH_RC_AID_MIN_US || r > HH_RC_AID_MAX_US) return;
    aidFwd = (f - 1500.0f) * 0.001f; aidRight = (r - 1500.0f) * 0.001f; aidValid = true; aidUs = now;
}

void heightHoldSetPosAid(int16_t fwdMm, int16_t rightMm, bool valid)
{
    aidFwd = fwdMm * 0.001f; aidRight = rightMm * 0.001f; aidValid = valid; aidUs = micros();
}

void heightHoldInit(void)
{
    hhDefaults(&hhParams);
    xyDefaults(&xyParams);
    hhState.active = false;
    hhFault = false;
    groundN = 0;
}

bool heightHoldIsActive(void) { return hhState.active; }

/* true while armed ON THE FLOOR (ground offset known, range fresh, height above the floor below HH_GROUND_HOLD_M): the PID integrator is then held at zero.
 * Why: the drone cannot rotate on the floor, so any small tilt error (0.3 deg) integrates to hundreds of motor microseconds within seconds (anti-gravity boosts it while the
 * throttle rises), saturates one motor and the mixer cuts the common throttle: no lift-off (measured 2026-11-05, ramps 14:01..15:47). */
bool heightHoldOnGround(void)
{
    if (!ghGateInit) { ghInit(&ghGate); ghGateInit = true; }
    if (!ARMING_FLAG(ARMED)) { ghUpdate(&ghGate, 0.0f, 0.0f, false); ghLastUs = 0; return false; }
    if (groundN < 20) return false;
    const kinematicState_t *st = kinematicEstimatorGetState();
    const timeUs_t nowUs = micros();
    if (!isfinite(st->posZ) || kinematicEstimatorRangeAgeUs(nowUs) > HH_MAX_RANGE_AGE_US) { ghLastUs = 0; return false; }
    const float dt = ghLastUs ? (float)cmpTimeUs(nowUs, ghLastUs) * 1e-6f : 0.0f;
    ghLastUs = nowUs;
    /* low-passed height, release after 0.15 s above HH_GROUND_HOLD_M, then latched until 1 s below 0.02 m or disarm (range flicker at a few cm no longer toggles the integrator) */
    return ghUpdate(&ghGate, st->posZ - groundZ, dt, true);
}
bool heightHoldHasFault(void) { return hhFault; }
bool heightHoldXyDisabled(void) { return xyStarted && xyState.disabled; }

/* floor contact for the EKF zero-velocity update: disarmed (the drone sits still), or armed on the floor within 3 cm of the learned ground */
bool heightHoldFloorContact(void)
{
    if (!ARMING_FLAG(ARMED)) return true;
    if (groundN < 20) return false;
    const kinematicState_t *st = kinematicEstimatorGetState();
    if (!isfinite(st->posZ)) return false;
    return (st->posZ - groundZ) < 0.03f;
}
int heightHoldMode(void) { return hhState.active ? hhState.mode : -1; }
int16_t heightHoldRcCommandThrottle(void) { return hhRcThrottle; }

static float throttleFractionFromUs(float us)    /* the PC's throttle01 -> CRSF mapping, inverted: us = 5*ticks/8 + 880, ticks = 172 + f*1639 */
{
    const float ticks = (us - 880.0f) * 8.0f / 5.0f;
    return constrainf((ticks - 172.0f) / 1639.0f, 0.0f, 1.0f);
}

static int16_t rcCommandFromFraction(float f)    /* fraction -> channel us -> exactly what updateRcCommands() would make of it */
{
    const float ticks = 172.0f + f * 1639.0f;
    return rcCommandThrottleFromUs(5.0f * ticks / 8.0f + 880.0f);
}

static void release(bool fault)
{
    xyHoldAngle[0] = xyHoldAngle[1] = 0.0f; xyStarted = false; xyPaused = false;      /* the sideways hold ends with the height hold */
    hhDisengage(&hhState);
    if (fault) hhFault = true;
}

void heightHoldUpdate(timeUs_t currentTimeUs)
{
    const bool armed = ARMING_FLAG(ARMED);
    if (!armed) { xyHoldAngle[0] = xyHoldAngle[1] = 0.0f; xyStarted = false; xyPaused = false; }
    if (armed && !wasArmed) { groundN = 0; groundZ = 0.0f; hhFault = false; hhLastMode = -1; }
    wasArmed = armed;

    const kinematicState_t *st = kinematicEstimatorGetState();
    const float z = st->posZ, vz = st->velZ;
    const bool finite = isfinite(z) && isfinite(vz) && fabsf(z) < 20.0f && fabsf(vz) < 20.0f;
    const bool rangeFresh = kinematicEstimatorRangeAgeUs(currentTimeUs) < HH_MAX_RANGE_AGE_US;
    const uint16_t chUs = rcData[HH_RC_CHANNEL];
    const bool requested = IS_RC_MODE_ACTIVE(BOXUSER1) && chUs >= HH_US_OFF;

    const bool landCmd = requested && chUs >= HH_US_LAND;
    if (!requested || !armed || !landCmd) hhLandLatch = hhLandLatch && requested && armed && landCmd;
    if (!requested) { hhFault = false; if (hhState.active) release(false); }

    /* learn the ground offset while armed on the floor (never while holding) */
    if (armed && !hhState.active && finite && rangeFresh && z < 0.5f && rcData[THROTTLE] < HH_GROUND_LEARN_US && fabsf(vz) < 0.15f) {
        if (groundN < 200) groundN++;
        groundZ += (z - groundZ) / (float)groundN;
    }

    const float dt = lastRunUs ? (float)cmpTimeUs(currentTimeUs, lastRunUs) * 1e-6f : 0.02f;
    lastRunUs = currentTimeUs;

    if (!hhState.active) {
        if (!(requested && !hhFault && !hhLandLatch && !landCmd && armed && !FLIGHT_MODE(FAILSAFE_MODE) && finite && rangeFresh && groundN >= 20)) return;
        const float zr = z - groundZ;
        const bool hoverRefOk = rcData[HH_RC_BASE_CHANNEL] >= HH_US_OFF && throttleFractionFromUs((float)rcData[HH_RC_BASE_CHANNEL]) > 0.2f && throttleFractionFromUs((float)rcData[HH_RC_BASE_CHANNEL]) < 0.9f;
        if (zr < HH_MIN_AIRBORNE_M) {
            /* on the floor with a real target (>= 1150 us) and a valid hover reference: FC take-off. Level, still, stick at minimum. */
            if (chUs < HH_US_TARGET_START || !hoverRefOk || rcData[THROTTLE] >= HH_TAKEOFF_MAX_THR_US || fabsf(vz) > 0.15f) return;
            if (abs(attitude.values.roll) > 100 || abs(attitude.values.pitch) > 100) return;      /* must be level (10 deg) to start */
            const float b = throttleFractionFromUs((float)rcData[HH_RC_BASE_CHANNEL]);
            hhTakeoffBegin(&hhState, &hhParams, b, ((float)chUs - (float)HH_US_TARGET_START) * 2.5f / 800.0f, getBatteryVoltageLatest() * 0.01f);
            hhRcThrottle = rcCommandFromFraction(hhState.f_cur);
        } else {
            if (fabsf(vz) > 0.5f) return;                                          /* already airborne (PC lifted off): wait until settled */
            float base = throttleFractionFromUs((float)rcData[THROTTLE]);          /* fallback: the throttle the PC is sending now */
            if (hoverRefOk) base = throttleFractionFromUs((float)rcData[HH_RC_BASE_CHANNEL]);
            hhEngage(&hhState, &hhParams, zr, base, getBatteryVoltageLatest() * 0.01f);
            hhRcThrottle = rcCommand[THROTTLE];
        }
    }

    /* active: let go on anything suspicious (the PC then takes the throttle back and lands) */
    const float zr = z - groundZ;
    /* frozen range payload while flying (Nimbus 'flatline'): the whole sample unchanged for EG_FLATLINE_S */
    if (!flatlineInit) { egFlatlineInit(&flatline); flatlineInit = true; }
    if (zr > 0.05f) {
        const uint32_t sig = egPayloadSignature(rangefinderGetLatestRawAltitude(), rangefinderGetLatestDistStrength(), rangefinderGetLatestDistPrecision(),
                                                opticalflowGetLatestVelX(), opticalflowGetLatestVelY(), opticalflowGetLatestFlowQuality());
        if (egFlatlineUpdate(&flatline, constrainf(dt, 0.005f, 0.1f), sig)) { egFlatlineInit(&flatline); release(true); return; }
    } else {
        egFlatlineInit(&flatline);
    }
    if (!armed || FLIGHT_MODE(FAILSAFE_MODE) || !finite || !rangeFresh || zr > HH_CEILING_M || zr < -0.2f) {
        release(true);
        return;
    }
    if (hhState.mode == HH_MODE_TAKEOFF && (abs(attitude.values.roll) > HH_MAX_TILT_DECI || abs(attitude.values.pitch) > HH_MAX_TILT_DECI)) { release(true); return; }
    if (landCmd) {
        if (hhState.mode != HH_MODE_LAND) {
            hhLand(&hhState, &hhParams);
            if (hhState.z_ref > zr) hhState.z_ref = zr;              /* immediate descent from current height */
        }
    } else if (chUs >= HH_US_TARGET_START) hhSetTarget(&hhState, &hhParams, ((float)chUs - (float)HH_US_TARGET_START) * 2.5f / 800.0f);
    const float dtc = constrainf(dt, 0.005f, 0.1f);
    const float vbat = getBatteryVoltageLatest() * 0.01f;
    if (rcData[HH_RC_BASE_CHANNEL] >= HH_US_OFF && fabsf(vz) < 0.3f) hhFollowBase(&hhState, &hhParams, dtc, throttleFractionFromUs((float)rcData[HH_RC_BASE_CHANNEL]), vbat);
    const float f = hhUpdate(&hhState, &hhParams, dtc, zr, vz, vbat);
    if (hhState.fail) { release(true); return; }                              /* take-off never lifted: the PC takes over and disarms */
    if (hhState.landed && f <= hhParams.thr_idle + 0.001f) { release(false); hhLandLatch = true; return; }
    hhRcThrottle = rcCommandFromFraction(f);

    /* sideways hold: the drone's own speed from the EKF, the true height, angle offsets for pid.c */
    float fwd = 0.0f, rgt = 0.0f, rollDeg = 0.0f, pitchDeg = 0.0f;
    if (!xyStarted) { xyStart(&xyState); xyStarted = true; xyPaused = false; }
    const uint16_t modeUs = rcData[HH_RC_MODE_CHANNEL];
    const bool altOnly = modeUs >= HH_US_ALT_LO && modeUs < HH_US_ALT_HI;
    if (altOnly) {
        xyPaused = true;                                                       /* ALT HOLD: the height hold runs, the sideways loop gives no tilt */
    } else if (xyPaused) {
        xyReposition(&xyState); xyPaused = false;                              /* back to POS HOLD: stay where it is now (v4.7.3R: as v4.6-rev1, no brake first) */
    }
    if (hhLastMode != HH_MODE_HOLD && hhState.mode == HH_MODE_HOLD) {
        xyReposition(&xyState);                                                /* handover anchor: reset position displacement to zero at hover location (v4.7.3R: as v4.6-rev1, no brake first) */
    }
    hhLastMode = hhState.mode;
    if (!xyPaused && kinematicEstimatorGetBodyVelocity(&fwd, &rgt) && hhState.active) {
        /* v4.7 ST: the RC sticks (rcCommand +-500: roll right +, pitch forward +) and the PC's position aid while fresh (v4.7.2: from RC channels or MSP) */
        hhReadRcAid(currentTimeUs);
        const bool aidFresh = aidValid && aidUs && cmpTimeUs(currentTimeUs, aidUs) < HH_AID_MAX_AGE_US;
        xyUpdateST(&xyState, &xyParams, dtc, fwd, rgt, hhCorrectHeight(zr), hhState.mode == HH_MODE_HOLD, rcCommand[PITCH] / 500.0f, rcCommand[ROLL] / 500.0f,
                   aidFresh, aidFwd, aidRight, &rollDeg, &pitchDeg);
    }
    xyHoldAngle[0] = rollDeg; xyHoldAngle[1] = pitchDeg;
}

#endif
