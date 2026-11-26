#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

/* FC-side height hold (see height_hold_core.h). Switch + target come from ONE RC channel (CH9 = AUX5):
 *   below 1100 us          hold OFF  (the throttle is the pilot's / the PC's, as always)
 *   1100..1149 us          hold ON at the current height
 *   1150..1950 us          hold ON at (us - 1150) * 2.5 / 800 metres (0..2.5 m; the core limits it to 0.2..2.5 m)
 *   1960..2000 us          LAND (descend at <=0.12 m/s, touch down, throttle to idle; the PC then disarms)
 *   FC take-off: while still on the floor, a target >= 1150 us plus a valid hover reference on CH10 starts a creep (level, throttle stick at minimum)
 * Mode activation: BOXUSER1 ("USER1") on AUX5 above 1100 us:  aux <n> 40 4 8 48 0 0
 * Height = EKF height (range sensor) minus the sensor's ground offset learned while armed on the ground. */
/* FC-side sideways hold (flight/xy_hold_core.c): roll and pitch ANGLE offsets in degrees, added to the ANGLE-mode target in pid.c (index 0 roll, 1 pitch); zero when not active */
extern float xyHoldAngle[2];
/* true while the drone is on the floor (disarmed, or armed within 3 cm of the learned ground): the EKF holds its speed at zero there (zero-velocity update) */
bool heightHoldFloorContact(void);
/* the sideways loop switched itself off for this flight (runaway guard) */
bool heightHoldXyDisabled(void);
void heightHoldInit(void);
void heightHoldUpdate(timeUs_t currentTimeUs);   /* called at the range-sensor rate (50 Hz), right after the range sample was fused into the EKF */
bool heightHoldIsActive(void);
bool heightHoldHasFault(void);
int heightHoldMode(void);                        /* -1 off, 0 hold, 1 take-off, 2 landing */
bool heightHoldOnGround(void);                   /* armed on the floor: the PID integrator must stay at zero */
int16_t heightHoldRcCommandThrottle(void);       /* what rcCommand[THROTTLE] would be for the hold's throttle */
void heightHoldSetPosAid(int16_t fwdMm, int16_t rightMm, bool valid);   /* v4.7: the PC's visual-odometry position aid (MSP2_DF_POS_AID); used while younger than 0.3 s */
