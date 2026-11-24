v4.3 = v4.2 (ALT/POS mode on AUX7) + capped floor integrator.
While the FC take-off creep runs and the drone is within 5 cm of the floor, the roll and pitch PID integrator is limited to +-100 instead of being zeroed (yaw stays zeroed). Disarmed / idle-armed / hold not active: unchanged (zeroed).
Why: left motors need about 0.12 of the motor range more than the right on every creep (9 creeps, 2026-11-14/16); with the integrator held at zero only P fights it and the drone leans on the floor (roll -3 deg, roll setpoint 22 deg/s before the 5 cm release; attempt 1 of 2026-11-16 leaned to -33 deg and was stopped, attempt 6 to -15 deg). In the clean creeps the integrator settled at 55-70 once released.
Tested: host test of the cap function only (16 tests pass). NOT tested on the real drone, and the benefit cannot be shown in the simulator (no roll dynamics).
If it behaves worse (roll hunting on the floor), go back to v4.1/v4.2.
NOT FLASHED.
