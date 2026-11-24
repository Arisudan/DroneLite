#!/bin/sh
# builds libhh.so from the firmware's pure-C control law (no Betaflight dependencies)
F=${FW_FLIGHT:-/home/htic/droneforge/bf_fixed/repo/src/main/flight}
cd "$(dirname "$0")" && gcc -O2 -shared -fPIC -I"$F" hh_py.c "$F/height_hold_core.c" "$F/xy_hold_core.c" "$F/estimator_gates.c" "$F/ground_hold_gate.c" -lm -o libhh.so && echo built
