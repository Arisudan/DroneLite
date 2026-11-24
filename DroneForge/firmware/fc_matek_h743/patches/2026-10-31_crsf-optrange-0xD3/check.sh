#!/usr/bin/env bash
# Self-contained check: pulls the frame builder out of the patch, compiles it against mocks (host gcc),
# and verifies the frames it produces with the nimbus_pc Python decoder.
set -euo pipefail
cd "$(dirname "$0")"
awk '/^\+\/\/ BEGIN NIMBUS-OPTRANGE/{f=1} f{line=$0; sub(/^\+/,"",line); print line} /^\+\/\/ END NIMBUS-OPTRANGE/{f=0}' 0001-crsf-add-optrange-frame-0xD3.patch > optrange_fn.inc
gcc -Wall -Wextra -O2 -o harness harness.c -lm
python3 crosscheck.py
rm -f harness optrange_fn.inc
