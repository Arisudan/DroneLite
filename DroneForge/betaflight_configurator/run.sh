#!/bin/bash
# Start the configurator server and open the UI. ./run.sh --demo uses the fake FC.
cd "$(dirname "$0")" || exit 1
python3 -m server "$@" &
SRV=$!
trap 'kill $SRV 2>/dev/null' EXIT INT TERM
for i in $(seq 1 50); do
  (exec 3<>/dev/tcp/127.0.0.1/8765) 2>/dev/null && break
  sleep 0.1
done
URL=http://127.0.0.1:8765
START=$SECONDS
if [ -x /snap/bin/chromium ]; then
  /snap/bin/chromium --app=$URL >/dev/null 2>&1
else
  firefox $URL >/dev/null 2>&1
fi
# If the browser returned at once (it handed over to an already running instance), keep serving until Ctrl+C.
if [ $((SECONDS - START)) -lt 3 ] && kill -0 $SRV 2>/dev/null; then wait $SRV; fi
