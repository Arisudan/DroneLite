"""Topic names (mirroring the Nimbus message groups; payloads are plain Python objects, decoded dicts or small dataclasses)."""
FRAME_IN = "frame_in"                 # nimbus_pc.crsf.Frame, every CRSF frame received (high rate)
FRAME_OUT = "frame_out"               # bytes, every frame written to the dongle (high rate)
TELEMETRY = "telemetry"               # (name, dict) decoded telemetry: link, battery, attitude, optrange, flight_mode, ...
CONTROLS = "controls"                 # nimbus_pc.sender.Controls currently sent to the drone (published at the control tick)
CONNECTION = "nimbus_connection"      # {"connected": bool, "port": str}
EVENT = "event"                       # str, human-readable event log line
ARM = "arm"                           # {"channel_high": bool}
FAILSAFE = "failsafe"                 # {"active": bool}
WATCHDOG = "optrange_guard_status"    # {"state": str}
SOURCE = "control_source"             # {"source": str, "allow_arm": bool}
SAFETY = "safety_status"            # dict from SafetyMonitor.status() + {"event": SafetyEvent-or-None}, published each control tick and on every event
LINK_STATE = "link_state"            # {"state": offline|connecting|connected|degraded|lost, "prev": str, "t": float}, only on change
PREFLIGHT = "preflight"              # list[director.Check], only when a check flips
NOTIFY = "notify"                    # notifications.Notice, every new notice
