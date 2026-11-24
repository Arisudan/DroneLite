"""The full rviz2 in its own window ('Open RViz' on the map views): DRONE_GCS's RTAB-Map layout (gazebo/config/rtabmap_gcs.rviz, default RViz colours), 3D orbit
view, Displays and Views panels. The map INSIDE the GCS is rviz_map_view.RvizMapView: on this desktop (COSMIC, Wayland) a re-parented X11 window is not composed
inside the GCS, so the embedded rviz2 pane (RvizPane) was removed on 2026-11-23 (archive/gcs_removed_2026-11-23/rviz_embed.py)."""
from __future__ import annotations

import os
import subprocess

DF = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONFIG = os.path.join(DF, "gazebo", "config", "rtabmap_gcs.rviz")
FASTDDS = os.path.join(DF, "gazebo", "config", "fastdds_shm.xml")
ROS_SETUP = "/opt/ros/jazzy/setup.bash"


_RVIZ_WINDOW = None


def launch_rviz_window(config: str = CONFIG) -> str:
    """The full rviz2 in ITS OWN window (3D orbit, Displays / Views panels), with the DRONE_GCS layout. One at a time; returns a status text."""
    global _RVIZ_WINDOW
    if _RVIZ_WINDOW is not None and _RVIZ_WINDOW.poll() is None: return "RViz is already open"
    if not os.path.exists(ROS_SETUP): return f"ROS 2 not found ({ROS_SETUP})"
    env = os.environ.copy(); env["QT_QPA_PLATFORM"] = "xcb"
    for k in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"): env.pop(k, None)       # OpenCV's bundled Qt plugins would crash rviz2
    env["QT_AUTO_SCREEN_SCALE_FACTOR"] = "0"; env["QT_SCALE_FACTOR"] = os.environ.get("DF_RVIZ_SCALE", "1")
    if os.path.exists(FASTDDS): env.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", FASTDDS)
    try:
        _RVIZ_WINDOW = subprocess.Popen(["bash", "-c", f"source {ROS_SETUP} >/dev/null 2>&1; exec rviz2 -d '{config}'"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        return "RViz opened in its own window"
    except OSError as e:
        return f"RViz did not start: {e}"
