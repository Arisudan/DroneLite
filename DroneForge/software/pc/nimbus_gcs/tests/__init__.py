import os, tempfile
os.environ.setdefault("DRONEFORGE_SETTINGS", os.path.join(tempfile.mkdtemp(), "flight_settings.json"))   # tests never touch the real settings file
os.environ.setdefault("DRONEFORGE_FLIGHTS", tempfile.mkdtemp())   # tests never write into the real flights folder
# never touch the owner's saved video source or window state either: throw-away files before any test module is imported (holds for unittest discovery, `df.py test`, as well as pytest)
os.environ["DRONEFORGE_VIDEO_STATE"] = os.path.join(tempfile.mkdtemp(prefix="df_video_"), "video.json")
os.environ["DRONEFORGE_UI_STATE"] = os.path.join(tempfile.mkdtemp(prefix="df_ui_"), "gcs_ui.json")
