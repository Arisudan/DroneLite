import os, tempfile
os.environ.setdefault("DRONEFORGE_SETTINGS", os.path.join(tempfile.mkdtemp(), "flight_settings.json"))   # tests never touch the real settings file
os.environ.setdefault("DRONEFORGE_FLIGHTS", tempfile.mkdtemp())   # tests never write into the real flights folder
