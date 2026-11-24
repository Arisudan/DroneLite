#!/usr/bin/env python3
"""Writes config/gui_house.config: Gazebo's default GUI layout plus an Image Display showing the delayed O4 feed (/droneforge/camera/feed, with the latency read-out).
usage: python3 tools/make_gui_config.py   (finds Gazebo's installed default gui.config)"""
import glob, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "config", "gui_house.config")
cands = glob.glob("/opt/ros/*/opt/gz_sim_vendor/share/gz/gz-sim*/gui/gui.config") + glob.glob("/usr/share/gz/gz-sim*/gui/gui.config") + glob.glob("/opt/ros/*/share/gz/gz-sim*/gui/gui.config")
if not cands: print("no default gui.config found"); sys.exit(1)
s = open(sorted(cands)[-1]).read()
plugin = '''
<!-- O4 feed as the PC receives it: delayed, with the latency read-out (camera_feed_latency.py) -->
<plugin filename="ImageDisplay" name="O4 feed (delayed)">
  <gz-gui>
    <title>O4 feed (delayed)</title>
    <property type="bool" key="showTitleBar">true</property>
    <property type="string" key="state">floating</property>
    <property type="double" key="width">480</property>
    <property type="double" key="height">380</property>
    <property type="double" key="x">20</property>
    <property type="double" key="y">60</property>
  </gz-gui>
  <topic>/droneforge/camera/feed</topic>
  <topic_picker>false</topic_picker>
</plugin>
'''
if "O4 feed" not in s: s = s.rstrip() + "\n" + plugin
open(OUT, "w").write(s); print("written", os.path.abspath(OUT))
