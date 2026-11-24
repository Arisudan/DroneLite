#!/usr/bin/env python3
"""Betaflight blackbox decoder: the implementation lives in software/pc/nimbus_gcs/bfl.py (used by the GCS Logs page too).
Usage: fc_decode_blackbox_bfl.py LOG.BFL [out.csv]"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "software", "pc"))
from nimbus_gcs.bfl import main
main()
