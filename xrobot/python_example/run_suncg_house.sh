#!/bin/bash
echo "SUNCG House (Python)"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskgroup_suncg.py
