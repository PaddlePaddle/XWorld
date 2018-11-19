#!/bin/bash
echo "Navigation Agent (Python)"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskgroup_navagent.py
