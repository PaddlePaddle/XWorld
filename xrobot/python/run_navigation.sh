#!/bin/bash
echo "Navigation Task (Python)"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskgroup_navigation.py
