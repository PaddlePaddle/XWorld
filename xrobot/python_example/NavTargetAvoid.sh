#!/bin/bash
echo "XWorld3DNavTargetAvoid"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DNavTargetAvoid.py
