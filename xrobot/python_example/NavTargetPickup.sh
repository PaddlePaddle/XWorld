#!/bin/bash
echo "XWorld3DNavTargetPickup"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DNavTargetPickup.py
