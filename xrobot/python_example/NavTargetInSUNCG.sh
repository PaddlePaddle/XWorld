#!/bin/bash
echo "XWorld3DNavTarget"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DNavTargetInSUNCG.py
