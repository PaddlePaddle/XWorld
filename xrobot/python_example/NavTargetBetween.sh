#!/bin/bash
echo "XWorld3DNavTargetBetween"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DNavTargetBetween.py
