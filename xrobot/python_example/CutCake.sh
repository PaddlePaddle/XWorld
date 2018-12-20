#!/bin/bash
echo "Demo"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DCutCake.py
