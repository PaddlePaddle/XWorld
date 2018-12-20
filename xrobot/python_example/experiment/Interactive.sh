#!/bin/bash
echo "Experiment"

PYTHONPATH=../..:..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python Interactive.py
