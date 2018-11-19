#!/bin/bash
echo "Interactions Demo (Python)"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskgroup_interaction.py
