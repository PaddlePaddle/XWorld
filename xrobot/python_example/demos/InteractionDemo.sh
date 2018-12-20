#!/bin/bash
echo "Interaction Demo"

PYTHONPATH=../..:..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python InteractionDemo.py
