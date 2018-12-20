#!/bin/bash
echo "Dialog"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python Dialog.py
