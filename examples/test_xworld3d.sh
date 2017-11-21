#!/bin/bash

PYTHONPATH=../python:$PYTHONPATH \
./test_xworld3d \
    --x3_conf=../games/xworld3d/confs/navigation.json \
    --pause_screen=1 \
    --context=1 \
    --task_groups_exclusive=1
