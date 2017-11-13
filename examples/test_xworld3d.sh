#!/bin/bash

PYTHONPATH=../python:$PYTHONPATH \
./test_xworld3d \
    --x3_conf=../games/xworld3d/confs/navigation.json \
    --x3_glsl_path=../games/xworld3d/glsl \
    --pause_screen=1 \
    --context=1 \
    --task_groups_exclusive=1
