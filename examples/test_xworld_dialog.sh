#!/bin/bash
PYTHONPATH=../python:$PYTHONPATH \
./test_xworld_dialog \
    --task_mode="interactive" \
    --task_groups_exclusive=1
