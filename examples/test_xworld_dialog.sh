#!/bin/bash
PYTHONPATH=../python:$PYTHONPATH \
./test_xworld_dialog \
    --pause_screen=1 \
    --context=1 \
    --task_mode="arxiv_interactive" \
    --task_groups_exclusive=1
