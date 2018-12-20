#!/bin/bash
echo "Benchmark $@"

PYTHONPATH=../..:..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python Benchmark.py $@
