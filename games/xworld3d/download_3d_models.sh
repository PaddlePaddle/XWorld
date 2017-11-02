#!/bin/bash

## download 3d models for xworld3d
mkdir -p models_3d
mkdir -p glsl

cd models_3d
wget -O models_3d.zip "https://www.dropbox.com/sh/4tulfzgybejinnf/AACGQ0bhFfHwR1JxfntX9ynpa?dl=1"
unzip models_3d.zip
rm -f models_3d.zip

cd ../glsl
wget -O glsl.zip "https://www.dropbox.com/sh/pc7yhg8jod0ganx/AABXkGiIYrgTaBJpPgmp9OFaa?dl=1"
unzip glsl.zip
rm -f glsl.zip
cd ..
