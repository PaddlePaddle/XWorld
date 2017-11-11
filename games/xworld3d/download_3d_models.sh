#!/bin/bash

## download 3d models for xworld3d
mkdir -p models_3d
mkdir -p glsl

cd models_3d
wget -O models_3d.zip "https://github.com/yu239/XWorld3D-model-zoo/blob/master/models_3d.zip"
unzip models_3d.zip
rm -f models_3d.zip

cd ../glsl
wget -O glsl.zip "https://github.com/yu239/XWorld3D-model-zoo/blob/master/glsl.zip"
unzip glsl.zip
rm -f glsl.zip
cd ..
