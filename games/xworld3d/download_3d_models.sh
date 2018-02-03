#!/bin/bash

## download 3d models for xworld3d
wget -O models_3d.zip "https://github.com/yu239/XWorld3D-model-zoo/blob/master/models_3d.zip?raw=true"
unzip -o models_3d.zip
rm -f models_3d.zip

wget -O glsl.zip "https://github.com/yu239/XWorld3D-model-zoo/blob/master/glsl.zip?raw=true"
unzip -o glsl.zip
rm -f glsl.zip
