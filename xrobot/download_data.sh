#!/bin/bash
wget -q -nv -O $1/data.zip "https://www.dropbox.com/sh/czpdi62mm3szc5e/AAAW1EDamOvx_b9iqKW1cPhaa?dl=1"
unzip -qq -o $1/data.zip -d $1
rm $1/data.zip
