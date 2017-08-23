#!/bin/bash

(($# < 2)) && echo "usage: ./export_vars.sh <name> <value>"

RED='\033[0;31m'
NC='\033[0m' # No Color

VAR_NAME=$1
VAR_VALUE=$2

if `grep "export $VAR_NAME=" ~/.bashrc`
then
    sed -i "/$VAR_NAME/d" ~/.bashrc
fi

echo "export $VAR_NAME=$VAR_VALUE" >> ~/.bashrc
printf "Evironment variable $VAR_NAME added. ${RED}*** Please restart bash shell ***${NC}\n"
