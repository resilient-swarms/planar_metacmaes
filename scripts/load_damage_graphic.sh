


#!/bin/bash

args="--load ${1}/${2} --o ${3} -n ${4}"
echo "will use args ${args}"
bash scripts/run_damage_graphic.sh $1 "$args"
