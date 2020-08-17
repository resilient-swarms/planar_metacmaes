#!/bin/bash

args="--load ${1}/${2} --o ${3} -n ${4}"

bash scripts/run_graphic.sh $1 "$args"