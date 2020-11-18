#!/bin/bash

# this will run inside the singularity container


export SFERES_DIR="/singularity_home/sferes2"

binary_type=$1
control_type=$2   # need to supply argument with the method
replicate_number=$3
RESULTS_DIR=$4 # destination folder

mkdir $RESULTS_DIR
mkdir $RESULTS_DIR/${control_type}
mkdir ${RESULTS_DIR}/${control_type}/exp${replicate_number}

${SFERES_DIR}/build/exp/planar_cmaes/planarCMAES_${binary_type}_binary ${replicate_number} ${control_type} --d ${RESULTS_DIR}/${control_type}/exp${replicate_number} >> log.txt
