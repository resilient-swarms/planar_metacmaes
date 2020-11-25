#!/bin/bash

# this will run inside the singularity container

 export HOME_CONTAINER=/singularity_home/ 
 export BOTS_DIR=$HOME_CONTAINER/Resibots 
 export SFERES_DIR=$HOME_CONTAINER/sferes2
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib/x86_64-linux-gnu:/lib:/usr/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:$BOTS_DIR/lib


binary_type=$1
control_type=$2   # need to supply argument with the method
replicate_number=$3
RESULTS_DIR=$4 # destination folder

mkdir $RESULTS_DIR
mkdir $RESULTS_DIR/${binary_type}_${control_type}
outputdir="${RESULTS_DIR}/${binary_type}_${control_type}/exp${replicate_number}"
mkdir ${outputdir}
binary=${SFERES_DIR}/build/exp/planar_cmaes/planarCMAES_${binary_type}_binary



${binary} ${replicate_number} ${control_type} --d ${outputdir} >> ${outputdir}/log.txt
