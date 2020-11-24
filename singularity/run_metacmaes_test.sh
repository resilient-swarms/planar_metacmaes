#!/bin/bash

# this will run inside the singularity container

 export HOME_CONTAINER=/singularity_home/ 
 export BOTS_DIR=$HOME_CONTAINER/Resibots 
 export SFERES_DIR=$HOME_CONTAINER/sferes2
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib/x86_64-linux-gnu:/lib:/usr/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:$BOTS_DIR/lib


test_type=$1       # test_individual, test, or train
condition_type=$2  #damage_meta, a control condition, or random
control_type=$3   # need to supply argument with the method
replicate_number=$4
RESULTS_DIR=$5 # destination folder

mkdir $RESULTS_DIR
mkdir $RESULTS_DIR/${condition_type}_${control_type}
outputdir=${RESULTS_DIR}/${condition_type}_${control_type}/exp${replicate_number}



if [[ "$condition_type" == "damage_meta" ]]; then
	condition_type="meta"  # remove the damage prefix
	gen=500
else
	gen=50000
fi

binary=${SFERES_DIR}/build/exp/planar_cmaes/${test_type}_damage_${condition_type}_binary

if [[ "$test_type" == "individual" ]]; then
	# loop over all damage indices
	for i in 0 1 2 3 4 5 6 7; do
		echo "damage $i"
		${binary} ${replicate_number} ${i} --d ${outputdir} >> ${outputdir}/log_${test_type}${i}.txt
	done
else
	${binary} ${replicate_number} --load ${outputdir}/gen_${gen} --d ${outputdir} -o ${outputdir}/${test_type}_damage_performance >> ${outputdir}/log_${test_type}.txt

fi	

