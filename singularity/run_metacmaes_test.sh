#!/bin/bash

# this will run inside the singularity container

 export HOME_CONTAINER=/singularity_home/ 
 export BOTS_DIR=$HOME_CONTAINER/Resibots 
 export SFERES_DIR=$HOME_CONTAINER/sferes2
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib/x86_64-linux-gnu:/lib:/usr/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:$BOTS_DIR/lib


 test_type=$1       #  test or train (train not implemented (yet))
condition_type=$2  # meta, control, or random
fm=$3              # 2D,4D, or 6D for control; otherwise linfm, nonlinfm or selectionfm; for meta conditions added genes as well, e.g. linfm_added0genes
control_type=$4   # need to supply argument with the method, e.g. b1p1 for meta, "" for random, and "pol" for control
replicate_number=$5
RESULTS_DIR=$6 # destination folder

mkdir $RESULTS_DIR
if [[ $condition_type == meta* ]]; then
	condition_prefix="damage_meta_"
else
	condition_prefix=$condition_type
fi
mkdir $RESULTS_DIR/${condition_prefix}${fm}_${control_type}
outputdir="${RESULTS_DIR}/${condition_prefix}${fm}_${control_type}/exp${replicate_number}"

mkdir ${outputdir}

currentdir=$PWD
# get the last filename
max="-1"
get_last_filename()
{
# get the last filename
shopt -s extglob
cd ${outputdir}
files=`ls -d !(*.*)` # gen_files don't have extension
for gen_file in $files; do
	number=${gen_file#"gen_"}
	#echo $number
	if (( $number > $max )); then
    		max=$number
	fi

done
}

get_last_filename
echo "max generation found is $max"
outputdir=$currentdir/$outputdir
cd /singularity_home/planar_metacmaes/ 
echo "am now in $PWD"
binary=${SFERES_DIR}/build/exp/planar_cmaes/${test_type}_damage_${condition_type}_binary
${binary} ${replicate_number} --load ${outputdir}/gen_${max} --d ${outputdir} -o ${outputdir}/${test_type}_damage_performance >> ${outputdir}/log_${test_type}.txt	

