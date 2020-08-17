#!/bin/bash

DEST=$1 # destination folder

for method in $2; do

    if [[ "${method}" == "envir_meta" ]]; then
        test_type="damage"
        last_gen=270             # leads to 2.8M evals
        binary_tag="damage_meta" # ensure that damages are selected
    else
        test_type="envir"
        last_gen=260            # leads to 2.8M evals
        binary_tag="envir_meta" # ensure that environments are selected
    fi
    method_tag="${method}" # ensure we take the correct outputfolder, with the right population and the right destination folder

    for replicate in $3; do
        echo "start doing ${method}, ${test_type}, run ${replicate}"
        Outfolder="${DEST}/${method_tag}/exp${replicate}"
        echo "will write to ${Outfolder}"

        nohup ${SFERES_DIR}/build/exp/MAP-Elites-Rhex/train_${binary_tag}_binary --load ${Outfolder}/gen_${last_gen} --d ${Outfolder} -o ${Outfolder}/${test_type}_trainperformance &
    done

done
