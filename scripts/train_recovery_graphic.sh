#!/bin/bash

individual=$3

for method in $1; do
    for test_type in $2; do

        if [[ "${method}" == "meta" ]]; then
            if [[ "${test_type}" == "envir" ]]; then
                last_gen=270 # leads to 2.8M evals
            else
                last_gen=260 # leads to 2.8M evals
            fi
            method_tag="${test_type}_${method}"
            binary_tag="${test_type}_${method}"
        else
            method_tag="${method}"
            binary_tag="${test_type}_control"
            last_gen=7000 # leads to 2.8M evals
        fi

        echo "start doing ${method}, ${test_type}"

        Outfolder=~/ToyData/${method_tag} # destination folder
        echo "will write to ${Outfolder}"

        ${SFERES_DIR}/build/exp/MAP-Elites-Rhex/train_${binary_tag}_binary --load ${Outfolder}/gen_${last_gen} --d ${Outfolder} -o ${Outfolder}/${test_type}_performanceGRAPHDUMMY -n $individual
    done

done
