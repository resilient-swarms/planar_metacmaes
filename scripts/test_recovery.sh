#!/bin/bash

DEST=$1 # destination folder

for method in $2; do
  for test_type in $3; do
    
    
    if [[ "${method}" == "meta" ]]; then
      if [[ "${test_type}" == "envir" ]]; then
        last_gen=270  
      else
        last_gen=260  
      fi
      method_tag="${test_type}_${method}"
      binary_tag="${test_type}_${method}"
    elif [[  "${method}" == "cmaescheck" ]]; then
      method_tag="${method}"
      binary_tag="${test_type}_cmaescheck"
    else
      method_tag="${method}"
      binary_tag="${test_type}_control"
      last_gen=7000  # leads to 2.8M evals
    fi
    for replicate in $4; do
      echo "start doing ${method}, ${test_type}, run ${replicate}"
      Outfolder="${DEST}/${method_tag}/exp${replicate}"
      
      if [[  "${method}" == "cmaescheck" ]]; then
        # do evolution on each of the
        damage_number=$5 
        Outfolder=${Outfolder}/damage${damage_number}
        mkdir -p ${Outfolder}
        echo "will write to ${Outfolder}"
        ${SFERES_DIR}/build/exp/MAP-Elites-Rhex/test_${binary_tag}_binary ${replicate} ${damage_number} --d ${Outfolder}
      else
        echo "will write to ${Outfolder}"
        ${SFERES_DIR}/build/exp/MAP-Elites-Rhex/test_${binary_tag}_binary --load ${Outfolder}/gen_${last_gen} --d ${Outfolder} -o ${Outfolder}/${test_type}_performance
      fi
    done

  done

done
