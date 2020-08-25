#!/bin/bash

cd $SFERES_DIR


jobtocome="build/exp/planar_cmaes/planarCMAES_damage_binary -d ${1} ${2}"
echo "Starting the following command: "${jobtocome}" "
${jobtocome}
