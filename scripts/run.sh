#!/bin/bash

cd $SFERES_DIR


jobtocome="build/exp/MAP-Elites-Rhex/rhex_metaCMAES_envir_binary -d ${1} ${2}"
echo "Starting the following command: "${jobtocome}" "
${jobtocome}