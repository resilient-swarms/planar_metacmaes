#!/bin/bash

cp -arv ~/planar_metacmaes/meta-cmaes/ ${BOTS_DIR}/include
cp -arv ~/planar_metacmaes/planar_cmaes/ ${SFERES_DIR}/exp
cp -avr ~/planar_metacmaes/planar_dart/src/armBody.skel ${BOTS_DIR}/share/armBody.skel
cd ~/planar_metacmaes/planar_dart/
./waf distclean
./waf configure --prefix=$BOTS_DIR
./waf
./waf install
