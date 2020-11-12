#!/bin/bash

cp -rv ~/planar_metacmaes/meta-cmaes/ ${BOTS_DIR}/include
cp -rv ~/planar_metacmaes/planar_cmaes/ ${SFERES_DIR}/exp
cd ~/planar_metacmaes/planar_dart/
./waf distclean
./waf configure --prefix=$BOTS_DIR
./waf
./waf install
cd $SFERES_DIR
./waf distclean
./waf configure --exp planar_cmaes
./waf --exp planar_cmaes
