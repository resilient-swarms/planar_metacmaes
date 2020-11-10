#!/bin/bash

#export BUILD_META=FALSE
#export BUILD_GRAPHIC=False

cd ~/planar_metacmaes/planar_dart/
./waf configure --prefix=$BOTS_DIR
./waf
./waf install
# cp -rv ~/planar_metacmaes/meta-cmaes/ ${BOTS_DIR}/include
# cp -rv ~/planar_metacmaes/planar_cmaes/ ${SFERES_DIR}/exp
cd $SFERES_DIR
./waf distclean
./waf configure --exp planar_cmaes
./waf --exp planar_cmaes
