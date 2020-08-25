#!/bin/bash

export BUILD_META=FALSE
export BUILD_GRAPHIC=False
cd $SFERES_DIR
./waf distclean
./waf configure --exp planar_cmaes
./waf --exp planar_cmaes
