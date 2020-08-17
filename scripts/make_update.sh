#!/bin/bash

# use this one when you have previously compiled binaries but you changed code
cd $SFERES_DIR
./waf distclean
./waf configure --exp MAP-Elites-Rhex
./waf --exp MAP-Elites-Rhex
