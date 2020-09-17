./waf clean
./waf configure --prefix=$BOTS_DIR
./waf
./waf install
./build/test 0.4 0.55 0.4 0.55 0.5 0.55 0.5 0.6 /home/agb/tempPlanar/planar_dart/src/armBody.skel
./build/test 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 /home/agb/tempPlanar/planar_dart/src/armBody.skel

./build/test 0.0 0.7 0.7 0.7 -0.5 0.55 0.5 0.6 /home/agb/tempPlanar/planar_dart/src/armBody.skel
