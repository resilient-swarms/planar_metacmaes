export RESIBOTS_DIR=/robo/
export BOTS_DIR=/planar_robo/
export SFERES_DIR=/home/agb/sferes2/
export RESULTS_DIR=/planar_robo/results/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib

cd
git clone git@github.com:agb1n19/planar_metacmaes.git
cd ~/planar_metacmaes/
sh setEnv.sh
mkdir extras
cd ~/planar_metacmaes/extras/
git clone git://github.com/dartsim/dart.git
cd dart
git checkout v6.3.0
mkdir build
cd build
cmake -DDART_ENABLE_SIMD=ON ..
make -j4
make install
cd ~/planar_metacmaes/planar_dart/
./waf configure --prefix=$BOTS_DIR
./waf
./waf install
cd
git clone https://github.com/sferes2/sferes2.git
cd $SFERES_DIR/modules
git clone https://github.com/resilient-swarms/map_elites.git
cd $SFERES_DIR
./waf configure
./waf
mkdir exp && cd exp/
cp -r ~/planar_metacmaes/planar_cmaes/ $SFERES_DIR/exp/
cd $SFERES_DIR
export BUILD_META=FALSE
./waf configure --exp planar_cmaes
./waf --exp planar_cmaes

