# planar_metacmaes

Analysis of learning behaviour-performances with Meta-Evolution (Using 8-joint planar robot) 

## Setting the environment

- Tested on Ubuntu 16.04
- GCC/GPP version 5 (Recommended)
- Set enivronment variables `BOTS_DIR`(Installation Path), `LD_LIBRARY_PATH`, `SFERES_DIR` (Sferes2 installation path) and `RESULTS_DIR`. This can be done by adding these lines to `~/.bashrc` or `~/.zshrc` file:
```
export BOTS_DIR=#/path/to/install
export SFERES_DIR=#/path/for/sferes/install
export RESULTS_DIR=#/path/to/results/folder
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib
```
- Install required packages:
```
sudo apt-add-repository ppa:libccd-debs/ppa # for ubuntu versions < 16
sudo apt-add-repository ppa:fcl-debs/ppa    # for ubuntu versions < 16
sudo apt-add-repository ppa:dartsim/ppa
sudo apt-get update

sudo apt-get install build-essential cmake pkg-config git
sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev libode-dev
sudo apt-get install libopenscenegraph-dev
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install freeglut3-dev

sudo apt-get install libtinyxml-dev libtinyxml2-dev
sudo apt-get install liburdfdom-dev liburdfdom-headers-dev

sudo apt-get install libboost-all-dev
sudo apt-get install libboost-serialization-dev libboost-filesystem-dev libboost-test-dev libboost-program-options-dev libboost-thread-dev libboost-regex-dev libboost-graph-dev
sudo apt-get install libtbb-dev
```
- In order to use planar files, you need to clone this repository to a `TEMP_DIR`:
```
cd ${TEMP_DIR}
git clone https://github.com/resilient-swarms/planar_metacmaes
```
- To install [DART](https://github.com/dartsim/dart) simulator:
```
git clone git://github.com/dartsim/dart.git
cd dart
git checkout v6.3.0

mkdir build
cd build
cmake -DDART_ENABLE_SIMD=ON -DCMAKE_INSTALL_PREFIX=$BOTS_DIR ..
make -j4
make install
```
- Install DART Wrapper for this specific Planar Robot including controller includes. Also, copy SKEL File of the robot to the Running Environment:
```
cd ${TEMP_DIR}/planar_metacmaes/planar_dart/
./waf configure --prefix=$BOTS_DIR
./waf
./waf install

cp -r ${TEMP_DIR}/planar_metacmaes/planar_dart/src/armBody.skel ${BOTS_DIR}/share/armBody.skel
```
- Clone [META-CMAES](https://github.com/resilient-swarms/meta-cmaes) Library and copy `meta-cmaes` dir to environment includes:
```
cd ${BOTS_DIR}/include
git clone https://github.com/resilient-swarms/meta-cmaes.git
```
- Replace changed `meta_cmaes` files for planar robot using:
```
cp -r ${TEMP_DIR}/planar_metacmaes/meta-cmaes/ ${BOTS_DIR}/include
```
- Get [SFERES](https://github.com/sferes2/sferes2) Library along with [MAP-Elites](https://github.com/resilient-swarms/map_elites) module, and put them into a directory of choice. Then compile and test the Installation:
```
git clone https://github.com/sferes2/sferes2.git

cd ${SFERES_DIR}/modules
git clone https://github.com/resilient-swarms/map_elites.git

cd ${SFERES_DIR}
./waf configure
./waf
```
- Create Experiment Folder and Copy `planar_metacmaes`
```
cd ${SFERES_DIR} && mkdir -p exp && cd exp
cp -r ${TEMP_DIR}/planar_metacmaes/planar_cmaes/ ${SFERES_DIR}/exp
```
- Testing Installation can be done by`:
```
cd ${SFERES_DIR}
./waf configure --exp planar_cmaes --dart $BOTS_DIR
./waf --exp planar_cmaes --dart $BOTS_DIR
```
## Executing Experiments

- Control Variables for building binaries
    * `BUILD_PHASE`: 
      - "TEST": test for recovery from all test-set damages (not seen in meta-evolution)
      - "TRAIN": test for recovery from all train-set damages (seen in meta-evolution)
      - "INDIVIDUAL_DAMAGE": for each individual damage, random search for the best recovery solution
      - any other string: evolution
    * `BUILD_TYPE`: in case BUILD_PHASE=evolution, determines the type of condition,
      - "damage_meta": Building for meta conditions
      - "random": Building the random control condition
      - any other string: Building all the other control conditions (control2D, control4D, and control6D)
    * `NUM_CORES`: the number of cores (1 by default)
    * `BUILD_GRAPHICS`
      - True: Building with generating robot captures
      - any other string: Building wihout recording captures
  

  
  
- To compile, use `bash ${TEMP_DIR}/planar_metacmaes/scripts/make.sh` (**NOTE**: Use only if [this line](https://github.com/agb1n19/planar_metacmaes/README.md#L95) is already executed)

- To run binaries, use `${SFERES_DIR}/build/exp/planar_cmaes/planarCMAES_${binary_type}$_binary ${replicate_number} ${control_type} --d ${RESULTS_DIR}/${control_type}/exp${replicate_number} >> ${logfile}`,
  * `control_type` can be `pos`, `pol`, `jpa`, `rjpa`, or `as` for control conditions, or `b#1p#2` for the meta-condition, where #1 and #2 are numbers in {1,2,5,10}. 
  * `binary_type` can be `damage_meta`, `random`, `control2D`, `control4D` or `control6D` (depending on behavioural dimensions of ${control_type})
  * `replicate_number` is an identifier to identify experiment replicate
  
[below needs updating with control_type]

- To assess train performance, use `${SFERES_DIR}/build/exp/planar_cmaes/train_${binary_type}_binary --load ${Outfolder}/gen_${last_gen} --d ${Outfolder} -o ${Outfolder}/damage_trainperformance`
  * `binary_type` can be `damage_meta` or `damage_control`
  * `Outfolder` is usually `${RESULTS_DIR}/${type}/exp${replicate_number}`
  * `last_gen` is the maximum of the generation to be considered
  
- To assess test performance, use `${SFERES_DIR}/build/exp/planar_cmaes/test_${binary_type}_binary --load ${Outfolder}/gen_${last_gen} --d ${Outfolder} -o ${Outfolder}/damage_performance`
