

//#define META_CMAES 0
//#define RANDOM_WEIGHT 1
//#define CONDITION_DUTY_CYCLE 2
//#define CONDITION_BODY_ORIENTATION 3
//#define CONDITION_LINEAR_VELOCITY 4
//#define CONDITION_CMAES_CHECK 5

#define META_CMAES 0
#define RANDOM_WEIGHT 1
#define CONDITION_POSITIONAL_COORD 2
#define CONDITION_POLAR_COORD 3
#define CONDITION_RESULTANT_ANGLE 4
#define CONDITION_ANGLE_SUM 5
#define CONDITION_CMAES_CHECK 6

//#define EXPERIMENT_TYPE CONDITION_DUTY_CYCLE

#define META() EXPERIMENT_TYPE == META_CMAES
#define CONTROL() EXPERIMENT_TYPE > META_CMAES && EXPERIMENT_TYPE < CONDITION_CMAES_CHECK
#define CMAES_CHECK() EXPERIMENT_TYPE == CONDITION_CMAES_CHECK
#define GLOBAL_WEIGHT() EXPERIMENT_TYPE == RANDOM_WEIGHT
#define NO_WEIGHT() EXPERIMENT_TYPE > RANDOM_WEIGHT && EXPERIMENT_TYPE < CONDITION_CMAES_CHECK
#define WEIGHT() EXPERIMENT_TYPE == RANDOM_WEIGHT || EXPERIMENT_TYPE == META_CMAES
#define POS_C() EXPERIMENT_TYPE == CONDITION_POSITIONAL_COORD
#define POL_C() EXPERIMENT_TYPE == CONDITION_POLAR_COORD
#define RA_C() EXPERIMENT_TYPE == CONDITION_RESULTANT_ANGLE
#define AS_C() EXPERIMENT_TYPE == CONDITION_ANGLE_SUM

#define ENVIR_TESTS() EVAL_ENVIR == 1 && (TEST || META())
#define DAMAGE_TESTS() EVAL_ENVIR == 0 && (TEST || META())

#ifdef GRAPHIC
#define NO_PARALLEL
#endif

#define FRICTION 1.0

//#define TAKE_COMPLEMENT // whether or not to test for generalisation to unseen world/damage conditions

//#define PRINTING
//#define CHECK_PARALLEL
#ifndef TEST
#if NUM_CORES > 1
    #define PARALLEL_RUN
#else
     #define SERIAL_RUN
#endif
#endif

#include <boost/random.hpp>
#include <iostream>
#include <mutex>
#include <cstdlib>

#include <sferes/gen/evo_float.hpp>
//#include <sferes/gen/sampled.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/phen/parameters.hpp>

#include <sferes/stat/pareto_front.hpp>

#include <planar_dart/planar_dart_simu.hpp>
#include <chrono>

#include <sferes/stc.hpp>
#include <meta-cmaes/global.hpp>

#if META()
#include <meta-cmaes/meta-CMAES.hpp>
#include <meta-cmaes/stat_maps.hpp>
#include <meta-cmaes/stat_pop.hpp>
#elif CMAES_CHECK()
#include <meta-cmaes/cmaescheck_fitness.hpp>
#include <meta-cmaes/cmaes.hpp>
#include <sferes/stat/best_fit.hpp>
typedef boost::fusion::vector<sferes::stat::BestFit<phen_t, CMAESCHECKParams>> stat_t;

typedef modif::Dummy<> modifier_t;
typedef sferes::ea::Cmaes<phen_t, eval_t, stat_t, modifier_t,CMAESCHECKParams> ea_t;
#else
#include <meta-cmaes/control_typedefs.hpp>
#ifdef TEST
#include <meta-cmaes/stat_map.hpp>
#else
#include <modules/map_elites/stat_map.hpp>
#endif
#include <sferes/ea/ea.hpp>
#include <modules/map_elites/map_elites.hpp>
typedef boost::fusion::vector<sferes::stat::Map<phen_t, BottomParams>> stat_t;

typedef modif::Dummy<> modifier_t;
typedef sferes::ea::MapElites<phen_t, eval_t, stat_t, modifier_t, BottomParams> ea_t;
#endif

#include <sferes/run.hpp>



using namespace sferes;

int main(int argc, char **argv)
{
    std::srand(atoi(argv[1])); 
    ea_t ea;
#ifdef PARALLEL_RUN
    sferes::eval::init_shared_mem();
#endif

#if CMAES_CHECK()
    global::damage_index = atoi(argv[2]);
    std::cout << "will do damage " << global::damage_index << std::endl;
#endif

    global::init_simu(std::string(argv[1]), std::string(std::getenv("BOTS_DIR")) + "/share/armBody.skel");
    run_ea(argc, argv, ea);

    global::global_robot.reset();
    return 0;
}
