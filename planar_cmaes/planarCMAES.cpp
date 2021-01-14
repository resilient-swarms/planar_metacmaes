

#define C_META_CMAES 0
#define C_RANDOM_WEIGHT 1
#define C_CONTROL 2
#define C_CMAES_CHECK 3

#define META() EXPERIMENT_TYPE == C_META_CMAES
#define CONTROL() EXPERIMENT_TYPE == C_CONTROL
#define CONTROL_PLUS() EXPERIMENT_TYPE == C_CONTROL || EXPERIMENT_TYPE == C_RANDOM_WEIGHT
#define CMAES_CHECK() EXPERIMENT_TYPE == C_CMAES_CHECK
#define GLOBAL_WEIGHT() EXPERIMENT_TYPE == C_RANDOM_WEIGHT
#define NO_WEIGHT() EXPERIMENT_TYPE == C_CONTROL
#define WEIGHT() EXPERIMENT_TYPE == C_RANDOM_WEIGHT || EXPERIMENT_TYPE == C_META_CMAES

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
typedef sferes::ea::Cmaes<phen_t, eval_t, stat_t, modifier_t, CMAESCHECKParams> ea_t;
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

#include <meta-cmaes/bottom_typedefs.hpp>
#include <sferes/run.hpp>
#if META()
#include <meta-cmaes/top_typedefs.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(MutationAnnealing)
BOOST_CLASS_EXPORT_IMPLEMENT(EpochAnnealing)
BOOST_CLASS_EXPORT_IMPLEMENT(BothAnnealing)
BOOST_CLASS_EXPORT_IMPLEMENT(MutationEndogenous)
BOOST_CLASS_EXPORT_IMPLEMENT(EpochEndogenous)
BOOST_CLASS_EXPORT_IMPLEMENT(BothEndogenous)
BOOST_CLASS_EXPORT_IMPLEMENT(RL)
BOOST_CLASS_EXPORT_IMPLEMENT(RLController)

#endif
namespace sferes
{
    namespace gen
    {
        template <>
        void bottom_gen_t::mutate()
        {
            if (BottomParams::sampled::ordered)
            {
                for (size_t i = 0; i < _data.size(); ++i)
                {
#if META()
                    float mutation_rate = sferes::eval::param_ctrl->get_mutation_rate();
#else
                    float mutation_rate = BottomParams::sampled::mutation_rate;
#endif
                    if (misc::rand<float>() < mutation_rate)
                    {
                        if (misc::flip_coin())
                            _data[i] = std::max(0, (int)_data[i] - 1);
                        else
                            _data[i] = std::min((int)BottomParams::sampled::values_size() - 1,
                                                (int)_data[i] + 1);
                    }
                }
            }
            else
            {
#if META()
                float mutation_rate = sferes::eval::param_ctrl->get_mutation_rate();
#else
                float mutation_rate = BottomParams::sampled::mutation_rate;
#endif
                BOOST_FOREACH (size_t &v, _data)
                    if (misc::rand<float>() < mutation_rate)
                        v = misc::rand<size_t>(0, BottomParams::sampled::values_size());
                _check_invariant();
            }
            _check_invariant();
        }
    } // namespace gen
} // namespace sferes

using namespace sferes;

int main(int argc, char **argv)
{
    long seed = atoi(argv[1]);
    std::srand(seed);
    ea_t ea;
#ifdef PARALLEL_RUN
    sferes::eval::init_shared_mem<shared_memory_t>();
#endif

#ifdef TEST
#if CMAES_CHECK()
    global::damage_index = atoi(argv[2]);
    std::cout << "will do damage " << global::damage_index << std::endl;
#endif
#else
#if CONTROL()
    global::set_condition(argv[2]);
#elif META()
    size_t index = 0;
    for (size_t i = 0; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--d")
        {
            index = i + 1;
	    break;
        }
    }
    sferes::eval::param_ctrl = init_parameter_control(seed, std::string(argv[2]), argv[index]);
#endif
#endif

    global::init_simu(std::string(argv[1]), std::string(std::getenv("BOTS_DIR")) + "/share/armBody.skel");
    run_ea(argc, argv, ea);

    global::global_robot.reset();
    return 0;
}
