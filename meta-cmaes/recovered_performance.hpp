
#ifndef RECOVERED_PERFORMANCE_HPP
#define RECOVERED_PERFORMANCE_HPP

#include "../meta-cmaes/global.hpp"

#if CMAES_CHECK()
#include "../meta-cmaes/cmaescheck_typedefs.hpp"
typedef CMAESCHECKParams BottomParams;
#endif


namespace sferes
{

namespace fit
{

template <typename Phen>
struct RecoveredPerformance
{

    static float _eval_single_envir(const Phen &indiv, size_t damage_option)
    {
        // copy of controller's parameters
        std::vector<double> _ctrl;
        _ctrl.clear();

        for (size_t i = 0; i < 24; i++)
            _ctrl.push_back(indiv.gen().data(i));

#ifdef GRAPHIC
        std::string fileprefix = "video"+std::to_string(damage_option);
#else
        std::string fileprefix = "";
#endif
#ifdef EVAL_ENVIR
        // launching the simulation
        auto robot = global::global_robot->clone();
        simulator_t simu(_ctrl, robot);
#else
        auto robot = global::damaged_robots[damage_option]->clone();
        simulator_t simu(_ctrl, robot, global::damage_sets[damage_option]);
#endif

        simu.run();
        float fitness = simu.euclidean_distance();

        // these assume a behaviour descriptor of size 6.
        if (fitness < -1000)
        {
            // this means that something bad happened in the simulation
            // we do not kill the individual in the meta-map, but set fitness to zero and bd does not contribute
            return 0.0; // will not count towards the sum
            // do not update the descriptor !
        }
        else
        {
            // update the meta-fitness
            return fitness;
        }
    }
    static void _eval_taskmax(std::ostream& os, std::vector<boost::shared_ptr<Phen>>& individuals)
    {
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::cout << "will check " << individuals.size() << "individuals in random order" << std::endl;

        size_t world = 0; // normal environment
        for (size_t damage = 0; damage < global::damage_sets.size(); ++damage)
        {
            os << "DAMAGE \t" << damage << std::endl;

            std::vector<boost::shared_ptr<Phen>> ids_left = individuals;
            float best_so_far = -std::numeric_limits<float>::infinity();
            while (ids_left.size() > 0)
            {
                //random choice
                std::uniform_int_distribution<> dis(0, ids_left.size() - 1);
                int index = dis(gen);
                float val = sferes::fit::RecoveredPerformance<Phen>::_eval_single_envir(*ids_left[index], damage);
                if (val > best_so_far)
                {
                    best_so_far = val;
                }
                // remove index
		        ids_left.erase(ids_left.begin() + index);
		        os << best_so_far << std::endl;
            }
	    

	}
    }
#ifdef EVAL_ENVIR
    static float _eval_all(const Phen &indiv)
    {
#ifdef PRINTING
        //std::cout << "start evaluating " << global::world_options.size() << " environments" << std::endl;
#endif
        float val = 0.0f;
        for (size_t world_option : global::world_options)
        {
            val += _eval_single_envir(indiv, 0);
        }
        return val;
    }
#else
    static float _eval_all(const Phen &indiv)
    {
#ifdef PRINTING
        std::cout << "start evaluating " << global::damage_sets.size() << " damage sets" << std::endl;
#endif
        float val = 0.0f;
        for (size_t i = 0; i < global::damage_sets.size(); ++i)
        {
            // initilisation of the simulation and the simulated robot, robot morphology currently set to raised.skel only
            float temp = _eval_single_envir(indiv, i);
#ifdef GRAPHIC
            std::cout << "performance "<< temp << std::endl;
#endif
             val += temp;
        }
        return val;
    }
#endif
};

} // namespace fit
} // namespace sferes

#endif
