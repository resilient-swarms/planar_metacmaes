
#ifndef PAIRWISE_DIST_HPP
#define PAIRWISE_DIST_HPP

#include <meta-cmaes/global.hpp>

#if CMAES_CHECK()
#include <meta-cmaes/cmaescheck_typedefs.hpp>
typedef CMAESCHECKParams BottomParams;
#endif

namespace sferes
{

    namespace fit
    {

        template <typename Phen>
        struct PairwiseDist
        {
            static float _pairwise_dist(std::vector<Eigen::VectorXd> positions)
            {
                float dist = 0.0f;
                size_t num_comps=(positions.size() - 1)*positions.size()/2;
                for (size_t i=0; i < positions.size() - 1; ++i)
                {
                    Eigen::VectorXd pos = positions[i];
                    if (pos.size() == 0)
                        continue;//counts as 0
                    for (size_t j=i+1; j > i && j < positions.size(); ++j)
                    {
                        Eigen::VectorXd pos2 = positions[i];
                        if (pos2.size() == 0)
                            continue;//counts as 0
                        dist += (pos - pos2).norm();
                    }
                }
                return dist/(float) num_comps;
            }
            static Eigen::VectorXd _eval_single_envir(const Phen &indiv, size_t damage_option)
            {

                // copy of controller's parameters
                std::vector<double> _ctrl;
                _ctrl.clear();

                for (size_t i = 0; i < 8; i++)
                    _ctrl.push_back(indiv.gen().data(i));

#ifdef EVAL_ENVIR
                // launching the simulation
                auto robot = global::global_robot->clone();
                simulator_t simu(_ctrl, robot);
#else
                auto robot = global::damaged_robots[damage_option]->clone();
                simulator_t simu(_ctrl, robot, global::damage_sets[damage_option]);
#endif

                simu.run();
                if (simu.euclidean_distance() == -1)
                {
                    return {}; //empty vector for invalid solution, will contribute only 0 to sum
                }
                return simu.final_position();
            }
#ifdef EVAL_ENVIR
            static float _eval_all(const Phen &indiv)
            {
#ifdef PRINTING
                //std::cout << "start evaluating " << global::world_options.size() << " environments" << std::endl;
#endif
                std::vector<Eigen::VectorXd> positions;
                for (size_t world_option : global::world_options)
                {
                    positions.push_back(PairwiseDist::_eval_single_envir(indiv, world_option));
                }
                return _pairwise_dist(positions);
            }
#else
            static float _eval_all(const Phen &indiv)
            {
#ifdef PRINTING
                std::cout << "start evaluating " << global::damage_sets.size() << " damage sets" << std::endl;
#endif
                std::vector<Eigen::VectorXd> positions;
                for (size_t i = 0; i < global::damage_sets.size(); ++i)
                {
                    // initilisation of the simulation and the simulated robot, robot morphology currently set to raised.skel only
                    positions.push_back(PairwiseDist::_eval_single_envir(indiv, i));
#ifdef GRAPHIC
                    std::cout << "position " << positions.back() << std::endl;
#endif
                }
                return PairwiseDist::_pairwise_dist(positions);
            }
#endif
        };

    } // namespace fit
} // namespace sferes

#endif
