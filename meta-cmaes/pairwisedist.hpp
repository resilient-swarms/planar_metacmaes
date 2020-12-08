
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
            static float compute(std::vector<Eigen::VectorXd> positions)
            {
                if (positions.size() <= 1)
                {
                    return 0.0f;
                }
                float dist = 0.0f;
                for (size_t i = 0; i < positions.size() - 1; ++i)
                {
                    Eigen::VectorXd pos = positions[i];
                    if (pos.size() == 0)
                        continue; //counts as 0
                    for (size_t j = i + 1; j < positions.size(); ++j)
                    {
                        Eigen::VectorXd pos2 = positions[j];
                        if (pos2.size() == 0)
                            continue; //counts as 0
                        dist += (pos - pos2).norm();
                    }
                }
                return dist;// no averaging over comparisons so that maps with high coverage and high pairwise distance are best
            }
            static std::tuple<Eigen::VectorXd, bool> _eval_single_envir(const Phen &indiv, size_t damage_option)
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
                return std::tuple<Eigen::VectorXd, bool>{simu.final_position(), simu.euclidean_distance() == -1};
            }
        };

    } // namespace fit
} // namespace sferes

#endif
