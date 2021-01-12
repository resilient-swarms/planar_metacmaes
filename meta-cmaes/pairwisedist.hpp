
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
                    for (size_t j = i + 1; j < positions.size(); ++j)
                    {
                        Eigen::VectorXd pos2 = positions[j];
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
                // launching the simulation
                auto robot = global::global_robot->clone();

                simulator_t simu(_ctrl, robot, global::damage_sets[damage_option]);


                simu.run();
                return std::tuple<Eigen::VectorXd, bool>{simu.final_position(), simu.dead()};
            }
        };

    } // namespace fit
} // namespace sferes

#endif
