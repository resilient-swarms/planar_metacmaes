
#ifndef RECOVERED_PERFORMANCE_HPP
#define RECOVERED_PERFORMANCE_HPP

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
        struct RecoveredPerformance
        {

            static int _eval_single_envir(const Phen &indiv, size_t damage_option)
            {

                // copy of controller's parameters
                std::vector<double> _ctrl;
                _ctrl.clear();

                for (size_t i = 0; i < 8; i++)
                    _ctrl.push_back(indiv.gen().data(i));

#ifdef GRAPHIC
                std::string fileprefix = "video" + std::to_string(damage_option);
#else
                std::string fileprefix = "";
#endif
#ifdef EVAL_ENVIR
                // launching the simulation
                auto robot = global::global_robot->clone();
                simulator_t simu(_ctrl, robot);
#else
                auto robot = global::damaged_robots[damage_option]->clone();
                // generate random target

                simulator_t simu(_ctrl, robot, global::damage_sets[damage_option]);
#endif

                simu.run(true);
                return simu.find_bin();
            }

            static float _eval_all(const Phen &indiv)
            {

                std::cout << "not implemented " << std::endl;
                exit(0);
                return 0.f;
            }
            static void _eval_taskmax(std::ostream &os, std::vector<boost::shared_ptr<Phen>> &individuals)
            {
                std::cout << "not implemented " << std::endl;
                exit(0);
                //                 std::random_device rd;  //Will be used to obtain a seed for the random number engine
                //                 std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
                //                 std::cout << "will check " << individuals.size() << "individuals in random order" << std::endl;
                // #ifdef EVAL_ENVIR
                //                 int damage = NULL; // will be ignored in eval_single_envir
                //                 for (size_t world = 0; world < global::world_options.size(); ++world)
                //                 {
                //                     os << "ENVIR \t" << world << std::endl;
                // #else
                //                 size_t world = 0; // normal environment
                //                 for (size_t damage = 0; damage < global::damage_sets.size(); ++damage)
                //                 {
                //                     os << "DAMAGE \t" << damage << std::endl;
                // #endif

                //                     std::vector<boost::shared_ptr<Phen>> ids_left = individuals;
                //                     float best_so_far = -std::numeric_limits<float>::infinity();
                //                     while (ids_left.size() > 0)
                //                     {
                //                         //random choice
                //                         std::uniform_int_distribution<> dis(0, ids_left.size() - 1);
                //                         int index = dis(gen);
                //                         float val = sferes::fit::RecoveredPerformance<Phen>::_eval_single_envir(*ids_left[index], world, damage);
                //                         if (val > best_so_far)
                //                         {
                //                             best_so_far = val;
                //                         }
                //                         // remove index
                //                         ids_left.erase(ids_left.begin() + index);
                //                         os << best_so_far << std::endl;
                //                     }
                //                 }
            }
            static void test_recoveredperformance(std::ostream &os, const boost::multi_array<boost::shared_ptr<Phen>, BottomParams::ea::behav_dim> &archive)
            {
                for (size_t damage_option = 0 ;  damage_option < global::damage_sets.size(); ++damage_option)
                {
                    std::set<int> unique_bins;
                    for (const bottom_indiv_t *k = archive.data(); k < (archive.data() + archive.num_elements()); ++k)
                    {
                        if (*k)
                        {
                            int index = sferes::fit::RecoveredPerformance<Phen>::_eval_single_envir(**k, damage_option);
                            unique_bins.insert(index);
                        }
                    }
                    os << unique_bins.size() << std::endl;
                }

                os << "END TEST META-INDIVIDUAL" << std::endl;
            }
            // assess maximal recovery for each damage separately
            static void test_max_recovery(std::ostream &os, const boost::multi_array<boost::shared_ptr<Phen>, BottomParams::ea::behav_dim> &archive)
            {

                std::vector<bottom_indiv_t> individuals;
                for (const bottom_indiv_t *k = archive.data(); k < (archive.data() + archive.num_elements()); ++k)
                {
                    if (*k)
                    {
                        individuals.push_back(*k);
                    }
                }
                sferes::fit::RecoveredPerformance<Phen>::_eval_taskmax(os, individuals);
            }

	    static void test_recoveredperformance(std::ostream &os, const std::vector<boost::shared_ptr<Phen>> &archive)
            {
                for (size_t damage_option = 0 ;  damage_option < global::damage_sets.size(); ++damage_option)
                {
                    std::set<int> unique_bins;
                    for (size_t k = 0; k < archive.size(); ++k)
                    {
                       int index = sferes::fit::RecoveredPerformance<Phen>::_eval_single_envir(*archive[k], damage_option);
                       unique_bins.insert(index);

                    }
                    os << unique_bins.size() << std::endl;
                }
            }
            // assess maximal recovery for each damage separately
            static void test_max_recovery(std::ostream &os, const std::vector<boost::shared_ptr<Phen>> &archive)
            {
                sferes::fit::RecoveredPerformance<Phen>::_eval_taskmax(os, archive);
            }
        };

    } // namespace fit
} // namespace sferes

#endif
