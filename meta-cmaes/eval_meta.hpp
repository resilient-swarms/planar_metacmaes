#ifndef EVAL_META_HPP
#define EVAL_META_HPP

#include <cmath>
#include <sferes/eval/eval.hpp>
#include <vector>
#include <meta-cmaes/feature_vector_typedefs.hpp>
#include <meta-cmaes/bottom_typedefs.hpp>
#include <meta-cmaes/mapelites_phenotype.hpp>
#include <meta-cmaes/eval_parallel.hpp>
#include <meta-cmaes/pairwisedist.hpp>
#include <Eigen/Dense>

//#define CHECK_PARALLEL

namespace sferes
{
    namespace eval
    {
        std::vector<planar_dart::planarDamage> get_damageset(size_t j)
        {
            std::string damage_type = global::damage_sets[j][0].type;
            float angle;
            if (j % 2 == 0)
            {

                angle = 0.50f + 0.50f * global::rng->nextFloat(); // generate positive angle ( in [0.50,1] before conversion)
            }
            else
            {
                angle = 0.50f * global::rng->nextFloat(); // generate negative angle ( in [0,0.50] before conversion)
            }
            damage_type += std::to_string(angle);
            std::string leg = global::damage_sets[j][0].data;
            //std::cout << "damage " << damage_type << " to leg " << leg << std::endl;
            planar_dart::planarDamage damage(damage_type, leg);
            return {damage};
        }
        template <typename Phen>
        struct _eval_serial_meta
        {
            size_t nb_evals;
            float value;
            _eval_serial_meta(Phen &meta_i, float percent)
            {
                std::vector<boost::shared_ptr<base_phen_t>> pop = meta_i.sample_individuals(percent);
                float fit = 0.0f;
#ifdef NO_DAMAGE_METAFIT

                std::vector<planar_dart::planarDamage> damages = {};
#else
                for (size_t j = 0; j < global::damage_sets.size(); ++j)
                {
                    std::vector<planar_dart::planarDamage> damages = get_damageset(j);
#endif
                    std::vector<Eigen::VectorXd> positions;
                    for (size_t i = 0; i < pop.size(); ++i)
                    {
                        // evaluate the individual
                        std::tuple<Eigen::VectorXd, bool> tup = sferes::fit::PairwiseDist<base_phen_t>::_eval_single_envir(*pop[i], damages);
                        bool dead = std::get<1>(tup);
                        if (!dead)
                        {
                            positions.push_back(std::get<0>(tup));
                        }
                        // use dead information for database?
                    }
                    fit += sferes::fit::PairwiseDist<base_phen_t>::compute(positions);
#ifdef NO_DAMAGE_METAFIT
                    value = fit;
                    nb_evals = pop.size();
#else
                }
                /* result */
                value = fit / (float)global::damage_sets.size();
                nb_evals = global::damage_sets.size() * pop.size();
#endif
            }
            }; // namespace eval

            template <typename Phen>
            struct _eval_parallel_meta : public _eval_parallel_individuals<CSharedMemPosition, base_phen_t, bottom_fit_t>
            {
                float value;
                size_t nb_evals;
                Phen meta_indiv;
                std::vector<planar_dart::planarDamage> damages;
                _eval_parallel_meta(Phen &meta_i, float percent)
                { //now join the bottom-level fitnesses
                    this->_pop = meta_i.sample_individuals();
                    value = 0.0f;
#ifdef NO_DAMAGE_METAFIT
                    damages = {};
                    this->run();
                    /* result */
                    nb_evals = this->_pop.size();
#else
                    for (size_t j = 0; j < global::damage_sets.size(); j += 2)
                    {
                        damages = get_damageset(j);
                        this->run();
                    }
                    /* result */
                    value = value / (float)global::damage_sets.size();
                    nb_evals = global::damage_sets.size() * this->_pop.size();
#endif
                }
                virtual void LaunchSlave(size_t slave_id)
                {
                    // we sample directly from map of developed individuals, so the individuals do not need to be developed, nor does their fitness prototype be set
                    std::tuple<Eigen::VectorXd, bool> tup = sferes::fit::PairwiseDist<base_phen_t>::_eval_single_envir(*this->_pop[slave_id], damages);
                    // write fitness and death value (no descriptors) to shared memory
                    dynamic_cast<CSharedMemPosition *>(shared_memory[slave_id])->setPosition(std::get<0>(tup)); // ASSUME SINGLE OBJECTIVE
                    dynamic_cast<CSharedMemPosition *>(shared_memory[slave_id])->setDeath(std::get<1>(tup));    // no need

#ifdef CHECK_PARALLEL
                    std::cout << "child position " << slave_id << " " << std::get<0>(tup) << std::endl;
                    std::cout << "child death " << slave_id << " " << std::get<1>(tup) << std::endl;
#endif
                    this->quit();
                }

                virtual void write_data()
                {
                    std::vector<Eigen::VectorXd> positions;
                    /* Back in the parent, copy the scores into the population data */
                    for (size_t i = 0; i < this->_pop.size(); ++i)
                    {

                        bool dead = shared_memory[i]->getDeath();
                        Eigen::VectorXd pos = dynamic_cast<CSharedMemPosition *>(shared_memory[i])->getPosition();
                        if (!dead)
                        {
                            positions.push_back(pos);
                        }

                        //Fit::add_metaeval_to_database(*this->_pop[i]);

#ifdef CHECK_PARALLEL

                        std::cout << "parent position " << i << " " << pos << std::endl;
                        std::cout << "parent death " << i << " " << dead << std::endl;
#endif
                    }

                    value += sferes::fit::PairwiseDist<base_phen_t>::compute(positions);
                }
            };
        } // namespace sferes
    }     // namespace eval

#endif
