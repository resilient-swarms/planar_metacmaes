#ifndef EVAL_META_HPP
#define EVAL_META_HPP

#include <cmath>
#include <sferes/eval/eval.hpp>
#include <vector>
#include <meta-cmaes/feature_vector_typedefs.hpp>
#include <meta-cmaes/bottom_typedefs.hpp>
#include <meta-cmaes/mapelites_phenotype.hpp>
#include <meta-cmaes/eval_parallel.hpp>

namespace sferes
{
namespace eval
{
template <typename Phen, typename Fit>
struct _eval_serial_meta
{
      size_t nb_evals;
      float value;
      _eval_serial_meta(Phen &meta_i)
      {
            std::vector<boost::shared_ptr<base_phen_t>> pop = meta_i.sample_individuals();
            value = 0.0f;
            for (size_t i = 0; i < pop.size(); ++i)
            {
                  // evaluate the individual
                  value += Fit::_eval_all(*pop[i]);
            }
            std::tuple<float, int> results = meta_i.fit().avg_value(value, pop.size());
            value = std::get<0>(results);
            nb_evals = std::get<1>(results);
      }
};

template <typename Phen, typename Fit>
struct _eval_parallel_meta : public _eval_parallel_individuals<base_phen_t, bottom_fit_t>
{
      float value;
      size_t nb_evals;
      Phen meta_indiv;
      _eval_parallel_meta(Phen &meta_i)
      { //now join the bottom-level fitnesses
            meta_indiv = meta_i;
            this->_pop = meta_i.sample_individuals();
            this->run();
      }

      virtual void LaunchSlave(size_t slave_id)
      {
            // we sample directly from map of developed individuals, so the individuals do not need to be developed, nor does their fitness prototype be set
            float fitness = Fit::_eval_all(*this->_pop[slave_id]);
            // write fitness and death value (no descriptors) to shared memory
            shared_memory[slave_id]->setFitness(fitness); // ASSUME SINGLE OBJECTIVE
                                                          //shared_memory[slave_id]->setDeath(false);// no need
#ifdef CHECK_PARALLEL
            std::cout << "child fitness " << slave_id << " " << fitness << std::endl;
#endif
            this->quit();
      }

      virtual void write_data()
      {
            value = 0.0;
            /* Back in the parent, copy the scores into the population data */
            for (size_t i = 0; i < this->_pop.size(); ++i)
            {
                  float temp = shared_memory[i]->getFitness();
                  value += temp;
#ifdef CHECK_PARALLEL
                  std::cout << "parent fitness " << i << " " << temp << std::endl;
#endif
            }
            std::tuple<float, size_t> results = meta_indiv.fit().avg_value(value, this->_pop.size());
            value = std::get<0>(results);
            nb_evals = std::get<1>(results);
      }
};

} // namespace eval
} // namespace sferes

#endif