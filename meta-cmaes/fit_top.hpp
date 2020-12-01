
#ifndef FIT_TOP_HPP_
#define FIT_TOP_HPP_

#include <sferes/fit/fitness.hpp>
#include <meta-cmaes/global.hpp>
#include <meta-cmaes/mapelites_phenotype.hpp>
#include <meta-cmaes/eval_meta.hpp>
#include <meta-cmaes/pairwisedist.hpp>


// typedef

/* bottom-level fitmap 
used to evaluate behavioural descriptor and fitness of controllers in the normal operating environment
*/
namespace sferes
{

namespace fit
{

#if CONTROL()
template <typename Params>
class FitTop : public sferes::fit::Fitness
#else
SFERES_FITNESS(FitTop, sferes::fit::Fitness)
#endif
{
public:
    /* current bottom-level map (new candidate to be added to _pop)*/
    template <typename MetaIndiv>
    void eval(MetaIndiv &indiv)
    {

        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        this->_objs.resize(1);
        std::fill(this->_objs.begin(), this->_objs.end(), 0);
        _eval<MetaIndiv>(indiv);
        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " <<     std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &boost::serialization::make_nvp("_value", this->_value);
        ar &boost::serialization::make_nvp("_objs", this->_objs);
    }

    bool dead() { return false; }

    inline void set_fitness(float fFitness)
    {
#if CONTROL()
        this->_objs.resize(1);
        this->_objs[0] = fFitness;
#endif
        this->_value = fFitness;
    }

    size_t nb_evals()
    {
        return _nb_evals;
    }

protected:
    size_t _nb_evals = 0;
    // descriptor work done here, in this case duty cycle
    template <typename MetaIndiv>
    void _eval(MetaIndiv &meta_indiv)
    {
#ifdef PARALLEL_RUN
        typedef sferes::eval::_eval_parallel_meta<MetaIndiv> top_eval_helper_t;
#else
        typedef sferes::eval::_eval_serial_meta<MetaIndiv> top_eval_helper_t;
#endif
        auto helper = top_eval_helper_t(meta_indiv); //allow parallelisation over individuals (_parallel_eval_meta)
        set_fitness(helper.value);
        _nb_evals = helper.nb_evals;
#ifdef PRINTING
        std::cout << "recovered performance " << this->_value << std::endl;
#endif
    }
};
} // namespace fit
} // namespace sferes
#endif
