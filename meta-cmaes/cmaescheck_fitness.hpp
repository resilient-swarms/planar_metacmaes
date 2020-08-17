
#ifndef CMAESCHECK_FITNESS_HPP
#define CMAESCHECK_FITNESS_HPP

#include <sferes/fit/fitness.hpp>
#include <meta-cmaes/cmaescheck_typedefs.hpp>
#include <meta-cmaes/recovered_performance.hpp>
/* bottom-level fitmap 
used to evaluate behavioural descriptor and fitness of controllers in the normal operating environment
*/

namespace sferes
{
namespace fit
{

SFERES_FITNESS(FitTaskMax, sferes::fit::Fitness)
{

public:
    FitTaskMax(){};
    inline void set_fitness(float fFitness)
    {
        this->_objs.resize(1);
        this->_objs[0] = fFitness;
        this->_value = fFitness;
    }

    template <typename Indiv>
    void eval(Indiv & indiv)
    {

        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        this->_value = 0;
        _eval<Indiv>(indiv);
        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " <<     std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }

protected:



    // descriptor work done here, in this case duty cycle
    template <typename Indiv>
    void _eval(Indiv & indiv)
    {
        
        float val = sferes::fit::RecoveredPerformance<Indiv>::_eval_single_envir(indiv, global::damage_index);
        set_fitness(val);
    }

};
} // namespace fit
} // namespace sferes




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// TOP   ////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// now that FitBottom is defined, define the rest of the bottom level
typedef sferes::fit::FitTaskMax<CMAESCHECKParams> fit_t;
typedef sferes::phen::Parameters<bottom_gen_t, fit_t, CMAESCHECKParams> phen_t;
typedef boost::shared_ptr<phen_t> bottom_indiv_t;

typedef sferes::eval::Eval<CMAESCHECKParams> eval_t;



#endif

