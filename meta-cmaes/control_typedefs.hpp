#ifndef CONTROL_TYPEDEFS_HPP
#define CONTROL_TYPEDEFS_HPP

#include <sferes/fit/fitness.hpp>
#include <meta-cmaes/params.hpp>
#include <boost/shared_ptr.hpp>
#include <sferes/ea/ea.hpp>


#include <meta-cmaes/fit_bottom.hpp>
#ifdef PARALLEL_RUN
#include <meta-cmaes/eval_parallel.hpp>
#else
#include <sferes/eval/eval.hpp>
#endif



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// TOP   ////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// now that FitBottom is defined, define the rest of the bottom level
typedef sferes::fit::FitBottom<BottomParams> fit_t;
typedef sferes::phen::Parameters<bottom_gen_t, fit_t, BottomParams> phen_t;
typedef boost::shared_ptr<phen_t> bottom_indiv_t;
#ifdef PARALLEL_RUN
    typedef sferes::eval::EvalParallelIndividuals<BottomParams> eval_t;
#else
    typedef sferes::eval::Eval<BottomParams> eval_t;
#endif












#endif