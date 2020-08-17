
#ifndef TOP_TYPEDEFS_HPP
#define TOP_TYPEDEFS_HPP

#include <sferes/gen/evo_float.hpp>
#include <meta-cmaes/params.hpp>
#include <meta-cmaes/mapelites_phenotype.hpp>
#include <meta-cmaes/eval_total.hpp>
#include <meta-cmaes/fit_top.hpp>



typedef sferes::gen::EvoFloat<NUM_GENES, CMAESParams> gen_t;



typedef sferes::eval::EvalTotal<CMAESParams> eval_t;
typedef sferes::fit::FitTop<CMAESParams>  fit_t;

 
typedef sferes::phen::Parameters<gen_t, fit_t, CMAESParams> meta_phen_t;
typedef MapElitesPhenotype<meta_phen_t> phen_t; 


#endif 