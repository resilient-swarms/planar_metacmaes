#include <boost/shared_ptr.hpp>
#include <sferes/dbg/dbg.hpp>
#include <sferes/stc.hpp>
#include <sferes/eval/eval.hpp>
#include <meta-cmaes/bottom_typedefs.hpp>

// eval class which allows us to:
// 1.  develop the individual maps for a few generations after which they are evaluated by their recovered performance
// 2.  count the total number of function evaluations (including those for the recovered performance calc.)

namespace sferes
{
namespace eval
{

SFERES_EVAL(EvalTotal, Eval)
{
public:
  EvalTotal() {}
  template <typename MetaIndiv>
  void eval(std::vector<boost::shared_ptr<MetaIndiv>> & pop, size_t begin, size_t end,
            const typename MetaIndiv::fit_t &fit_proto)
  {
    //dbg::trace trace("eval", DBG_HERE);
    assert(pop.size());
    assert(begin < pop.size());
    assert(end <= pop.size());
    for (size_t i = begin; i < end; ++i)
    {
#ifdef PRINTING
      std::cout << "running the map for " + std::to_string(BottomParams::bottom_epochs) + " epochs" << std::endl;
      std::cout << typeid(pop[i]).name() << std::endl;
#endif
      //pop[i]->develop();// already developed into a map, see cmaes.hpp
      pop[i]->do_epochs(BottomParams::bottom_epochs);
#ifdef PRINTING
      std::cout << "evaluating meta-individual  " + std::to_string(i) << std::endl;
#endif
      global::nb_evals +=  pop[i]->eval_individuals.nb_evals;
      pop[i]->fit().eval<MetaIndiv>(*pop[i]); // evaluate its recovered performance
      global::nb_evals += pop[i]->fit().nb_evals();
    }
#ifdef PRINTING
      std::cout<< "number of evaluations is now "<<_nb_evals<<std::endl;
#endif
  }
  unsigned nb_evals() const { return _nb_evals; }

protected:
  unsigned _nb_evals;
  
};
} // namespace eval
} // namespace sferes