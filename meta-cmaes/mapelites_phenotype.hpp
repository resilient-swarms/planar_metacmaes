

#ifndef MAPELITES_PHENOTYPE_HPP
#define MAPELITES_PHENOTYPE_HPP

#include "../meta-cmaes/params.hpp"

#include "../meta-cmaes/feature_vector_typedefs.hpp"
#include "../meta-cmaes/bottom_typedefs.hpp"
#include "../meta-cmaes/fit_bottom.hpp"
#include "../meta-cmaes/global.hpp"
#include "../meta-cmaes/eval_parallel.hpp"

// now that FitBottom is defined, define the rest of the bottom level
typedef sferes::fit::FitBottom bottom_fit_t;
typedef sferes::phen::Parameters<bottom_gen_t, bottom_fit_t, BottomParams> base_phen_t;
typedef boost::shared_ptr<base_phen_t> bottom_indiv_t;

template <typename Phen>
struct _eval_serial_individuals
{
  typedef std::vector<boost::shared_ptr<Phen>> pop_t;
  pop_t _pop;
  _eval_serial_individuals()
  {
  }

  void run(std::vector<boost::shared_ptr<Phen>> &p)
  {
    this->_pop = p;
    for (size_t i = 0; i < _pop.size(); ++i)
    {
      _pop[i]->fit().eval(*_pop[i]);
#if META()
      bottom_fit_t::add_to_database<Phen>(*_pop[i]);
#endif
    }
  }
};

#ifdef PARALLEL_RUN
typedef sferes::eval::_eval_parallel_individuals<base_phen_t,bottom_fit_t> bottom_eval_helper_t;
#else
typedef _eval_serial_individuals<base_phen_t> bottom_eval_helper_t;
#endif
class EvalIndividuals
{
public:
  EvalIndividuals() : nb_evals(0) {}
  unsigned nb_evals;
  template <typename Phen>
  void eval(std::vector<boost::shared_ptr<Phen>> &p)
  {
    //dbg::trace trace("eval", DBG_HERE);
    assert(p.size());
    float value = 0.0f;
    auto helper = bottom_eval_helper_t();
    helper.run(p);
    nb_evals += p.size(); //
#ifdef PRINTING
    std::cout << "number of evaluations is now " << _nb_evals << std::endl;
#endif
  }

  

protected:
  
};

typedef EvalIndividuals bottom_eval_t;

class MapElites // essentially a copy of map-elites (to avoid SFERES_EA) + some additional functionalities to support the top-level of meta-cmaes
{

public:
  typedef typename std::vector<bottom_indiv_t> bottom_pop_t;
  typedef typename bottom_pop_t::iterator bottom_it_t;
  typedef typename std::vector<std::vector<bottom_indiv_t>> bottom_front_t;
  typedef boost::shared_ptr<base_phen_t> bottom_phen_ptr_t;
  static const size_t behav_dim = BottomParams::ea::behav_dim;

  typedef std::array<float, behav_dim> point_t;
  typedef boost::multi_array<bottom_phen_ptr_t, behav_dim> array_t;
  typedef std::array<typename array_t::index, behav_dim> behav_index_t;
  behav_index_t behav_shape;
  bottom_pop_t _pop; // current batch
  weight_t W; //characteristic weight matrix of this map
  bottom_eval_t eval_individuals;//
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // note not needed when we use NoAlign
  MapElites()
  {
    assert(behav_dim == BottomParams::ea::behav_shape_size());
    for (size_t i = 0; i < BottomParams::ea::behav_shape_size(); ++i)
      behav_shape[i] = BottomParams::ea::behav_shape(i);
    _array.resize(behav_shape);
  }

  void random_pop()
  {
    // do not need it; random pop comes from meta-cmaes
  }

  // for resuming
  void _set_pop(const std::vector<boost::shared_ptr<base_phen_t>> &pop)
  {
#ifdef PRINTING

    std::cout << "setting pop " << std::endl;
#endif
    assert(!pop.empty());

    //        std::cout << this->res_dir() << " " << this->gen() << std::endl;

    //        std::string fname = ea.res_dir() + "/archive_" +
    //                boost::lexical_cast<std::string>(ea.gen()) +
    //                std::string(".dat");

    for (size_t h = 0; h < pop.size(); ++h)
    {
      //            std::cout << "Fitness of ind " << h << " is " << pop[h]->fit().value() << std::endl;
      //            std::cout << "Descriptor is " ; //<< pop[h]->fit().desc()[0] << std::endl;
      //            for (size_t desc_index = 0; desc_index < pop[h]->fit().desc().size(); ++desc_index)
      //                std::cout << pop[h]->fit().desc()[desc_index] << " ";
      //            std::cout << std::endl;

      //            pop[h]->develop();
      //            pop[h]->fit().eval(*pop[h]);  // we need to evaluate the individuals to get the descriptor values

      point_t p = get_point(pop[h]);

      behav_index_t behav_pos;
      for (size_t i = 0; i < BottomParams::ea::behav_shape_size(); ++i)
      {
        behav_pos[i] = round(p[i] * behav_shape[i]);
        behav_pos[i] = std::min(behav_pos[i], behav_shape[i] - 1);
        assert(behav_pos[i] < behav_shape[i]);
      }
      _array(behav_pos) = pop[h];
    }
  }

  void epoch()
  {
    this->_pop.clear();

    for (const bottom_phen_ptr_t *i = _array.data(); i < (_array.data() + _array.num_elements()); ++i)
      if (*i)
        this->_pop.push_back(*i);
#ifdef PRINTING
    std::cout << "start map-elites epoch with pop of " << this->_pop.size() << " individuals " << std::endl;
    std::cout << "start map-elites epoch with array of " << this->_array.size() << " individuals " << std::endl;
#endif
    bottom_pop_t ptmp;
    for (size_t i = 0; i < BottomParams::pop::size; ++i)
    {
      bottom_indiv_t p1 = _selection(this->_pop);
      bottom_indiv_t p2 = _selection(this->_pop);
      boost::shared_ptr<base_phen_t> i1, i2;
      p1->cross(p2, i1, i2);
      i1->mutate();
      i2->mutate();
#ifdef PRINTING

      std::cout << "will develop individual " << 2 * i << std::endl;
#endif
      i1->develop();
      i1->fit() = bottom_fit_t(this->W);
#ifdef PRINTING

      std::cout << "will develop individual " << 2 * i + 1 << std::endl;
#endif
      i2->develop();
      i2->fit() = bottom_fit_t(this->W);

      ptmp.push_back(i1);
      ptmp.push_back(i2);
    }

    eval_individuals.eval<base_phen_t>(ptmp);

    for (size_t i = 0; i < ptmp.size(); ++i)
    {
      _add_to_archive(ptmp[i]);
    }
  }

  long int getindex(const array_t &m, const bottom_phen_ptr_t *requestedElement, const unsigned short int direction) const
  {
    int offset = requestedElement - m.origin();
    return (offset / m.strides()[direction] % m.shape()[direction] + m.index_bases()[direction]);
  }

  behav_index_t getindexarray(const array_t &m, const bottom_phen_ptr_t *requestedElement) const
  {
    behav_index_t _index;
    for (unsigned int dir = 0; dir < behav_dim; dir++)
    {
      _index[dir] = getindex(m, requestedElement, dir);
    }

    return _index;
  }

  const array_t &archive() const
  {
    return _array;
  }

  template <typename I>
  point_t get_point(const I &indiv) const
  {
    return _get_point(indiv);
  }

  bottom_indiv_t _selection(const bottom_pop_t &pop)
  {
    int x1 = misc::rand<int>(0, pop.size());
    return pop[x1];
  }

  // void eval_individuals()
  // {

  //   ind->fit() = bottom_fit_t(W);
  //   ind->develop();
  //   ind->fit().eval<base_phen_t>(*ind);
  //   ++global::nb_evals;
  // }

  void do_epochs(size_t num_epochs)
  {
    eval_individuals.nb_evals = 0;
    for (size_t i = 0; i < num_epochs; ++i)
    {
      this->epoch();
    }
  }

  bool _add_to_archive(bottom_indiv_t &i1)
  {
    if (i1->fit().dead())
    {
#ifdef PRINTING
      std::cout << "dead" << std::endl;

#endif
      return false;
    }

    point_t p = _get_point(i1);

    behav_index_t behav_pos;
    for (size_t i = 0; i < BottomParams::ea::behav_shape_size(); ++i)
    {
      behav_pos[i] = round(p[i] * behav_shape[i]);
      behav_pos[i] = std::min(behav_pos[i], behav_shape[i] - 1);
      assert(behav_pos[i] < behav_shape[i]);
#ifdef PRINTING
      std::cout << "b" << i << " " << behav_pos[i] << std::endl;
#endif
    }
#ifdef PRINTING
    std::cout << "checkingg" << std::endl;

#endif
    if (!_array(behav_pos) || (i1->fit().value() - _array(behav_pos)->fit().value()) > BottomParams::ea::epsilon || (fabs(i1->fit().value() - _array(behav_pos)->fit().value()) <= BottomParams::ea::epsilon && _dist_center(i1) < _dist_center(_array(behav_pos))))
    {
#ifdef PRINTING
      std::cout << "inserting" << std::endl;

#endif
      _array(behav_pos) = i1;
      _non_empty_indices.insert(behav_pos);
#ifdef PRINTING
      std::cout << "inserted" << std::endl;

#endif
      return true;
    }
    return false;
  }
  std::vector<bottom_indiv_t> sample_individuals()
  {
    size_t num_individuals = std::max(1, (int)std::round(CMAESParams::pop::percentage_evaluated * _non_empty_indices.size()));
    return _pick(_non_empty_indices.size(), num_individuals);
  }

protected:
  array_t _array;
  //array_t _prev_array;
  std::set<behav_index_t> _non_empty_indices; // all non-empty solutions' indices stored here

  template <typename T>
  T _get_Nth_Element(std::set<T> &searchSet, int n)
  {
    return *(std::next(searchSet.begin(), n));
  }
  std::vector<bottom_indiv_t> _pick(size_t N, size_t k)
  {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::set<size_t> elems = global::_pickSet(N, k, gen);

    std::vector<bottom_indiv_t> result;
    for (size_t el : elems)
    {
      behav_index_t pos = _get_Nth_Element<behav_index_t>(_non_empty_indices, el);
#ifdef PRINTING
      std::cout << "chosen position: ";
      for (int i = 0; i < behav_dim; ++i)
      {
        std::cout << pos[i] << ",";
      }
      std::cout << "\n";
#endif
      result.push_back(_array(pos));
    }
    return result;
  }

  template <typename I>
  float _dist_center(const I &indiv)
  {
    /* Returns distance to center of behavior descriptor cell */
    float dist = 0.0;
    point_t p = _get_point(indiv);
    for (size_t i = 0; i < BottomParams::ea::behav_shape_size(); ++i)
      dist += pow(p[i] - (float)round(p[i] * (float)(behav_shape[i] - 1)) / (float)(behav_shape[i] - 1), 2);

    dist = sqrt(dist);
    return dist;
  }

  template <typename I>
  point_t _get_point(const I &indiv) const
  {
    point_t p;
    for (size_t i = 0; i < BottomParams::ea::behav_shape_size(); ++i)
      p[i] = std::min(1.0f, indiv->fit().desc()[i]);

    return p;
  }
};

/* phenotype has members _gen and _fit, the genotype and fitmap, 
                        and is responsible for individuals random init, mutation, development */
template <typename Phen>
class MapElitesPhenotype : public Phen, public MapElites
{
public:
  MapElitesPhenotype(){}; // will create random genotype and no fitness

  /*  set the weights (genotype)    */
  // void set_weights()
  // {
  //   W = this->gen().data(); // get the genotype

  //   // #ifdef PRINTING
  //   //     std::cout << "setting weights (genotype) of the phenotype "<< std::endl;
  //   //     for (float w : weights)
  //   //     {
  //   //       std::cout << w <<",";
  //   //     }
  //   //     std::cout <<" \n " << W << std::endl;
  //   // #endif
  // }
  void genotype_to_mat(const std::vector<float> &weights)
  {
    size_t count = 0;
#ifdef PRINTING
    std::cout << "before conversion " << std::endl;
#endif
    for (size_t j = 0; j < NUM_BOTTOM_FEATURES; ++j)
    {
      float sum = std::accumulate(weights.begin() + j * NUM_BASE_FEATURES, weights.begin() + (j + 1) * NUM_BASE_FEATURES, 0.0);
      for (size_t k = 0; k < NUM_BASE_FEATURES; ++k)
      {
        W(j, k) = weights[count] / sum; // put it available for the MapElites parent class

#ifdef PRINTING
        std::cout << "sum " << sum << std::endl;
        std::cout << weights[count] << std::endl;
        std::cout << W(j, k) << "," << std::endl;
#endif
        ++count;
      }
    }
#ifdef PRINTING
    std::cout << "after conversion " << std::endl;
    std::cout << W << std::endl;
#endif
  }
  void random_pop()
  {
    bottom_pop_t ptmp;
    for (size_t i = 0; i < BottomParams::pop::init_size; ++i)
    {
      boost::shared_ptr<base_phen_t> indiv(new base_phen_t());
      indiv->random();
      indiv->develop();
      ptmp.push_back(indiv);
    }
    eval_individuals.eval<base_phen_t>(ptmp); // note that W is not initialised but we do not care here, just for the database; no need for archive therefore
  }
  void develop()
  {
#ifdef PRINTING
    std::cout << "start developing the map-phenotype" << std::endl;
#endif
    /* do develop of the subclass phenotype*/
    Phen::develop();

    undevelop();
    /* fill map j with individuals */
    genotype_to_mat(this->gen().data());
    for (int i = 0; i < global::database.size(); ++i)
    {
      entry_to_map(global::database[i], W); // obtain behavioural features and put individuals in the map
    }
#ifdef PRINTING
    std::cout << "stop developing the map-phenotype" << std::endl;
#endif
  }
  /* get rid of the previous map at this meta-population index */
  void undevelop()
  {
    this->_array = array_t(behav_shape);//
    //std::cout << "map empty : "<<  this->_non_empty_indices.empty() << std::endl;
    this->_non_empty_indices.clear();
  }

  void entry_to_map(const global::data_entry_t &entry, const weight_t &weight)
  {

    // use weight and base features --> bottom-level features

    // create new individual
    boost::shared_ptr<base_phen_t> individual(new base_phen_t());
    //
    individual->fit() = bottom_fit_t(weight);
    individual->fit().set_desc(individual->fit().get_desc(entry.base_features));
    individual->fit().set_value(entry.fitness);
    entry.set_genotype<boost::shared_ptr<base_phen_t>>(individual);

    this->_add_to_archive(individual);
#ifdef PRINTING
    std::cout << "added individual to archive " << std::endl;
#endif
  }
};

#endif