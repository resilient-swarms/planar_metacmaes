

#ifndef METAMAP_ELITE_HPP_
#define METAMAP_ELITE_HPP_

#include <algorithm>
#include <limits>
#include <array>

#include <boost/foreach.hpp>
#include <boost/multi_array.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/algorithm/query/find.hpp>
#include <boost/fusion/include/find.hpp>

#include <sferes/ea/ea.hpp>


#include "../meta-cmaes/top_typedefs.hpp"

#include "../meta-cmaes/global.hpp"

#include "../meta-cmaes/cmaes.hpp"

#include "../meta-cmaes/stat_maps.hpp"
#include "../meta-cmaes/stat_pop.hpp"
//#include <sferes/ea/cmaes_interface.h> // to access parameter initialisation functions
//#include <meta-cmaes/params.hpp>




namespace sferes
{

namespace ea
{

// Main class
SFERES_EA(MetaCmaes, sferes::ea::Cmaes)
{
public:
        SFERES_EA_FRIEND(MetaCmaes);
        typedef boost::shared_ptr<phen_t> indiv_t;
        typedef typename std::vector<indiv_t> pop_t;

        MetaCmaes()
        {
        }

        void random_pop()
        {
#ifdef PRINTING
                std::cout << "initialise database " << std::endl;
#endif
                boost::shared_ptr<Phen> dummy_map(new Phen()); // create a meaningless map
                dummy_map->random_pop();                       // generate individuals which are then added to the databse

#ifdef PRINTING

                for (size_t k = 0; k < global::database.size(); ++k)
                {
                        std::cout << global::database[k].base_features << "\t" << global::database[k].fitness << std::endl;
                }
                std::cout << "CMAES random pop " << std::endl;
#endif
                this->_pop.resize(CMAESParams::pop::size);
                BOOST_FOREACH (boost::shared_ptr<Phen> &indiv, this->_pop)
                {
                        indiv = boost::shared_ptr<Phen>(new Phen());
                        indiv->random();
                        indiv->develop();
                }
#ifdef PRINTING

                std::cout << "end CMAES random pop with population size " << this->_pop.size() << std::endl;
#endif
        }
        //         void set_database()
        //         {

        //                 auto stat_pop = boost::fusion::find<sferes::stat::Stat_Pop>(this->stat());
        //                 if (stat_pop == boost::fusion::end(this->stat()))
        //                 {
        //                         throw std::runtime_error("could not load the database. did you not use Stat_Pop statistic ?! ");
        //                 }
        //                 else
        //                 {
        //                         stat_pop->set_database();
        //                 }
        //         }

        //         // identical copy of load, except that set_database() is added at the end
        //         void load(const std::string &fname)
        //         {
        //                 dbg::trace trace("ea", DBG_HERE);
        //                 std::cout << "loading " << fname << std::endl;
        //                 std::ifstream ifs(fname.c_str());
        //                 if (ifs.fail())
        //                 {
        //                         std::cerr << "Cannot open :" << fname
        //                                   << "(does file exist ?)" << std::endl;
        //                         exit(1);
        //                 }
        // #ifdef SFERES_XML_WRITE
        //                 typedef boost::archive::xml_iarchive ia_t;
        // #else
        //                 typedef boost::archive::binary_iarchive ia_t;
        // #endif
        //                 ia_t ia(ifs);
        //                 boost::fusion::for_each(this->stat(), ReadStat_f<ia_t>(ia));
        //                 set_database();
        //         }
};
} // namespace ea
} // namespace sferes


typedef boost::fusion::vector<sferes::stat::Stat_Pop<phen_t, CMAESParams>, sferes::stat::Stat_Maps<phen_t, CMAESParams>> stat_t;

typedef modif::Dummy<> modifier_t;
typedef ea::MetaCmaes<phen_t, eval_t, stat_t, modifier_t, CMAESParams> ea_t;
#endif