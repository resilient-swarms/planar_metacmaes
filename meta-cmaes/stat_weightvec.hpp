#ifndef STAT_POP_HPP_
#define STAT_POP_HPP_

#include <numeric>
#include <boost/multi_array.hpp>
#include <boost/serialization/array.hpp>

#include "../meta-cmaes/feature_vector_typedefs.hpp"
#include "../meta-cmaes/global.hpp"
#include <sferes/stat/stat.hpp>

// #define MAP_WRITE_PARENTS

namespace sferes
{
namespace stat
{
SFERES_STAT(Stat_Weightvec, Stat)
{
public:
    weight_t W;
    Stat_Weightvec()
    {
    }
    void set_weight()
    {
        global::W = this->W;
    }
    void show(std::ostream & os, size_t k)
    {
        std::cout << "weight vector of the map is "<< W <<std::endl;
    }
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar &BOOST_SERIALIZATION_NVP(W);
         if (Archive::is_loading::value)
        {
            set_weight();
        }
    }

    template <typename E>
    void refresh(const E &ea)
    {

        if (ea.gen() % CMAESParams::pop::dump_period == 0)
        {
#ifdef PRINTING
            std::cout << "starting dump of Stat_Map" << std::endl;
#endif
            W = ea.W;
           
        }
    }
};
} // namespace stat
} // namespace sferes

#endif