#ifndef STAT_MAP_HPP_
#define STAT_MAP_HPP_

#include <numeric>
#include <boost/multi_array.hpp>
#include <boost/serialization/array.hpp>
#include <sferes/stat/stat.hpp>

#include <meta-cmaes/params.hpp>
#include <meta-cmaes/global.hpp>
#include <meta-cmaes/recovered_performance.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h> /* For mode constants */

#include <map>

//modifies the stat-map to calculate averaged performance over all individuals

// #define MAP_WRITE_PARENTS

namespace sferes
{
namespace stat
{
#if CONTROL()
SFERES_STAT(Map, Stat)
#else
template <typename Phen, typename Params>
class Map
#endif
{
public:
    typedef boost::shared_ptr<Phen> phen_t;
    typedef boost::multi_array<phen_t, Params::ea::behav_dim> array_t;
    typedef std::array<float, Params::ea::behav_dim> point_t;
    typedef std::array<typename array_t::index, Params::ea::behav_dim> behav_index_t;

    size_t behav_dim;
    behav_index_t behav_shape;
    behav_index_t behav_strides;
    behav_index_t behav_indexbase;
    float *m_pfSharedMem;

    Map() : behav_dim(Params::ea::behav_dim)
    {
        for (size_t i = 0; i < Params::ea::behav_shape_size(); ++i)
            behav_shape[i] = Params::ea::behav_shape(i);
    }

    template <typename E>
    void refresh(const E &ea)
    {
        _archive.clear();

        for (size_t i = 0; i < behav_dim; ++i)
        {
            assert(ea.archive().shape()[i] == behav_shape[i]);
            behav_strides[i] = ea.archive().strides()[i];
            behav_indexbase[i] = ea.archive().index_bases()[i];
        }

        for (const phen_t *i = ea.archive().data(); i < (ea.archive().data() + ea.archive().num_elements()); ++i)
        {
            phen_t p = *i;
            _archive.push_back(p);
        }

        if (ea.gen() % Params::pop::dump_period == 0)
        {
            _write_archive(ea.archive(), std::string("archive_"), ea);
#ifdef MAP_WRITE_PARENTS
            _write_parents(ea.archive(), ea.parents(), std::string("parents_"), ea);
#endif
        }
    }
    void wait_and_erase(std::ostream & os, std::map<size_t, pid_t> & SlavePIDs)
    {
        // wait for a new process to finish and erase it from the list
        int *status;
        pid_t pid = waitpid(-1, status, 0); // -1: any child; status; 0: only children that exit
        // //std::cout<<"waited for pid "<<pid<<std::endl;
        auto it = SlavePIDs.begin();
        while (it != SlavePIDs.end())
        {
            // Check if value of this entry matches with given value
            if (it->second == pid)
            {
                //std::cout << "writing " << it->first << std::endl;
                os << m_pfSharedMem[it->first] << std::endl; // remove the process from the list
                os.flush();
                //std::cout << "erasing " << it->first << "," << std::endl;
                SlavePIDs.erase(it); // remove the process from the list
                break;
            }
            // Go to next entry in map
            it++;
        }
    }
    void show(std::ostream & os, size_t n)
    {
        std::cout << "NUM_CORES " << NUM_CORES << std::endl;
        std::cout << "show stat" << std::endl;
        std::cout << "read the archive" << std::endl;

        std::vector<bottom_indiv_t> individuals;
        for (bottom_indiv_t *k = _archive.data(); k < (_archive.data() + _archive.size()); ++k)
        {
            if (*k)
            {
                individuals.push_back(*k);
            }
        }
#ifdef GRAPHIC // we are just interested in observing a particular individual
        float val = 0.0f;
        size_t count = 0;
        std::cout << "loading individual" << n << std::endl;
        for (bottom_indiv_t indiv : individuals)
        {
            if (count == n)
            {
                float val = sferes::fit::RecoveredPerformance<Phen>::_eval_all(*indiv);
                std::cout << val << std::endl;
                break;
            }
            ++count;

            //std::cout << count;
        }

#else
#ifdef INDIVIDUAL_DAMAGE
        test_max_recovery(os, individuals);
#else
        test_recoveredperformance(os, individuals);
#endif
#endif
    }
    void test_recoveredperformance(std::ostream & os, std::vector<bottom_indiv_t> & individuals)
    {
        float val = 0.0f;
        std::cout << "will do " << individuals.size() << "individuals" << std::endl;
        size_t i = 0;
        size_t unShareMemSize = individuals.size() * sizeof(float); //m_unPopSize *
        std::map<size_t, pid_t> SlavePIDs;
        /* Get pointer to shared memory area */
        // Memory buffer will be readable and writable:
        int protection = PROT_READ | PROT_WRITE;
        // The buffer will be shared (meaning other processes can access it), but
        // anonymous (meaning third-party processes cannot obtain an address for it),
        // so only this process and its children will be able to use it:
        int visibility = MAP_ANONYMOUS | MAP_SHARED;
        m_pfSharedMem = reinterpret_cast<float *>(::mmap(NULL, unShareMemSize, protection, visibility, -1, 0));
        if (m_pfSharedMem == MAP_FAILED)
        {
            ::perror("shared memory, vector float");
            exit(1);
        }
        //os << NUM_CORES << std::endl;
        while (i < individuals.size())
        {

            if (SlavePIDs.size() < NUM_CORES) // still need more
            {
                /* Perform fork */

                SlavePIDs[i] = ::fork();
                if (SlavePIDs[i] == 0)
                {
                    /* We're in a slave */
                    val = sferes::fit::RecoveredPerformance<Phen>::_eval_all(*individuals[i]);
#ifdef EVAL_ENVIR
                    val /= (float)global::world_options.size();
#else
                    val /= (float)global::damage_sets.size();
#endif
                    //os << "child" << k << std::endl;
                    /* fitness is copied to the 0'th index of each block */
                    ::memcpy(m_pfSharedMem + i,
                             &val,
                             sizeof(float));
                    //os << val << std::endl;
                    //os.flush(); //make sure it is already flushed !
                    exit(EXIT_SUCCESS);
                }
                else
                {
                    i++; //continue because you finished one
                }
            }
            else
            {
                this->wait_and_erase(os, SlavePIDs);
            }
        }
        // now wait for all the other child processes to finish
        while (!SlavePIDs.empty())
        {
            wait_and_erase(os, SlavePIDs);
        } // keep performing waits until an error returns, meaning no more child existing
    }

    // assess maximal recovery for each damage separately
    void test_max_recovery(std::ostream & os, std::vector<bottom_indiv_t> & individuals)
    {
        //
        sferes::fit::RecoveredPerformance<Phen>::_eval_taskmax(os,individuals);
    }

    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar &BOOST_SERIALIZATION_NVP(_archive);
        ar &BOOST_SERIALIZATION_NVP(behav_dim);
        ar &BOOST_SERIALIZATION_NVP(behav_shape);
        ar &BOOST_SERIALIZATION_NVP(behav_strides);
        ar &BOOST_SERIALIZATION_NVP(behav_indexbase);
    }

protected:
    std::vector<phen_t> _archive;

    template <typename EA>
    void _write_parents(const array_t &array,
                        const array_t &p_array,
                        const std::string &prefix,
                        const EA &ea) const
    {
        std::cout << "writing..." << prefix << ea.gen() << std::endl;
        std::string fname = ea.res_dir() + "/" + prefix + boost::lexical_cast<std::string>(ea.gen()) + std::string(".dat");
        std::ofstream ofs(fname.c_str());

        for (const phen_t *i = array.data(); i < (array.data() + array.num_elements()); ++i)
        {
            if (*i)
            {
                behav_index_t idx = ea.getindexarray(array, i);
                assert(array(idx)->fit().value() == (*i)->fit().value());
                if (p_array(idx))
                {
                    for (size_t dim = 0; dim < behav_dim; ++dim)
                        ofs << idx[dim] / (float)behav_shape[dim] << " ";
                    ofs << " " << p_array(idx)->fit().value() << " ";

                    point_t p = ea.get_point(p_array(idx));
                    behav_index_t posinparent;
                    for (size_t dim = 0; dim < behav_dim; ++dim)
                    {
                        posinparent[dim] = round(p[dim] * behav_shape[dim]);
                        ofs << posinparent[dim] / (float)behav_shape[dim] << " ";
                    }
                    ofs << " " << array(idx)->fit().value() << std::endl;
                }
            }
        }
    }

    template <typename EA>
    void _write_archive(const array_t &array,
                        const std::string &prefix,
                        const EA &ea) const
    {
        std::cout << "writing..." << prefix << ea.gen() << std::endl;
        std::string fname = ea.res_dir() + "/" + prefix + boost::lexical_cast<std::string>(ea.gen()) + std::string(".dat");

        std::ofstream ofs(fname.c_str());

        size_t offset = 0;
        for (const phen_t *i = array.data(); i < (array.data() + array.num_elements()); ++i)
        {
            if (*i)
            {
                behav_index_t posinarray = ea.getindexarray(array, i);
                assert(array(posinarray)->fit().value() == (*i)->fit().value());

                ofs << offset << "    ";
                for (size_t dim = 0; dim < behav_dim; ++dim)
                    ofs << posinarray[dim] / (float)behav_shape[dim] << " ";
                ofs << " " << array(posinarray)->fit().value() << " ";
                for (size_t k = 0; k < array(posinarray)->gen().size(); ++k)
                    ofs << array(posinarray)->gen().data(k) << " ";
                ofs << std::endl;
            }
            ++offset;
        }
    }
}; // namespace stat
} // namespace stat
} // namespace sferes

#endif
