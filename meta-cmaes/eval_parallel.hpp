
//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef EVAL_PARALLEL_HPP
#define EVAL_PARALLEL_HPP

#include <cmath>
#include <sferes/eval/eval.hpp>
#include <vector>

#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h> /* For mode constants */

#include <chrono>
#include <ctime>

#include "../meta-cmaes/params.hpp"
#include "../meta-cmaes/feature_vector_typedefs.hpp"

// /* redirect the output streams */
// void redirect(char* jobname, pid_t pid)
// {
//     /* Initialize ARGoS */
//     /* Redirect LOG and std::err to dedicated files to prevent clutter on the screen */
//     std::ofstream cLOGFile(std::string("ARGoS_LOG_" + argos::ToString(pid)).c_str(), std::ios::out);
//     std::cout.DisableColoredOutput();
//     std::cout.GetStream().rdbuf(cLOGFile.rdbuf());
//     std::ofstream cLOGERRFile(std::string("ARGoS_LOGERR_" + argos::ToString(pid)).c_str(), std::ios::out);
//     std::err.DisableColoredOutput();
//     std::err.GetStream().rdbuf(cLOGERRFile.rdbuf());
//     std::cout << "starting "<< jobname << std::endl;// tell which job it is
//     std::err << "starting "<< jobname << std::endl;// tell which job it is
// }

namespace sferes
{
namespace eval
{
int num_memory;
/** Shared memory manager for data exchange between master and slaves */
class CSharedMem
{

public:
  /**
       * Class constructor.
       * @param un_genome_size The size of the genome of an individual.
       * @param un_pop_size The size of the population.
       */
#if META()

  CSharedMem(size_t descriptor_size, size_t base_feature_size) : m_unDescriptorSize(descriptor_size), m_unBasefeatures(base_feature_size)
  {
    init_shared_mem();
  }
#else
  CSharedMem(size_t descriptor_size) : m_unDescriptorSize(descriptor_size)
  {
    init_shared_mem();
  }
#endif
  void init_shared_mem()
  {
    size_t unShareMemSize = get_block_size() * sizeof(float); //m_unPopSize *
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
    died = reinterpret_cast<bool *>(::mmap(NULL, sizeof(bool), protection, visibility, -1, 0));
    if (died == MAP_FAILED)
    {
      ::perror("shared memory, bool");
      exit(1);
    }
  }

  void init_shared_mem_filedescriptor()
  {
    /* Create shared memory area for master-slave communication */
    // m_nSharedMemFD = ::shm_open(SHARED_MEMORY_FILE.c_str(),
    //                      O_RDWR | O_CREAT,
    //                      S_IRUSR | S_IWUSR);
    // if(m_nSharedMemFD < 0) {
    //     ::perror(SHARED_MEMORY_FILE.c_str());
    //     exit(1);
    // }
    // size_t unShareMemSize = m_unPopSize * get_block_size() * sizeof(float);
    //::ftruncate(m_nSharedMemFD, unShareMemSize);
    /* Get pointer to shared memory area */
    // m_pfSharedMem = reinterpret_cast<float*>(
    //     ::mmap(NULL,
    //           unShareMemSize,
    //           PROT_READ | PROT_WRITE,
    //           MAP_SHARED,
    //           m_nSharedMemFD,
    //           0));
    // if(m_pfSharedMem == MAP_FAILED) {
    //     ::perror("shared memory");
    //     exit(1);
    // }
  }
  /**
       * Class destructor.
       */
  ~CSharedMem()
  {
    munmap(m_pfSharedMem, get_block_size() * sizeof(float));
  }

  /* get the memory block size */

  inline size_t get_block_size()
  {
#if META()
    return (m_unDescriptorSize + m_unBasefeatures + 1);
#else
    return (m_unDescriptorSize + 1);
#endif
  }
  /**
       * Returns the score of an individual.
       * @param un_individual The individual.
       */
  inline float getFitness()
  {
    return m_pfSharedMem[0];
  }
  /**
       * Returns the descriptor of an individual.
       * @param un_individual The individual.
       */
  inline std::vector<float> getDescriptor()
  {
    float *descriptor = &m_pfSharedMem[1]; // pointer to the descriptor
    std::vector<float> bd;
    bd.resize(m_unDescriptorSize);
    for (int i = 0; i < m_unDescriptorSize; ++i)
    {
      bd[i] = descriptor[i];
    }
    return bd;
  }

  inline base_features_t getBaseDescriptor()
  {
    float *descriptor = &m_pfSharedMem[m_unDescriptorSize + 1]; // pointer to the descriptor
    base_features_t b;
    for (int i = 0; i < m_unBasefeatures; ++i)
    {
      b(i, 0) = descriptor[i];
    }
    return b;
  }

  inline bool getDeath()
  {
    return died[0];
  }
  /**
       * Sets the score of an individual.
       * @param un_individual The individual.
       * @param f_score The score.
       */
  inline void setFitness(float f_score)
  {
    /* fitness is copied to the 0'th index of each block */
    ::memcpy(m_pfSharedMem + 0,
             &f_score,
             sizeof(float));
  }

  /**
       * Sets the descriptor of an individual.
       * @param un_individual The individual.
       * @param desc The descriptor
       */
  inline void setDescriptor(std::vector<float> desc)
  {
    /* descriptor is copied to the 1:m_unDescriptorSize'th index of each block */
    for (int i = 0; i < desc.size(); ++i)
    {
      ::memcpy(m_pfSharedMem + (i + 1),
               &desc[i],
               sizeof(float));
    }
  }

  /**
       * Sets the descriptor of an individual.
       * @param un_individual The individual.
       * @param desc The descriptor
       */
  inline void setBaseFeatures(base_features_t b)
  {

    /* descriptor is copied to the 1:m_unDescriptorSize'th index of each block */

    for (int i = 0; i < m_unBasefeatures; ++i)
    {

      ::memcpy(m_pfSharedMem + (m_unDescriptorSize + i + 1),
               &b(i, 0),
               sizeof(float));
    }
  }

  inline void setDeath(bool dead)
  {
    /* death is copied to the final index of each block */
    ::memcpy(died + 0,
             &dead,
             sizeof(bool));
  }

private:
  /** Descriptor size */
  size_t m_unDescriptorSize;

  size_t m_unBasefeatures;
  // /** Population size */
  // size_t m_unPopSize;

  /** File descriptor for shared memory area */
  //int m_nSharedMemFD;

  /** Pointer to the shared memory area */
  float *m_pfSharedMem;
  bool *died;
};
/** The shared memory manager */
static std::vector<CSharedMem *> shared_memory;

template <typename Phen, typename Fit>
struct _eval_parallel_individuals
{
  typedef std::vector<boost::shared_ptr<Phen>> pop_t;
  pop_t _pop;
  std::vector<float> bd;
  /** PID of the master process */
  pid_t MasterPID;

  /** PIDs of the slave processes */
  std::vector<pid_t> SlavePIDs;

  ~_eval_parallel_individuals(){};

  _eval_parallel_individuals() : MasterPID(::getpid())
  {
  }

  _eval_parallel_individuals(pop_t &pop) : _pop(pop),
                                           MasterPID(::getpid())
  {
    run();
  }
  _eval_parallel_individuals(const _eval_parallel_individuals &ev) : _pop(ev._pop),
                                                                     MasterPID(::getpid())
  {
    run();
  }
  void run()
  {
    allocate_additional_memory(this->_pop.size());
    create_processes();
    destroy_additional_memory();
  }
  void run(pop_t &pop)
  {
    this->_pop = pop;
    allocate_additional_memory(this->_pop.size());
    create_processes();
    destroy_additional_memory();
  }
  void allocate_additional_memory(int num_pop)
  {
    int to_add = num_pop - num_memory; //need to use integer not size_t, can be negative

    for (int i = 0; i < to_add; ++i)
    {
#if META()
      shared_memory.push_back(new CSharedMem(BottomParams::ea::behav_dim, NUM_BASE_FEATURES));
#else
      shared_memory.push_back(new CSharedMem(BottomParams::ea::behav_dim));
#endif
    }
    //std::cout<<"allocated memory: "<<shared_memory.size()<<std::endl;// this should happen only at the 0'th generation
  }

  void destroy_additional_memory()
  {

    shared_memory.erase(shared_memory.begin() + num_memory, shared_memory.end());
    //std::cout<<"erased memory: "<<shared_memory.size()<<std::endl;// this should happen only at the 0'th generation
  }
  inline void quit()
  {
    //std::cout.Flush();
    //std::err.Flush();
    exit(EXIT_SUCCESS);
  }

  virtual void LaunchSlave(size_t slave_id)
  {

    //pid_t slave_pid = SlavePIDs[slave_id];
    /* Install handler for SIGTERM */
    //::signal(SIGTERM, SlaveHandleSIGTERM);

    // initialise the fitness function and the genotype
    this->_pop[slave_id]->develop();
    assert(slave_id < this->_pop.size());
    // evaluate the individual
    this->_pop[slave_id]->fit().eval(*this->_pop[slave_id]);

    assert(!std::isnan(this->_pop[slave_id]->fit().value())); // ASSUMES SINGLE OBJECTIVE
    // write fitness and descriptors to shared memory
    shared_memory[slave_id]->setFitness(this->_pop[slave_id]->fit().value()); // ASSUME SINGLE OBJECTIVE
    bd = this->_pop[slave_id]->fit().desc();
    shared_memory[slave_id]->setDescriptor(bd);
    shared_memory[slave_id]->setDeath(this->_pop[slave_id]->fit().dead());
#if META()
    shared_memory[slave_id]->setBaseFeatures(this->_pop[slave_id]->fit().b());
#ifdef CHECK_PARALLEL
    std::cout << " child base descriptor " << this->_pop[slave_id]->fit().b() << std::endl;
    std::cout << " child base descriptor (shared mem)" << shared_memory[slave_id]->getBaseDescriptor() << std::endl;
#endif
#endif
#ifdef CHECK_PARALLEL
    std::cout << "child fitness " << slave_id << " " << this->_pop[slave_id]->fit().value() << std::endl;
    std::cout << "child: descriptor for individual " << slave_id << std::endl;

    for (size_t j = 0; j < this->_pop[slave_id]->fit().desc().size(); ++j)
    {
      std::cout << "   " << this->_pop[slave_id]->fit().desc()[j] << std::endl;
    }

    std::cout << "child: descriptor (sharedmem) for individual " << slave_id << std::endl;
    std::vector<float> temp = shared_memory[slave_id]->getDescriptor();
    for (size_t j = 0; j < temp.size(); ++j)
    {
      std::cout << "   " << temp[j] << std::endl;
    }
    std::cout << "child: death " << this->_pop[slave_id]->fit().dead() << std::endl;
#endif
    quit();
  }

  virtual void write_data()
  {
    /* Back in the parent, copy the scores into the population data */
    for (size_t i = 0; i < this->_pop.size(); ++i)
    {
      this->_pop[i]->fit().set_fitness(shared_memory[i]->getFitness());
#if META()
      this->_pop[i]->fit().set_b(shared_memory[i]->getBaseDescriptor());
#endif
      bd = shared_memory[i]->getDescriptor();
      this->_pop[i]->fit().set_desc(bd);
      this->_pop[i]->fit().set_dead(shared_memory[i]->getDeath());

#if META()
      Fit::add_to_database(*this->_pop[i]);
#ifdef CHECK_PARALLEL
      std::cout << " parent base descriptor " << this->_pop[i]->fit().b() << std::endl;
#endif
#endif
#ifdef CHECK_PARALLEL
      std::cout << "parent fitness " << i << " " << this->_pop[i]->fit().value() << std::endl;
      std::cout << "parent: descriptor for individual " << i << std::endl;
      for (size_t j = 0; j < this->_pop[i]->fit().desc().size(); ++j)
      {
        std::cout << "   " << this->_pop[i]->fit().desc()[j] << std::endl;
      }
      std::cout << "parent: death " << this->_pop[i]->fit().dead() << std::endl;
#endif
    }
  }

  void wait_and_erase()
  {
    // wait for a new process to finish and erase it from the list
    int *status;
    pid_t pid = waitpid(-1, status, 0); // -1: any child; status; 0: only children that exit
    //std::cout<<"waited for pid "<<pid<<std::endl;
    auto it = std::find(SlavePIDs.begin(), SlavePIDs.end(), pid);
    SlavePIDs.erase(it); // remove the process from the list
  }

  /* create the different child processes */
  void create_processes()
  {

    /* Create and run slave processes until all individuals done; but never start more then NUM_CORES processes */
    size_t i = 0;
    while (i < this->_pop.size())
    {
      //std::cout<<"count "<< i << std::endl;

      if (SlavePIDs.size() < NUM_CORES) // still need more
      {
        /* Perform fork */
        SlavePIDs.push_back(::fork());
        if (SlavePIDs.back() == 0)
        {
          /* We're in a slave */
          this->LaunchSlave(i);
        }
        else
        {
          ++i; // increment the parent's index
        }
      }
      else
      {
        wait_and_erase();
      }
    }
    // now wait for all the other child processes to finish
    while (!SlavePIDs.empty())
    {
      wait_and_erase();
    }; // keep performing waits until an error returns, meaning no more child existing

    write_data(); //write data once finished
    //std::cout.Flush();
    //std::err.Flush();
    //std::cout << "finished all processes "<< std::endl;
  }
};

#if CONTROL()

SFERES_EVAL(EvalParallelIndividuals, Eval){
  public :
      template <typename Phen>
      void eval(std::vector<boost::shared_ptr<Phen>> & pop, size_t begin, size_t end,
                const typename Phen::fit_t &fit_proto){
          dbg::trace trace("eval", DBG_HERE);
assert(pop.size());
assert(begin < pop.size());
assert(end <= pop.size());
#ifdef ANALYSIS
throw std::runtime_error("cannot use parallel while doing analysis");
#endif
/* if you want timer */
//auto t1 = std::chrono::system_clock::now();
/* initialise the fitmap */
for (size_t i = begin; i < end; ++i)
{
  pop[i]->fit() = fit_proto;
}
auto helper = _eval_parallel_individuals<Phen, typename Phen::fit_t>();
helper._pop = pop;
helper.run();
this->_nb_evals += (end - begin);
}
};

#endif
// Some computation here
//auto t2 = std::chrono::system_clock::now();
//std::chrono::duration<double> duration =t2 - t1;
//std::cout <<"evaluation time: "<< duration.count() <<'\n';
//std::cout.Flush()

void init_shared_mem()
{
  // times 2 because two individuals generated per reproduction
  num_memory = 2 * BottomParams::pop::init_size; // for the initial population, allocate on the spot
  shared_memory.clear();
  for (size_t i = 0; i < num_memory; ++i)
  {
#if META()
    shared_memory.push_back(new sferes::eval::CSharedMem(BottomParams::ea::behav_dim, NUM_BASE_FEATURES));
#else
    shared_memory.push_back(new sferes::eval::CSharedMem(BottomParams::ea::behav_dim));
#endif
  }
}
} // namespace eval
} // namespace sferes

#endif
