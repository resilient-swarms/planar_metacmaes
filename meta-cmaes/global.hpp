#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <planar_dart/safety_measures.hpp>
#include <planar_dart/descriptors.hpp>
#include <planar_dart/planar.hpp>
#include <meta-cmaes/feature_vector_typedefs.hpp>
#include <meta-cmaes/feature_map.hpp>

#if META()
#include <meta-cmaes/database.hpp>
#include <meta-cmaes/params.hpp>
#endif
#if META() || CMAES_CHECK()
#include <sferes/ea/cmaes_interface.h>
#endif

#define JOINT_SIZE 8

namespace global
{
    class RNG
    {
        std::random_device rd;
        std::mt19937 gen;
        long seed;

    public:
        RNG() {}
        RNG(long s)
        {
            gen = std::mt19937(s);
            seed = s;
        }
        /* int in [0,num-1] */
        int nextInt(size_t num)
        {
            std::uniform_int_distribution<unsigned> distrib(0, num - 1);
            return distrib(gen);
        }
        /* float in [0,1] */
        float nextFloat()
        {
            std::uniform_real_distribution<float> distrib(0.0f, 1.0f);
            return distrib(gen);
        }
    };
    RNG *rng;
    // set the right condition
    enum ConditionType
    {
        meta = 0,
        rand,
        pos,
        pol,
        jpa,
        rjpa,
        as,
        cmaes_check
    } condition;
#ifndef TEST
    void set_condition(const std::string &cond)
    {
        if (cond == "meta")
        {
            condition = ConditionType::meta;
            assert(META());
            assert(BEHAV_DIM == 4);
        }
        else if (cond == "rand")
        {
            condition = ConditionType::rand;
            assert(GLOBAL_WEIGHT());
            assert(BEHAV_DIM == 4);
        }
        else if (cond == "pos")
        {
            condition = ConditionType::pos;
            assert(CONTROL());
            assert(BEHAV_DIM == 2);
        }
        else if (cond == "pol")
        {
            condition = ConditionType::pol;
            assert(CONTROL());
            assert(BEHAV_DIM == 2);
        }
        else if (cond == "jpa")
        {
            condition = ConditionType::jpa;
            assert(CONTROL());
            assert(BEHAV_DIM == 4);
        }
        else if (cond == "rjpa")
        {
            condition = ConditionType::rjpa;
            assert(CONTROL());
            assert(BEHAV_DIM == 4);
        }
        else if (cond == "as")
        {
            condition = ConditionType::as;
            assert(CONTROL());
            assert(BEHAV_DIM == 6);
        }
        else
        {
            throw std::runtime_error("condition " + cond + " not known");
        }
    }
#endif
#if CMAES_CHECK()
    size_t damage_index;
#endif
    size_t nb_evals = 0;
    std::shared_ptr<planar_dart::planar> global_robot;

#if DAMAGE_TESTS() || ENVIR_TESTS()
    //std::vector<std::shared_ptr<planar_dart::planar>> damaged_robots;
    std::vector<std::vector<planar_dart::planarDamage>> damage_sets;
    void init_damage(std::string seed, std::string robot_file)
    {
        rng = new RNG((long) stoi(seed));
        std::string damage_type = "stuck";
        std::seed_seq seed2(seed.begin(), seed.end());
        std::mt19937 gen(seed2);
        std::cout << "damage sets :" << std::endl;
        std::ofstream ofs("damage_sets_" + seed + ".txt");

        for (size_t leg = 0; leg < JOINT_SIZE; ++leg)
        {
            std::cout << damage_type << "+ ," << leg << "\n";
            std::cout << damage_type << "- ," << leg << "\n";
            ofs << damage_type << "+ ," << leg << "\n";
            ofs << damage_type << "- ," << leg << "\n";
#ifndef TAKE_COMPLEMENT
            // push back twice to later add both pos and negative angle
            std::string data = std::to_string(leg);
            damage_sets.push_back({planar_dart::planarDamage(damage_type, data)});
            damage_sets.push_back({planar_dart::planarDamage(damage_type, data)});
#endif
        }
        std::cout << std::endl;
#ifdef TAKE_COMPLEMENT
        damage_type = "offset";
        ofs << "test:" << std::endl;
        for (size_t leg = 0; leg < JOINT_SIZE; ++leg)
        {
            for(float off = -1.0f; off < 1.05f; off+=0.05)
            {
                if (off == 0)
                {
                    continue;
                }
                std::string dt = damage_type + std::to_string(off);
                std::cout << dt << "," << leg << "\n";
                ofs << dt << "," << leg << "\n";
                std::string data = std::to_string(leg);
                damage_sets.push_back({planar_dart::planarDamage(dt, data)});
            }
        }
        std::cout << std::endl;
#if CMAES_CHECK()
        std::cout << "will do damage " << global::damage_index << ": " << damage_sets[global::damage_index][0].type << ", " << damage_sets[global::damage_index][0].data << std::endl;
#endif
#endif
        // for (size_t i = 0; i < global::damage_sets.size(); ++i)
        // {
        //     global::damaged_robots.push_back(std::make_shared<planar_dart::planar>(robot_file, "planar", false, global::damage_sets[i])); // we repeat this creation process for damages
        // }
    }
#endif
#if GLOBAL_WEIGHT()
    feature_map_t feature_map;
    void init_weight(std::string seed, std::string robot_file)
    {
        std::ofstream ofs("global_weight_" + seed + ".txt");
        feature_map = feature_map_t::random();
        feature_map.print_weights(ofs);
    }
#endif

    void init_simu(std::string seed, std::string robot_file)
    {
        global::global_robot = std::make_shared<planar_dart::planar>(robot_file, "planar", false); // we repeat this creation process for damages
#if DAMAGE_TESTS() || ENVIR_TESTS()
        // damage tests (meta-learning with damages or test for damage recovery)
        init_damage(seed, robot_file);
//#elif ENVIR_TESTS() // recovery tests (meta-learning with environments or test for environment adaptation)
//    init_world(seed, robot_file);
#endif

#if GLOBAL_WEIGHT() // if using a global weight
        init_weight(seed, robot_file);
#endif
    }

#if META() || CMAES_CHECK()
    cmaes_t evo;
#endif

#if META()
    typedef SampledDataEntry data_entry_t;
    //<size_t num_base_features, size_t capacity, typename DataType>
    //typedef CircularBuffer<BottomParams::MAX_DATABASE_SIZE, data_entry_t> database_t;
    typedef BestKPerBin<NUM_BASE_FEATURES, BottomParams::MAX_DATABASE_SIZE, data_entry_t> database_t;
    database_t database;

#endif
} // namespace global

#endif
