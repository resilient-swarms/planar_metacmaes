
#define NO_SERIALIZATION

#include <meta-cmaes/parameter_controller.hpp>
#include <fstream>

// here we test the rl system with some simple dynamics:

struct RL
{
    size_t num_params;
    RLController controller;
    float mutation_rate, bottom_epochs;
    RL(){};
    RL(long seed, std::string parameter)
    {
        controller = RLController();
        if (parameter == "mutation_rate")
        {
            controller.addParameter("mutation_rate", ParameterController::ParameterType::NUMERIC_DOUBLE_, 0.f, 1.f);
            num_params = 1;
        }
        else if (parameter == "bottom_epochs")
        {
            controller.addParameter("bottom_epochs", ParameterController::ParameterType::NUMERIC_INT_, 1.f, 10.f);
            num_params = 1;
        }
        else if (parameter == "both")
        {
            controller.addParameter("mutation_rate", ParameterController::ParameterType::NUMERIC_DOUBLE_, 0.f, 1.f);
            controller.addParameter("bottom_epochs", ParameterController::ParameterType::NUMERIC_INT_, 1.f, 10.f);
            num_params = 2;
        }
        else
        {
            throw std::runtime_error("not implemented");
        }
        controller.initialize(seed, "bins:20");
    }

    float get_mutation_rate()
    {
        std::cout << "mutation rate " << mutation_rate << std::endl;
        return mutation_rate;
    }

    int get_bottom_epochs()
    {
        int be = (int)std::round(bottom_epochs);
        std::cout << "bottom epochs " << be << std::endl;
        return be;
    }

    void set_observation(std::vector<float> &obs, float reward)
    {

        obs.insert(obs.begin(), reward);
        std::cout << "obs " << std::endl;
        for (size_t i = 1; i < obs.size(); ++i)
        {
            std::cout << obs[i] << ", ";
        }
        std::cout << std::endl;
        std::cout << "reward " << obs[0] << std::endl;
        controller.updateObservables(obs);
        // Update parameters
        float normal_be = 1;
        bottom_epochs = controller.getNextValue("bottom_epochs", normal_be);

        float normal_mr = 0.125;
        mutation_rate = controller.getNextValue("mutation_rate", normal_mr);
    }
};
int main(int argc, char **argv)
{
    //generate data for scenario 1:
    // try to fix the observation at 0.50
    RL rl = RL(atoi(argv[1]), "mutation_rate");
    float rwd1 = 0.0f;
    float o = 0.0f;
    std::ofstream ofs1("rlstats.txt");
    float performance1 = 0.f;
    for (size_t i = 0; i < 50000; ++i)
    {
        performance1 += rwd1;
        std::cout << "cycle " << i << std::endl;

        std::vector<float> obs;
        obs.push_back(o);
        rl.set_observation(obs, rwd1);
        float mr = rl.get_mutation_rate();
        o += (mr - 0.50) * 0.05;
        // move toroid
        if (o > 1.0f)
        {
            o = o - 1.0f;
        }
        else
        {
            if (o < 0.0f)
            {
                o = 1.0f + o;
            }
        }
        float dist = std::abs(o - 0.50);
        if (dist < 0.05)
        {
            // in the surrounding bins, don't penalise
            rwd1 = 0;
        }
        else
        {
            rwd1 = -dist;
        }
        if (i % 200 == 0)
        {
            ofs1 << performance1 << std::endl;
            performance1 = 0;
        }
    }
    // 2. after N consecutive observations greater than 0.50,
    // get a reward of N
    RL rl2 = RL(atoi(argv[1]), "mutation_rate");
    float rwd = 0.0f;
    size_t N = 0;
    std::ofstream ofs2("rl2stats.txt");
    float performance = 0.f;
    for (size_t i = 0; i < 50000; ++i)
    {
        std::cout << "cycle " << i << std::endl;
        performance += rwd;
        std::vector<float> obs;
        obs.push_back(o);
        rl2.set_observation(obs, rwd);
        float mr = rl2.get_mutation_rate();
        o += (mr - 0.50) * 0.05;
        // move toroid
        if (o > 1.0f)
        {
            o = o - 1.0f;
        }
        else
        {
            if (o < 0.0f)
            {
                o = 1.0f + o;
            }
        }
        if (o > 0.50f)
        {

            // increment counter
            ++N;
            rwd = (float)N;
        }
        else
        {
            rwd = 0.f;
            // reset counter
            N = 0;
        }
        if (i % 500 == 0)
        {
            ofs2 << performance << std::endl;
            performance = 0;
        }
    }

    // 3.  reward zigzag between o in [0.40,0.50] and [0.50,0.60]
    RL rl3 = RL(atoi(argv[1]), "mutation_rate");
    rwd = 0.0f;
    o = 0.50f;
    float sign = 0.0f;
    // 0 -> no previous match, go to either one;
    // -1 -> need to go to [0.30,0.40] for reward;
    // +1 -> need to go to [0.50,0.60] for reward
    N = 0; // counter of the number of alternations
    std::ofstream ofs3("rl3stats.txt");
    performance = 0.f;
    for (size_t i = 0; i < 600000; ++i)
    {
        std::cout << "cycle " << i << std::endl;
        performance += rwd;
        std::vector<float> obs;
        obs.push_back(o);
        rl3.set_observation(obs, rwd);
        float mr = rl3.get_mutation_rate();
        o += (mr - 0.50);
        // move toroid
        if (o > 1.0f)
        {
            o = o - 1.0f;
        }
        else
        {
            if (o < 0.0f)
            {
                o = 1.0f + o;
            }
        }
        if (o > 0.40f && o < 0.50f)
        {
            if (sign == 0.0f)
            {
                // increment counter
                ++N;
                sign = 1.0f;
                rwd = 0.0f;
            }
            else
            {
                if (sign == -1.0f)
                {
                    
                    rwd = (float) N; // success
                    sign = 1.0f;
                    ++N;
                }
                else
                {
                    sign = 0.0f; // not successful, reset everything
                    N = 0;
                    rwd = 0;
                }
            }
        }
        else
        {
            if (o > 0.50f && o < 0.60f)
            {
                if (sign == 0.0f)
                {
                    // increment counter
                    
                    sign = -1.0f;
                    rwd = 0.0f;
                    ++N;
                }
                else
                {
                    if (sign == 1.0f)
                    {
                         ++N;
                        rwd = N; // success
                        sign = -1.0f;
                       
                    }
                    else
                    {
                        sign = 0.0f; // not successful, reset everything
                        N = 0;
                        rwd = 0;
                    }
                }
            }
            else
            {
                sign = 0.0f; // not successful, reset everything
                N = 0;
                rwd = 0;
            }
        }
        if (i % 500 == 0)
        {
            ofs3 << performance << std::endl;
            performance = 0;
        }
    }
    return 0;
}
