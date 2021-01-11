#include <iostream>
#include <cstdlib>
#include <planar_dart/planar_dart_simu.hpp>

#define CTRL_SIZE 8

struct Params
{
    static constexpr double radius() { return 0.01; }

    static Eigen::Vector3d head() { return Eigen::Vector3d(1, 1, 0.5); }

    static Eigen::Vector3d tail() { return Eigen::Vector3d(1, 0, 0.5); }

    static Eigen::Vector4d color() { return Eigen::Vector4d(1, 0, 0, 1); }

    static std::string skel_name() { return "floor"; }

    static std::string body_name() { return "BodyNode"; }
};

int main(int argc, char **argv)
{
    // using the same model as the hexapod and so the robot has a damages parameter but is set to 0
    std::vector<planar_dart::planarDamage> damages(0);
    if (argc == CTRL_SIZE + 4)
    {

        damages.push_back(planar_dart::planarDamage(argv[10],argv[11]));
    }
    else
    {
        assert(argc == CTRL_SIZE + 2);
    }


    srand(time(NULL)); //set seed

    // loads the robot with name planar tels it that it is not a URDF file and give it the blank damages
    auto global_robot = std::make_shared<planar_dart::planar>(argv[9], "planar", false); 
    
    std::vector<double> ctrl = {atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]),
                                atof(argv[5]), atof(argv[6]), atof(argv[7]), atof(argv[8])};

    //note: boost fusion vector at most 10 template arguments
    using desc_t = boost::fusion::vector<planar_dart::descriptors::PositionalCoord,
                                         planar_dart::descriptors::PolarCoord,
                                         planar_dart::descriptors::JointPairAngle,
                                         planar_dart::descriptors::RelativeJointPairAngle,
                                         planar_dart::descriptors::AngleSum,
                                         planar_dart::descriptors::ConstantHigh,
                                         planar_dart::descriptors::ConstantLow,
                                         planar_dart::descriptors::Genotype,
                                         planar_dart::descriptors::NoisyGenotype,
                                         planar_dart::descriptors::RandomVal>;

    using safe_t = boost::fusion::vector<planar_dart::safety_measures::LinkColliding>;
    using viz_t = boost::fusion::vector<planar_dart::visualizations::TargetArrow>;
    planar_dart::planarDARTSimu<planar_dart::desc<desc_t>, planar_dart::viz<viz_t>> simu(ctrl, global_robot->clone(), damages);

#ifdef GRAPHIC
    simu.fixed_camera(simu.ISOMETRIC);
#endif
    simu.run();

    std::cout << "Euclidean" << std::endl;
    std::cout << simu.euclidean_distance() << std::endl;

    std::vector<double> v;
    simu.get_descriptor<planar_dart::descriptors::PositionalCoord>(v);
    std::cout << "POS:" << std::endl;
    for (size_t i = 0; i < v.size(); i++)
    {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> vv;
    simu.get_descriptor<planar_dart::descriptors::PolarCoord>(vv);
    std::cout << "POL:" << std::endl;
    for (size_t i = 0; i < vv.size(); i++)
    {
        std::cout << vv[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> sr;
    simu.get_descriptor<planar_dart::descriptors::JointPairAngle>(sr);
    std::cout << "JPA: " << std::endl;
    for (size_t i = 0; i < sr.size(); i++)
    {
        std::cout << sr[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> rra;
    simu.get_descriptor<planar_dart::descriptors::RelativeJointPairAngle>(rra);
    std::cout << "RJPA: " << std::endl;
    for (size_t i = 0; i < rra.size(); i++)
    {
        std::cout << rra[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> vels;
    simu.get_descriptor<planar_dart::descriptors::AngleSum>(vels);

    std::cout << "AS:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    simu.get_descriptor<planar_dart::descriptors::ConstantHigh>(vels);

    std::cout << "CONSTANT_HIGH:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    // simu.get_descriptor<planar_dart::descriptors::ConstantMid>(vels);

    // std::cout << "CONSTANT_MID:" << std::endl;
    // for (size_t i = 0; i < vels.size(); i++)
    // {
    //     std::cout << vels[i] << " ";
    // }
    // std::cout << std::endl;

    simu.get_descriptor<planar_dart::descriptors::ConstantLow>(vels);

    std::cout << "CONSTANT_LOW:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    simu.get_descriptor<planar_dart::descriptors::RandomVal>(vels);

    std::cout << "RANDOM_VAL:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    simu.get_descriptor<planar_dart::descriptors::Genotype>(vels);

    std::cout << "GEN:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    simu.get_descriptor<planar_dart::descriptors::NoisyGenotype>(vels);

    std::cout << "NOISY_GEN:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++)
    {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}
