#include <iostream>
#include <cstdlib>
#include <planar_dart/planar_dart_simu.hpp>

#define CTRL_SIZE 8

struct Params {
    static constexpr double radius() { return 0.01; }

    static Eigen::Vector3d head() { return Eigen::Vector3d(1, 1, 0.5); }

    static Eigen::Vector3d tail() { return Eigen::Vector3d(1, 0, 0.5); }

    static Eigen::Vector4d color() { return Eigen::Vector4d(1, 0, 0, 1); }

    static std::string skel_name() { return "floor"; }

    static std::string body_name() { return "BodyNode"; }
};

int main(int argc, char** argv)
{
    // using the same model as the hexapod and so the robot has a damages parameter but is set to 0
    std::vector<planar_dart::planarDamage> damages(0);

    assert(argc == 8);
    damages.push_back(planar_dart::planarDamage("stuck_at45", "5"));

    // loads the robot with name planar tels it that it is not a URDF file and give it the blank damages
    // possible models: raised.skel, skinny.skel, planar8.skel, raised_loosehind.skel
    auto global_robot = std::make_shared<planar_dart::planar>(std::string("armBody.skel", "planar", false, damages);

    // sets the control vector up, some examples:
    //tripod
    // ./waf && ./build/test 0 1 raised.skel 1 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0 0.5 0 0.5
    // hill climb
    // ./waf && ./build/test 1 1 raised.skel 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0 0 0.5 0.5 0.5
    // stair climbing gait
    // ./waf && ./build/test 2 1 raised.skel 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.33 0.66 0 0.33 0.66
    std::vector<double> ctrl = {atof(argv[0]), atof(argv[1]),
                                atof(argv[2]), atof(argv[3]), atof(argv[4]),
                                atof(argv[5]),atof(argv[6]),atof(argv[7])};

    using desc_t = boost::fusion::vector<planar_dart::descriptors::DutyCycle,
                    planar_dart::descriptors::PositionalCoord,
                    planar_dart::descriptors::PolarCoord,
                    planar_dart::descriptors::ResultantAngle,
                    planar_dart::descriptors::AngleSum>;

     // using safe_t = boost::fusion::vector<planar_dart::safety_measures::BodyColliding, planar_dart::safety_measures::MaxHeight, planar_dart::safety_measures::TurnOver>;
     using viz_t = boost::fusion::vector<>;
     planar_dart::planarDARTSimu<planar_dart::desc<desc_t>, planar_dart::viz<viz_t>> simu(ctrl, global_robot, 1);

#ifdef GRAPHIC
    simu.fixed_camera(Eigen::Vector3d(3, 0, 0.5));
    simu.follow_planar();
#endif
    simu.run();

    std::cout << "Euclidean" << std::endl;
    std::cout << simu.euclidean_distance() << std::endl;


    std::vector<double> v;
    simu.get_descriptor<planar_dart::descriptors::PositionalCoord>(v);
    std::cout << "POS:" << std::endl;
    for (size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> vv;
    simu.get_descriptor<planar_dart::descriptors::PolarCoord>(vv);
    std::cout << "POL:" << std::endl;
    for (size_t i = 0; i < vv.size(); i++) {
        std::cout << vv[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> sr;
    simu.get_descriptor<planar_dart::descriptors::ResultantAngle>(sr);
    std::cout << "RA: " << sr << std::endl;

    std::vector<double> vels;
    simu.get_descriptor<planar_dart::descriptors::AngleSum>(vels);

    std::cout << "AS:" << std::endl;
    for (size_t i = 0; i < vels.size(); i++) {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}
