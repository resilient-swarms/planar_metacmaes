#ifndef PLANAR_DART_DESCRIPTORS_HPP
#define PLANAR_DART_DESCRIPTORS_HPP

#include <algorithm>
#include <map>
#include <vector>
#include <numeric>

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>

#include <planar_dart/planar.hpp>

#define JOINT_SIZE 8

namespace planar_dart {

    namespace descriptors {

        struct DescriptorBase {
        public:
            using robot_t = std::shared_ptr<planar>;

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                assert(false);
            }

            template <typename T>
            void get(T& results)
            {
                assert(false);
            }
        };

	struct PositionalCoord : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                auto gripper_body = rob->gripper();
                Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                _x = _posi[0];
                _y = _posi[1];
                //normalise
                _y = (_y + factor)/(2*factor);
                _x = (-_x)/(factor);

            }

            void get(std::vector<double>& results)
            {
                results.push_back(_x);
                results.push_back(_y);
            }

        protected:
            double _x, _y;
            double factor = 0.5425;
        };
    struct PolarCoord : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                //std::string gripper_index = "link_8";
                //auto gripper_body = rob->skeleton()->getBodyNode(gripper_index)->createMarker();
                auto gripper_body = rob->gripper();
                Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                double _x = _posi[0];
                double _y = _posi[1];
               /* std::string gripper_index = "link_8";
                auto gripper = rob->skeleton()->getBodyNode(gripper_index);
                double _x = gripper->getPosition(0);
                double _y = gripper->getPosition(1);*/

                _d = sqrt(pow(_x, 2) + pow(_y, 2));
                _theta = atan(_y/_x);
                //normalise
                _d /= factor;
                _theta = _theta<0?0.0:toNormalise(_theta);

            }

            void get(std::vector<double>& results)
            {
                results.push_back(_theta);
                results.push_back(_d);
            }

        protected:
            double _theta, _d;
            double factor = 0.5425;
            double toNormalise(double angle){
                double MAX = (PI/2.0);
                double MIN = 0.0 - (PI/2.0);

                return (angle - MIN)/PI;
            }
        };
    struct ResultantAngle : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                _angles = {};
                Eigen::Vector3d _base(0,0,0);
                for(size_t i = 1;i<JOINT_SIZE;){
                    //std::string gripper_index = "joint_" + std::to_string(i+1);
                    //auto gripper_body = rob->skeleton()->getBodyNode(gripper_index)->createMarker();
                    auto joint_body = rob->joint(i);
                    Eigen::Vector3d _posi = joint_body->getWorldPosition();

                    joint_body = rob->joint(i-1);
                    Eigen::Vector3d _posi_r = joint_body->getWorldPosition();
                    //auto gripper_body = rob->gripper();
                    //Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                    double a = toNormalise(getRAngle(_base[0], _base[1], _posi_r[0], _posi_r[1], _posi[0], _posi[1]));
                    _angles.push_back(a<0?0.0:a);
                    /*auto bod = rob->skeleton()->getJoint(gripper_index);

                    _angles.push_back(atan(bod.getPosition(1) / bod.getPosition(0)));*/
                    _base = _posi;
                    i+=2;
                }
            }
                

            void get(std::vector<double>& results)
            {
                results = _angles;
            }

        protected:
            std::vector<double> _angles;
            double factor = 0.5425;
            double toNormalise(double angle){
                double MAX = (PI/4.0);
                double MIN = 0.0 - (PI/4.0);

                return (angle - MIN)/(MAX - MIN);
            }
            double getRAngle(double P1X, double P1Y, double P2X, double P2Y,
            double P3X, double P3Y){
                double numerator = P2Y*(P1X-P3X) + P1Y*(P3X-P2X) + P3Y*(P2X-P1X);
                double denominator = (P2X-P1X)*(P1X-P3X) + (P2Y-P1Y)*(P1Y-P3Y);
                double ratio = numerator/denominator;

                double angle = atan(ratio);

                return angle;
            }

        };

    struct AngleSum : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                _sum_angles = {};
                std::vector<double> commands = simu.controller().parameters();
                //Eigen::VectorXd commands = rob->skeleton()->getCommands();
                for(size_t i = 0;i<6;++i){
                    double t = commands[i] + commands[i+1] + commands[i+2];
                    //double t = (toNormalise(commands[i]) + toNormalise(commands[i+1]) + toNormalise(commands[i+2]))/3.0;
                    _sum_angles.push_back(t/3.0);
                }
            }

            void get(std::vector<double>& results)
            {
                results = _sum_angles;
            }

        protected:
            std::vector<double> _sum_angles;
            /*double toNormalise(double angle){
                double MAX = (PI/2.0);
                double MIN = 0.0 - (PI/2.0);

                return (angle - MIN)/(MAX - MIN);
            }*/
        };
    }
}

#endif
