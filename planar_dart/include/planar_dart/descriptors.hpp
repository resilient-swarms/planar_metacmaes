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

namespace planar_dart
{

    namespace descriptors
    {
        struct DescriptorBase
        {
        public:
            using robot_t = std::shared_ptr<planar>;
            double factor = 0.5425; // -0.5425 is the largest y-value in skeleton (link 8)
            double thickness = 0.0775;// thickness of the skeleton
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                assert(false);
            }

            template <typename T>
            void get(T &results)
            {
                assert(false);
            }
        };

        struct PositionalCoord : public DescriptorBase
        {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                auto gripper_body = rob->gripper();
                Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                _x = _posi[0];
                _y = _posi[1];
                //normalise
                _y = (_y + factor) / (2 * factor);
                _x = (-_x) / (factor);
#ifdef GRAPHIC
                std::cout << "gripper position " << _posi << std::endl;
                std::cout << "positional coord " << _x << " , " << _y << std::endl;
#endif
            }

            void get(std::vector<double> &results)
            {
                results.clear();
                results.push_back(_x <= 0.0 ? 0.0 : _x);
                results.push_back(_y <= 0.0 ? 0.0 : _y);
            }

        protected:
            double _x, _y;
        };
        struct PolarCoord : public DescriptorBase
        {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                auto gripper_body = rob->gripper();
                Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                double _x = _posi[0];
                double _y = _posi[1];
                /* std::string gripper_index = "link_8";
                auto gripper = rob->skeleton()->getBodyNode(gripper_index);
                double _x = gripper->getPosition(0);
                double _y = gripper->getPosition(1);*/
                _d = sqrt(pow(_x, 2) + pow(_y, 2));
                _theta = atan2(_y,_x);
                _theta = _theta <= 0.10 ? _theta + 2.*PI : _theta;// 0.10 leaves room for thickness of the robot
                assert((_y > 0) || (_theta<=2*PI + 0.10 && _theta>=PI-0.10 && _d <= factor + thickness/2));//either illegal move to wall or d in factor and theta [PI,2PI]
                
#ifdef GRAPHIC
                std::cout << "gripper position " << _posi << std::endl;
                std::cout << "polar coord " << _d << " , " << _theta << std::endl;
#endif
            }

            void get(std::vector<double> &results)
            {
                results.clear();
                results.push_back(normalise_radius(_d));
                results.push_back(normalise_angle(_theta));
                
            }

        protected:
            double _theta, _d;
            double MAX = 2*PI;
            double MIN = PI;
            double normalise_radius(double r)
            {
                double temp = r/factor;
                return std::min(std::max(0.0,temp),1.0);
            }
            double normalise_angle(double angle)
            {
                double temp = (angle - MIN) / PI;
                return std::min(std::max(0.0,temp),1.0);
            }
        };
        struct ResultantAngle : public DescriptorBase
        {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                _angles = {};
                Eigen::Vector3d _base(0, 0, 0);
                for (size_t i = 1; i < JOINT_SIZE;)
                {
                    //std::string gripper_index = "joint_" + std::to_string(i+1);
                    //auto gripper_body = rob->skeleton()->getBodyNode(gripper_index)->createMarker();
                    auto joint_body = rob->joint(i);
                    Eigen::Vector3d _posi = joint_body->getWorldPosition();
                    joint_body = rob->joint(i - 1);
                    Eigen::Vector3d _posi_r = joint_body->getWorldPosition();
                    //auto gripper_body = rob->gripper();
                    //Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                    double a = toNormalise(getRAngle(_base[0], _base[1], _posi_r[0], _posi_r[1], _posi[0], _posi[1]));
                    _angles.push_back(a <= 0 ? 0.0 : a);
#if GRAPHIC
                    std::cout << "joint " << i - 1 << " position " << _posi_r.transpose() << std::endl;
                    std::cout << "joint " << i << " position " << _posi.transpose() << std::endl;
                    std::cout << "angle " << i <<": " << _angles.back();
#endif
                    //_angles.push_back(a);
                    /*auto bod = rob->skeleton()->getJoint(gripper_index);
                    _angles.push_back(atan(bod.getPosition(1) / bod.getPosition(0)));*/
                    _base = _posi;
                    i += 2;
                }
            }

            void get(std::vector<double> &results)
            {
                results.clear();
                results = _angles;
            }

        protected:
            std::vector<double> _angles;
            double toNormalise(double angle)
            {
                double MAX = (PI / 2.0);
                double MIN = 0.0 - (PI / 2.0);

                return (angle - MIN) / (MAX - MIN);
            }
            double getRAngle(double P1X, double P1Y, double P2X, double P2Y,
                             double P3X, double P3Y)
            {
                double numerator = P2Y * (P1X - P3X) + P1Y * (P3X - P2X) + P3Y * (P2X - P1X);
                double denominator = (P2X - P1X) * (P1X - P3X) + (P2Y - P1Y) * (P1Y - P3Y);
                if (denominator == 0)
                    return 0.0;
                double ratio = numerator / denominator;

                double angle = atan(ratio);

                return angle;
            }
        };

        struct AngleSum : public DescriptorBase
        {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                _sum_angles = {};
                std::vector<double> commands = simu.controller().parameters();
                //Eigen::VectorXd commands = rob->skeleton()->getCommands();
                for (size_t i = 0; i < 6; ++i)
                {
                    double t = commands[i] + commands[i + 1] + commands[i + 2];
                    //double t = (toNormalise(commands[i]) + toNormalise(commands[i+1]) + toNormalise(commands[i+2]))/3.0;
                    _sum_angles.push_back(t / 3.0);
                }
            }

            void get(std::vector<double> &results)
            {
                results.clear();
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
    } // namespace descriptors
} // namespace planar_dart

#endif
