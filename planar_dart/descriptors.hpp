#ifndef PLANAR_DART_DESCRIPTORS_HPP
#define PLANAR_DART_DESCRIPTORS_HPP

#include <algorithm>
#include <map>
#include <vector>
#include <numeric>

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>

#include "../planar_dart/planar.hpp"

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
                std::string gripper_index = "link_8";
                dart::dynamics::Joint gripper = rob->skeleton()->getBodyNode(gripper_index);
                _x = gripper.getPosition(0);
                _y = gripper.getPosition(1);

            }

            void get(std::vector<double>& results)
            {
                results.push_back(_x);
                results.push_back(_y);
            }

        protected:
            double _x, _y;
        };
    }
    struct PolarCoord : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                std::string gripper_index = "link_8";
                dart::dynamics::Joint gripper = rob->skeleton()->getBodyNode(gripper_index);
                double _x = gripper.getPosition(0);
                double _y = gripper.getPosition(1);

                _d = sqrt(pow(_x, 2) + pow(_y, 2));
                _theta = atan(_y/_x)
            }

            void get(std::vector<double>& results)
            {
                results.push_back(_theta);
                results.push_back(_d);
            }

        protected:
            double _theta, _d;
        };
    }
    struct ResultantAngle : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                for(size_t i = 0;i<JOINT_SIZE;++i){
                    std::string gripper_index = "joint_" + std::to_string(i+1);
                    dart::dynamics::Joint bod = rob->skeleton()->getJoint(gripper_index);

                    _angles.push_back(atan(bod.getPosition(1) / bod.getPosition(0)));
                }
            }
                

            void get(std::vector<double>& results)
            {
                results = _angles;
            }

        protected:
            std::vector<double> _angles;
        };
    }

    struct AngleSum : public DescriptorBase {
        public:

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                Eigen::VectorXd commands = rob->skeleton()->getCommands();
                for(size_t i = 0;i<6;++i){
                    _sum_angles.push_back(commands[i] + commands[i+1] + commands[i+2]);
                }
            }

            void get(std::vector<double>& results)
            {
                results = _sum_angles;
            }

        protected:
            std::vector<double> _sum_angles;
        };
    }
}

#endif
