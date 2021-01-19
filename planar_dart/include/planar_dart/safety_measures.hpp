#ifndef PLANAR_DART_SAFETY_MEASURES_HPP
#define PLANAR_DART_SAFETY_MEASURES_HPP

namespace planar_dart
{

    namespace safety_measures
    {

        struct LinkColliding
        {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            {
                //skip i=0 because that's the starting joint
                for (size_t i = 1; i <= JOINT_SIZE; ++i) // also do i=8 because that's the gripper
                {
                    //std::string gripper_index = "joint_" + std::to_string(i+1);
                    //auto gripper_body = rob->skeleton()->getBodyNode(gripper_index)->createMarker();
                    auto joint_body = rob->joint(i, "linkcollidingcheck");
                    Eigen::Vector3d _posi = joint_body->getWorldPosition();
                    if (_posi[1] > 0.01) //
                    {
#ifdef GRAPHIC
                        std::cout << "stop because of joint " << i << " hitting the wall " << _posi[1] << std::endl;
#endif
                        simu.stop_sim();
                        return;
                    }
                }
                double angle_sum = 0.0;
                std::vector<double> commands = simu.controller().parameters();
                for (size_t i = 0; i < JOINT_SIZE; ++i)
                {
                    angle_sum += transform(commands[i]); // 0 --> -90; 0.50 --> 0 ; 1 --> 90 degrees
#ifdef GRAPHIC
                    std::cout << "angle sum " << angle_sum << std::endl;
#endif
                    if (angle_sum > 359.0 || angle_sum < -359.0)
                    {
#ifdef GRAPHIC
                        std::cout << "stop because of cycle; collision with itself" << std::endl;
#endif
                        simu.stop_sim();
                        return;
                    }
                }
            }

            double transform(double command)
            {
                return 180.0 * (command - 0.50);
            }
        };
    } // namespace safety_measures
} // namespace planar_dart

#endif
