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
                auto gripper_body = rob->gripper("safety");
                Eigen::Vector3d _posi = gripper_body->getWorldPosition();
                if (_posi[1] > 0)
                {
                    //std::cout << "stop because of collision with wall" << std::endl;
                    simu.stop_sim();
                    return;
                }
                double angle_sum = 0.0;
                std::vector<double> commands = simu.controller().parameters();
                for (size_t i = 0; i < 8; ++i)
                {
                    angle_sum += transform(commands[i]);  // 0 --> -90; 0.50 --> 0 ; 1 --> 90 degrees
                    //std::cout << "angle sum " << angle_sum << std::endl;
                    if (angle_sum > 359.0 || angle_sum < -359.0)
                    {
                        //std::cout << "stop because of cycle; collision with itself" << std::endl;
                        simu.stop_sim();
                        return;
                    }
                }

            }

            double transform(double command)
            {
                return 180.0* (command - 0.50);
            }
        };
    } // namespace safety_measures
} // namespace planar_dart

#endif
