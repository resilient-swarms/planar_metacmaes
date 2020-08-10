#ifndef PLANAR_DART_CONTROL
#define PLANAR_DART_CONTROL

#include <algorithm>
#include "../planar_dart/planar.hpp>
#include "../planar_dart/planar_controller.hpp"
#define PI 3.14159265

#define JOINT_COUNT 8      // no. outputs from the controller
#define STUCK 0.7853981625     // 45Deg

namespace planar_dart {

    class control {
    public:
        using robot_t = std::shared_ptr<planar>;

        control() {}

        control(const std::vector<double>& ctrl)
        {
            set_parameters(ctrl);
            
        }

        control(const std::vector<double>& ctrl, robot_t robot, std::vector<planar_dart::planarDamage> damages = {})
            : _robot(robot), _damages(damages)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            std::vector<double>::const_iterator first = ctrl.begin();
            std::vector<double>::const_iterator last = ctrl.begin() + ctrl.size();
            std::vector<double> f_ctrl(first, last);

            assert(f_ctrl.size() == JOINT_SIZE);

            _ctrl.resize(JOINT_SIZE, 0);
            for(size_t i = 0; i < JOINT_SIZE; ++i)
            {
                _ctrl[i] = f_ctrl[i];
            }
            _leg_count = JOINT_COUNT;

            // need to know which, and the no. of legs removed.
            int leg = 0;
            for (auto dmg : _damages) {
                if (dmg.type == "stuck_at45") {
                    _leg_count -= 1;
                    _removed_legs.push_back(stoi(dmg.data));
                }

                leg+=1;
            }
            //_ctrll.set_parameters(f_ctrl);
        }

        const std::vector<double> parameters() const
        {
            return _ctrl;
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {
            std::cout << "No time step is required, movement is discrete" << std::endl;
            update()
        }

        void update()
        {
            if (_robot == nullptr)
                return;

            //_target_positions = _ctrll.pos(t);
            
            

            std::vector<double> values = parameters();

            std::vector<double> tcommands(JOINT_COUNT,0);

            size_t dof = _robot->skeleton()->getNumDofs();
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);

            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                // for those legs not removed, add the command
                if (!std::binary_search(_removed_legs.begin(), _removed_legs.end(), i))
                {
                    //tcommands[i] = values[i];
                    commands[i] = toRadians(values[i]);
                }

                // indicate removed legs with an stuck value 
                else
                {
                    //tcommands[i] = STUCK;
                    commands[i] = STUCK;
                }
            }

            _robot->skeleton()->setCommands(commands);

        }
        double toNormalise(double angle){
            double MAX = PI/2;
            double MIN = 0.0 - PI/2;

            return (angle - MIN)/(MAX - MIN);
        }

        double toRadians(double val){
            double MAX = PI/2;
            double MIN = 0.0 - PI/2;

            return ((MAX - MIN) * val) + MIN;
        }

    protected:
        //planar_controller::controller _ctrll;
        std::vector<double> _ctrl;
        //rhex_dart::PIDControl _pid;
        robot_t _robot;
        std::vector<PlanarDamage> _damages;
        std::vector<int> _removed_legs;
        int _leg_count;
    };
}

#endif
