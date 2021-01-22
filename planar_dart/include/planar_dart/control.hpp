#ifndef PLANAR_DART_CONTROL
#define PLANAR_DART_CONTROL

#include <algorithm>
#include <planar_dart/planar.hpp>
#include <planar_dart/control.hpp>
#define PI 3.14159265

#define JOINT_COUNT 8      // no. outputs from the controller
#define STUCK 0.7853981625 // 45Deg

namespace planar_dart
{

    class control
    {
    public:
        using robot_t = std::shared_ptr<planar>;

        control() {}

        control(const std::vector<double> &ctrl)
        {
            set_parameters(ctrl);
        }

        control(const std::vector<double> &ctrl, robot_t robot, std::vector<planar_dart::planarDamage> damages = {})
            : _robot(robot), _damages(damages)
        {
            set_parameters(ctrl);
        }

        void set_robot(robot_t r)
        {
            _robot = r;
        }

        void set_parameters(const std::vector<double> &ctrl)
        {
            std::vector<double>::const_iterator first = ctrl.begin();
            std::vector<double>::const_iterator last = ctrl.begin() + ctrl.size();
            std::vector<double> f_ctrl(first, last);

            assert(f_ctrl.size() == JOINT_SIZE);

            _ctrl.resize(JOINT_SIZE, 0);
            for (size_t i = 0; i < JOINT_SIZE; ++i)
            {
                _ctrl[i] = f_ctrl[i];
            }
            _leg_count = JOINT_COUNT;

            int leg = 0;
            for (auto dmg : _damages)
            {
                size_t pos = dmg.type.rfind("stuck");
                if (pos != std::string::npos)
                {
                    _leg_count -= 1;
                    int leg = stoi(dmg.data);
                    double radians = stod(dmg.type.substr(pos + 5));
                    _stuck_legs[leg] = radians;
                }
                else
                {
                    size_t pos = dmg.type.rfind("offset");
                    if (pos != std::string::npos)
                    {
                        _leg_count -= 1;
                        int leg = stoi(dmg.data);
                        double radians = stod(dmg.type.substr(pos + 6));
                        _offset_legs[leg] = radians;
                    }
                }

                leg += 1;
            }
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
            update();
        }

        void update()
        {
            if (_robot == nullptr)
                return;

            //_target_positions = _ctrll.pos(t);

            std::vector<double> values = parameters();

            size_t dof = _robot->skeleton()->getNumDofs();
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);
#ifdef TEST
            _final_commands = std::vector<double>(dof);// commands before conversion to radians, accounting for damages
#endif
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                // for those legs not removed, add the command
                auto it = _stuck_legs.find(i);
                if (it != _stuck_legs.end())
                {
                    //tcommands[i] = values[i];
#ifdef TEST
                    _final_commands[i] = _stuck_legs[i];
#endif
                    commands[i] = toRadians(_stuck_legs[i]);
                }
                // indicate removed legs with an stuck value
                else
                {
                    it = _offset_legs.find(i);
                    if (it != _offset_legs.end())
                    {
                        commands[i] = std::max(0.0, std::min(1.0, _offset_legs[i] + values[i]));
#ifdef TEST
                        _final_commands[i] = commands[i];
#endif
                        commands[i] = toRadians(commands[i]);
                    }
                    else
                    {
                        //tcommands[i] = values[i];
#ifdef TEST
                        _final_commands[i] = values[i];
#endif
                        commands[i] = toRadians(values[i]);
                    }
                }
            }
            //std::cout << "commands " << commands.transpose() << std::endl;
            _robot->skeleton()->setCommands(commands);
        }
        double toNormalise(double angle)
        {
            double MAX = (PI / 2.0);
            double MIN = 0.0 - (PI / 2.0);

            return (angle - MIN) / (MAX - MIN);
        }

        double toRadians(double val)
        {
            double MAX = (PI / 2.0);
            double MIN = 0.0 - (PI / 2.0);

            return ((MAX - MIN) * val) + MIN;
        }
#ifdef TEST
        std::vector<double> get_final_commands()
        {
            return _final_commands;
        }
#endif

    protected:
        std::vector<double> _ctrl;
        robot_t _robot;
        std::vector<planarDamage> _damages;
        std::map<int, double> _stuck_legs;
        std::map<int, double> _offset_legs;
        int _leg_count;
#ifdef TEST
        std::vector<double> _final_commands;
#endif
    };
} // namespace planar_dart

#endif
