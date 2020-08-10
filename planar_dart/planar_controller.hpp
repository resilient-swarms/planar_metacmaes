#define _USE_MATH_DEFINES
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define PI 3.14159265
#define JOINT_SIZE 8
#define DOF 6
#define F 3
#define OFFSET PI/2
#define MIN_P 0.33 // min period is 0.33 or 3 cycles per second

namespace planar_controller {

    class controller {
    public:

        controller() {}

        controller(const std::vector<double>& ctrl)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == JOINT_SIZE);

            _ctrl.resize(JOINT_SIZE, 0);
            for(size_t i = 0; i < JOINT_SIZE; ++i)
            {
                _ctrl[i] = ctrl[i];
            }

            // take away some portion of 0.66 controlled by argument
            // giving us a cycle range between (0.33Hz - 1Hz)
            //_period = 1 - ctrl[0] * (1 - MIN_P);
            //_duty_factor.resize(DOF, 0);
            //_duty_time.resize(DOF, 0);
            //_stance_angle.resize(DOF, 0);
            //_stance_offset.resize(DOF, 0);
            /*
            for (size_t i = 1; i <= DOF; ++i)
            {
                _duty_factor[i-1] = ctrl[i];
                _duty_time[i-1] = _duty_factor[i-1] * _period;
                _stance_angle[i-1] = ctrl[i+6] * PI;
                _stance_offset[i-1] = (ctrl[i+12] - 0.5) * OFFSET;
            }

            // define phase offsets here.
            // this scheme does not bias gaits to belong to a particular style like tripod.
            _phase_offset.resize(DOF, 0);
            _phase_offset[0] = 0;                           // this is the reference leg
            _phase_offset[1] = ctrl[19] * _period / 2;
            _phase_offset[2] = ctrl[20] * _period / 2;
            _phase_offset[3] = ctrl[21] * _period / 2;
            _phase_offset[4] = ctrl[22] * _period / 2;
            _phase_offset[5] = ctrl[23] * _period / 2;

            _last_time = 0;
            _dt = 0.0;

            // offset according to respective phase offsets defined above, this will define the type of gait
            _phase.resize(DOF, 0);
            for(size_t i = 0; i < DOF; ++i)
                _phase[i] += _phase_offset[i];

            _counter.resize(DOF, 0);*//
        }

        // calculate leg signal according to (Seipal and Holmes 2007)
        std::vector<double> pos(double t)
        {
           /* _dt = t - _last_time;
            _last_time = t;*/

            update();
            /*
            std::vector<double> output(DOF,0);

            for (size_t i = 0; i < DOF; ++i){
                double t = fmod(_phase[i], _period);
                if (t <= _duty_time[i])
                    output[i] = - _stance_angle[i] / 2 + (_stance_angle[i] / _duty_time[i]) * t;

                else if(t > _duty_time[i] && t <= _period)
                    output[i] = _stance_angle[i] / 2 + ((2 * PI - _stance_angle[i]) / (_period - _duty_time[i])) * (t - _duty_time[i]);

                _counter[i] = floor(_phase[i] / _period);
                output[i] += _counter[i] * 2 * PI;

                for (size_t j = 0; j < DOF; ++j)
                    output[i] += _stance_offset[i];
            }*/

            //return output;
            return _ctrl;
        }

        void update()
        {
            //for (size_t i = 0; i < DOF; ++i)
            //    _phase[i] = _phase[i] + _dt;
        }

        const std::vector<double>& parameters() const
        {
            return _ctrl;
        }

    protected:
        double _f;
        double _period;
        double _time;
        double _dt;
        double _last_time;

        std::vector<double> _stance_angle;
        std::vector<double> _duty_factor;
        std::vector<double> _duty_time;
        std::vector<double> _stance_offset;
        std::vector<double> _phase_offset;

        std::vector<std::vector<double> > _phase_bias;
        std::vector<std::vector<double> > _weights;

        std::vector<int> _counter;
        std::vector<double> _ctrl;
        std::vector<double> _phase;
    };
}

#endif // RHEX_CONTROLLER_BUEHLER
