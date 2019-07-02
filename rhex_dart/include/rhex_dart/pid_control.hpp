#ifndef RHEX_DART_PID_CONTROL
#define RHEX_DART_PID_CONTROL

#define PI 3.14159265

namespace rhex_dart {

    class PIDControl {
    public:

        PIDControl() {}

        PIDControl(double p, double i, double d, double delta_time)
        {

            _Kp = p;
            _Ki = i;
            _Kd = d;
            _delta_time = delta_time;

            clear();
        }

        void clear() // Clears PID computations and coefficients
        {
            _set_point = 0.0;

            _Pterm = 0.0;
            _Iterm = 0.0;
            _Dterm = 0.0;
            _last_error = 0.0;

            _windup_guard = PI;

            _output = 0.0;
        }

        void update(double feedback_value){
            // Calculates PID value for given reference feedback
            // u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

            double error = _set_point - feedback_value;
            // print("Delta time is: " + str(delta_time))
            double delta_error = error - _last_error;

            _Pterm = _Kp * error;
            _Iterm += error * _delta_time;

            if (_Iterm < -_windup_guard)
                _Iterm = -_windup_guard;
            else if (_Iterm > _windup_guard)
                _Iterm = _windup_guard;

            _Dterm = 0.0;
            if (_delta_time > 0)
                _Dterm = delta_error / _delta_time;

            // Remember last error for next calculation
            _last_error = error;
            // std::cout << "str(_Pterm) + " " + str(_Iterm) + " " + str(_Dterm) << std::endl;
            _output = _Pterm + (_Ki * _Iterm) + (_Kd * _Dterm);
        }

        void set_Kp(double proportional_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Proportional Gain
            _Kp = proportional_gain;
        }

        void set_Ki(double integral_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Integral Gain
            _Ki = integral_gain;
        }

        void set_Kd(double derivative_gain)
        {
            // Determines how aggressively the PID reacts to the current error with setting Derivative Gain
            _Kd = derivative_gain;
        }

        void set_windup(double windup)
        {
            /* Integral windup, also known as integrator windup or reset windup,
            refers to the situation in a PID feedback controller where
            a large change in setpoint occurs (say a positive change)
            and the integral terms accumulates a significant error
            during the rise (windup), thus overshooting and continuing
            to increase as this accumulated error is unwound
            (offset by errors in the other direction).
            The specific problem is the excess overshooting.
            */
            _windup_guard = windup;
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            _controller.set_parameters(ctrl);
        }

        const std::vector<double>& parameters() const
        {
            return _controller.parameters();
        }

        double get_output()
        {
            return _output;
        }

    protected:
        double _Kp;
        double _Ki;
        double _Kd;
        double _delta_time;

        double _set_point;
        double _Pterm;
        double _Iterm;
        double _Dterm;
        double _last_error = 0.0;

        double _windup_guard = PI;

        double _output = 0.0;
    };
}


#endif // PID_CONTROL

