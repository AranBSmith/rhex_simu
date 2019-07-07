#ifndef RHEX_DART_RHEX_CONTROL_HOPF
#define RHEX_DART_RHEX_CONTROL_HOPF

#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_hopf.hpp>
#define PI 3.14159265

#define PROP 5
#define INTEG 20
#define DIFF 0.0

namespace rhex_dart {

    class RhexControlHopf {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControlHopf() {}

        RhexControlHopf(const std::vector<double>& ctrl, robot_t robot)
            : _robot(robot)
        {
            set_parameters(ctrl);
            _feedback = std::vector<double>(6, 0.0);
            _compensatory_count = std::vector<int>(6, 0);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {

            std::vector<double> start_phase(6,0);
            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            for (size_t i = 0; i < 6; ++i)
            {
                start_phase[i] = current_positions[i+6];
            }

            _pid.clear();
            _pid.set_Kp(ctrl[0] * PROP);
            _pid.set_Ki(ctrl[1] * INTEG);
            _pid.set_Kd(ctrl[2] * DIFF);

            std::vector<double>::const_iterator first = ctrl.begin() + 3;
            std::vector<double>::const_iterator last = ctrl.begin() + ctrl.size();
            std::vector<double> cpg_ctrl(first, last);

            _cpg.set_parameters(cpg_ctrl);
        }

        const std::vector<std::vector<double> > parameters() const
        {
            return _cpg.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {
            if (_robot == nullptr)
                return;

            _target_positions = _cpg.pos(t);

            Eigen::VectorXd current_positions = _robot->skeleton()->getPositions();

            std::vector<double> feedback(6,0);

            for (size_t i = 0; i < 6; ++i){
                feedback[i] = current_positions[i+6];
            }

            // if the target is one or more full rotations ahead, subtract the appropriate amount of rotations.
            // this needs to be sustained for the rest of the simulation as the signal will never decrease.
            for (size_t i = 0; i < 6; ++i){
                double diff = _target_positions[i] - 2 * PI * _compensatory_count[i] - feedback[i];
                while(diff >= 2*PI){
                    _compensatory_count[i] += 1;
                    diff = _target_positions[i] - 2 * PI * _compensatory_count[i] - feedback[i];
                }
                _target_positions[i] -= 2 * PI * _compensatory_count[i];

                // similarly, if the feedback is more than 2 PI ahead of the signal, we dont want the leg to wait
                // which influences other legs in a negative way because of phase.
//                diff = feedback[i] - _target_positions[i];
//                while (diff > 2*PI){
//                    _compensatory_count[i] -= 1;
//                    diff = feedback[i] - _target_positions[i] + 2 * PI * _compensatory_count[i];
//                }
//                _target_positions[i] += 2 * PI * _compensatory_count[i];
                diff = feedback[i] - _target_positions[i];
                if (diff > 2*(3*PI)/4)
                    _target_positions[i] +=  2 * PI;
            }

            std::cout << "Target/cpg/setpoint positions: " ;
            for (size_t i = 0; i < 6; ++i){
                std::cout << _target_positions[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "feedback/current positions: " ;

            for (size_t i = 0; i < 6; ++i){
                std::cout << feedback[i] << " ";
            }
            std::cout << std::endl;

            _pid.set_points(_target_positions);
            _pid.update(feedback, t);

            std::vector<double> pid_output = _pid.get_output();

            Eigen::VectorXd commands = Eigen::VectorXd::Zero(54);
            for (size_t i = 0; i < 6; ++i){
                if (pid_output[i] > 3)
                    commands[i+6] = 3;
                else if (pid_output[i] < -3)
                    commands[i+6] = -3;
                else
                    commands[i+6] = pid_output[i];

            }

            std::cout << "Setting commands: ";
            for (size_t i = 0; i < 6; ++i){
                std::cout << commands[i + 6] << " ";
            }
            std::cout << std::endl;

            _robot->skeleton()->setCommands(commands);

        }

    protected:
        rhex_controller::RhexControllerHopf _cpg;
        rhex_dart::PIDControl _pid;
        robot_t _robot;
        std::vector<double> _feedback;
        std::vector<double> _target_positions;
        std::vector<int> _compensatory_count;


    };
}

#endif
