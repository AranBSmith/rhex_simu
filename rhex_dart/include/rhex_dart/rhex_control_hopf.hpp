#ifndef RHEX_DART_RHEX_CONTROL_HOPF
#define RHEX_DART_RHEX_CONTROL_HOPF

#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_hopf.hpp>
#define PI 3.14159265

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
            _pid.set_Kp(ctrl[0]);
            _pid.set_Ki(ctrl[1]);
            _pid.set_Kd(ctrl[2]);

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


    };
}

#endif
