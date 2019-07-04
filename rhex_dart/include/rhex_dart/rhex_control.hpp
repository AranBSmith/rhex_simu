#ifndef RHEX_DART_RHEX_CONTROL
#define RHEX_DART_RHEX_CONTROL

#include <rhex_dart/rhex.hpp>
#include <rhex_dart/pid_control.hpp>
#include <rhex_controller/rhex_controller_simple.hpp>
#include <rhex_controller/rhex_controller_cpg.hpp>
#define PI 3.14159265

namespace rhex_dart {

    class RhexControl {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControl() {}

        RhexControl(const std::vector<double>& ctrl, robot_t robot)
            : _cpg(ctrl), _robot(robot),  _pid(0.2, 15.0, 0.001)
        {
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            _cpg.set_parameters(ctrl);
        }

        const std::vector<double> parameters() const
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

            _target_positions = _cpg.pos(t); // get this from cpg

            Eigen::VectorXd tmp = _robot->skeleton()->getPositions();

            // it looks like you can give the joint any value and it will translate it into an appropriate range for itself to work with.
//            std::cout << "Current positions: ";
//            std::cout << tmp << std::endl;

            // feedback to be put into PID
            // CPG phase is the new setpoint for PID

            // setpoint PID from CPG

            _pid.set_points(_target_positions);

            _pid.set_delta_time(t);

            std::vector<double> feedback(6,0);


            // update pid with current joint positions
//            std::cout << "This is the tmp array: ";
//            for (size_t i = 0; i < tmp.size(); ++i){
//                std::cout <<  tmp[i] << " ";
//            }
            //std::cout << std::endl;

            for (size_t i = 0; i < 6; ++i){
                feedback[i] = tmp[i+6];
            }

            // the pid controller updates its values with the new setpoint
            std::cout << "Feedback is: " <<std::endl;
            for ( size_t i = 0; i < feedback.size(); ++i){
                std::cout << feedback[i] << " ";
              }
            std::cout << std::endl;

            _pid.update(feedback);

            // updates the robot with the new forces for the joints
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(54);

            std::vector<double> pid_output = _pid.get_output();

            // for each value in pid_output, transform it accordingly using cpg function
            // first 6 are COM, the next six are hipjoint positions, the rest are
            // joints which are not real DOFs but require an input
            for (size_t i = 0; i < 54; ++i){
                if (i >= 6 && i < 12)
                    commands[i] = _cpg.mono_transform(pid_output[i]); // transform the monotous input into the cycle
                else
                    commands[i] = 0;
            }

            std::cout << "Setting commands: ";
            for (size_t i = 0; i < commands.size(); ++i){
                std::cout << commands[i] << " ";
            }

            std::cout << std::endl;
            _robot->skeleton()->setCommands(commands);

        }

    protected:
        rhex_controller::RhexControllerCPG _cpg;
        rhex_dart::PIDControl _pid;
        robot_t _robot;

        std::vector<double> _target_positions;


    };
}

#endif
