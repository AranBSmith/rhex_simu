#ifndef RHEX_DART_RHEX_CONTROL
#define RHEX_DART_RHEX_CONTROL

#include <rhex_dart/rhex.hpp>
#include <rhex_controller/rhex_controller_simple.hpp>

#define PI 3.14159265

namespace rhex_dart {

    class RhexControl {
    public:
        using robot_t = std::shared_ptr<Rhex>;

        RhexControl() {}

        RhexControl(const std::vector<double>& ctrl, robot_t robot)
            : _controller(ctrl, robot->broken_legs()), _robot(robot)
        {
            _target_positions = _robot->skeleton()->getPositions();

            size_t dof = _robot->skeleton()->getNumDofs();
            _p = Eigen::VectorXd::Zero(dof);

            // first 6 DOF are 6d position - we don't want to put P values there
            for (size_t i = 0; i < 6; ++i) {
                _p(i) = 0.0;
            }

            for (size_t i = 6; i < dof; ++i) {
                _p(i) = 1.0;
            }
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            _controller.set_parameters(ctrl);
        }

        const std::vector<double>& parameters() const
        {
            return _controller.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {
            // gets target positions as a vector of doubles
            auto inter = _controller.pos(t);
//            std::cout << "inter: " ;
//            for (int i = 0; i < inter.size(); i++) {
//                    std::cout << inter.at(i) << ' ';
//            }
//            std::cout << std::endl;

            // initialised the target positions vector as all zeros
            _target_positions = Eigen::VectorXd::Zero(inter.size() + 6);

            // sets the first 6 DoF to be 0 and then updates the remaining target positions with the ones from the controller
            for(int i = 0; i < (inter.size() + 6); i++){
                if(i < 6){
                    _target_positions[i] = 0;
                } else {
                    _target_positions[i] = inter[i-6];
                }
            }
           
            set_commands();
        }

        void set_commands()
        {

            if (_robot == nullptr)
                return;

            Eigen::VectorXd q = _robot->skeleton()->getPositions();
            std::cout << "Current positions: ";
            std::cout << q << std::endl;
            Eigen::VectorXd dq = _robot->skeleton()->getVelocities();

            // values of q exceed 2*PI this normalises them in the range 0-2*PI
            for(int i = 0; i < q.size(); i++) {
                q[i] = (remainder(q[i], double(2 * PI)) < 0) ? remainder(q[i], double(2 * PI)) + 2 * PI : remainder(q[i], double(2 * PI));
            }

            get_PD();

            // depending on where my target is and where the robot is at i want the differences to be configured differently
            // if my target position is 6 whereas my current position is 0.5 there is no point going rotating forward fast 
            // it would be preferable to have a negative difference and head toward the target position.

            // should the rhex ever reverse its legs? being behind in position should inspire more power to rotating the hip joint
            // K_d does not seem to used here, so is this just a p-controller?
            double diff;
            size_t dof = _robot->skeleton()->getNumDofs();
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);

            for (int i = 0; i < dof; i++) {
                if(_target_positions[i] - q[i] > PI) {
                    diff = (_target_positions[i] - q[i]) - 2 * PI;
                } else if(_target_positions[i] - q[i] < -PI) {
                    diff = _target_positions[i] - q[i] + 2 * PI;
                } else {
                    diff = _target_positions[i] - q[i];
                }

                // makes sure a torque larger than 3 is not applied
                commands[i] = (_K_p[i] * (diff) > 3) ? 3 : _K_p[i] * (diff);
                commands[i] = (commands[i] < -3) ? -3 : commands[i];
            }

//            std::cout << "Setting commands: ";
//            std::cout << commands << std::endl;
            // updates the robot with the new forces for the joints
            _robot->skeleton()->setCommands(commands);
        }

        void get_PD(){
            auto tempKp = _controller.get_Kp();
            auto tempKd = _controller.get_Kd();
            _K_p = Eigen::VectorXd::Zero(tempKp.size());
            _K_d = Eigen::VectorXd::Zero(tempKd.size());

            for(int i=0; i < tempKp.size(); i++){
                _K_d[i]=tempKd[i];
                _K_p[i]=tempKp[i];
            }
        }

    protected:
        rhex_controller::RhexControllerSimple _controller;
        robot_t _robot;

        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;     // not used
        Eigen::VectorXd _K_p; //
        Eigen::VectorXd _K_d;

    };
}

#endif
