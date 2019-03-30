#ifndef RHEX_DART_RHEX_CONTROL
#define RHEX_DART_RHEX_CONTROL

#include <rhex_dart/rhex.hpp>
#include <rhex_controller/rhex_controller_simple.hpp>

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
            //auto angles = _controller.pos(t);
           // std::cout<<"5"<<std::endl;
            auto inter = _controller.pos(t);
            //std::cout<<inter[0]<<" "<<inter[1]<<" "<<inter[2]<<" "<<inter[3]<<" "<<inter[4]<<" "<<inter[5]<<std::endl;
            _target_positions=Eigen::VectorXd::Zero(inter.size()+6);
           // std::cout<<"5"<<std::endl;
            for(int i=0; i<(inter.size()+6) ; i++){
                if(i<6){
                    _target_positions[i]=0;
                }
                else{
                    _target_positions[i]=inter[i-6];
                    //std::cout<<inter[i]<<std::endl;
                }
            }
           // std::cout<<_target_positions[6]<<" "<<_target_positions[7]<<" "<<_target_positions[8]<<" "<<_target_positions[9]<<" "<<_target_positions[10]<<" "<<_target_positions[11]<<std::endl;
            set_commands();
        }

        void set_commands()
        {
            if (_robot == nullptr)
                return;
           // std::cout<<"7"<<std::endl;

            Eigen::VectorXd q = _robot->skeleton()->getPositions();
            Eigen::VectorXd dq = _robot->skeleton()->getVelocities();
            for(int i=0; i< q.size() ; i++){
                q[i] = (remainder(q[i],double(2*3.1415))<0)? remainder(q[i],double(2*3.1415))+ 2*3.1415: remainder(q[i],double(2*3.1415));
            }
           // std::cout<<"7"<<std::endl;
            //std::cout<<q[6]<<" "<<q[7]<<" "<<q[8]<<" "<<q[9]<<" "<<q[10]<<" "<<q[11]<<std::endl;
           // std::cout<<dq[6]<<" "<<dq[7]<<" "<<dq[8]<<" "<<dq[9]<<" "<<dq[10]<<" "<<dq[11]<<std::endl;
           // Eigen::VectorXd q_err = _target_positions - q;
            get_PD();
           // std::cout<<_K_d[6]<<" "<<_K_d[7]<<" "<<_K_d[8]<<" "<<_K_d[9]<<" "<<_K_d[10]<<" "<<_K_d[11]<<std::endl;
            //std::cout<<_K_p[6]<<" "<<_K_p[7]<<" "<<_K_p[8]<<" "<<_K_p[9]<<" "<<_K_p[10]<<" "<<_K_p[11]<<std::endl;            
           // std::cout<<_K_d.size()<<" "<<_K_p.size()<<" "<<_target_positions.size()<<" "<<q.size()<<" "<<dq.size()<<std::endl;
          //  Eigen::VectorXd commands = _K_p.array() * (_target_positions.array() - q.array()) - _K_d.array() * dq.array();
            double diff;
            Eigen::VectorXd commands = Eigen::VectorXd::Zero(54);
            for (int i=0 ; i<54 ; i++){
                if(_target_positions[i]-q[i] > 3.1415){
                    diff = (_target_positions[i]-q[i])-2*3.1415;
                }else if(_target_positions[i]-q[i]<-3.1415){
                    diff = _target_positions[i]-q[i]+2*3.1415;
                }else{
                    diff = _target_positions[i]-q[i];
                }
                commands[i] = (_K_p[i]*(diff)>3)?3:_K_p[i]*(diff);
                commands[i] = (commands[i]<-3)?-3:commands[i];
            }
           // std::cout<<commands[6]<<" "<<commands[7]<<" "<<commands[8]<<" "<<commands[9]<<" "<<commands[10]<<" "<<commands[11]<<std::endl;
           // std::cout<<"7"<<std::endl;
           // Eigen::VectorXd vel = q_err * gain;
           // vel = vel.cwiseProduct(_p);

            _robot->skeleton()->setCommands(commands);
           // std::cout<<"----------------"<<std::endl;
         //   std::cout<<commands<< " " << _target_positions<<std::endl;
           // std::cout<<"----------------"<<std::endl;
        }

        void get_PD(){
            auto tempKp = _controller.get_Kp();
            auto tempKd = _controller.get_Kd();
            _K_p = Eigen::VectorXd::Zero(tempKp.size());
            _K_d = Eigen::VectorXd::Zero(tempKd.size());
            for(int i=0; i<tempKp.size() ; i++){
                _K_d[i]=tempKd[i];
                _K_p[i]=tempKp[i];
            }
        }



    protected:
        rhex_controller::RhexControllerSimple _controller;
        robot_t _robot;

        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;
        Eigen::VectorXd _K_p;
        Eigen::VectorXd _K_d;

    };
}

#endif
