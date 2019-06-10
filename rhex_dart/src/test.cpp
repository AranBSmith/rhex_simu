#include <iostream>
#include <cstdlib>
#include <rhex_dart/rhex_dart_simu.hpp>

struct Params {
    static constexpr double radius() { return 0.01; }

    static Eigen::Vector3d head() { return Eigen::Vector3d(1, 1, 0.5); }

    static Eigen::Vector3d tail() { return Eigen::Vector3d(1, 0, 0.5); }

    static Eigen::Vector4d color() { return Eigen::Vector4d(1, 0, 0, 1); }

    static std::string skel_name() { return "floor"; }

    static std::string body_name() { return "BodyNode"; }
};

int main()
{
    //using the same model as the hexapod and so the robot has a damages parameter but is set to 0
    std::vector<rhex_dart::RhexDamage> brk = {};

    //loads the robot with name Rhex tels it that it is not a URDF file and give it the blank damages
    auto global_robot = std::make_shared<rhex_dart::Rhex>("RHex8.skel","Rhex",false,brk);

    // sets the control vector up
    std::vector<double> ctrl = {0.5, 0.95, 0.5, 0.95, 0.5, 0.9, 0.1, 5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    // ctrl = {1, 0.85, 0.95, 1, 0.35, 0.2, 0.85, 0.05, 0.3, 0.95, 0.85, 0.4, 0.8, 0.55, 0.15, 0.25, 0.15, 0.85, 0.85, 0.85, 0.6, 0.5, 0.95, 0.05, 1, 0.25, 0.8, 0.75, 0.9, 0.3, 0.95, 0.3, 0.3, 0.65, 0.7, 0.75};
    // ctrl = {0.9, 1.0, 1.0, 0.75, 0.4, 0.95, 0.8, 0.9, 0.15, 0.2, 0.9, 0.15, 0.85, 0.3, 0.4, 0.8, 0.8, 0.7, 1.0, 0.9, 0.65, 0.1, 0.6, 0.4, 1.0, 0.75, 0.8, 0.8, 0.8, 0.75, 0.35, 0.35, 0.5, 0.8, 0.75, 0.85};
    // ctrl = {0.75, 0.9, 0.7, 0.75, 0.9, 0, 1, 0.1, 0.35, 0.2, 0.1, 0.15, 0.95, 0.25, 0.15, 0, 0.15, 0.15, 0.85, 0.95, 0.6, 0.9, 1, 0.2, 0.9, 0.85, 0.8, 0.25, 0.6, 0.95, 1, 0, 0.6, 0.3, 0.85, 0.75};
    // ctrl = {0.05, 0.8, 0.1, 0.9, 0.7, 0.35, 0.9, 1, 0.3, 0.7, 0.05, 0.15, 0.95, 0.75, 0.7, 0.95, 0.6, 0.6, 0.75, 0.7, 0.95, 0.6, 0.3, 0.35, 1, 0.4, 0.65, 0.55, 0.25, 0.6, 0.9, 0.75, 0.4, 1, 0.7, 0.85};

    using desc_t = boost::fusion::vector<rhex_dart::descriptors::DutyCycle, rhex_dart::descriptors::BodyOrientation>;

    using viz_t = boost::fusion::vector<rhex_dart::visualizations::HeadingArrow, rhex_dart::visualizations::PointingArrow<Params>>;

    rhex_dart::RhexDARTSimu<rhex_dart::desc<desc_t>, rhex_dart::viz<viz_t>> simu(ctrl, global_robot);

#ifdef GRAPHIC
    simu.fixed_camera(Eigen::Vector3d(0, 1, 5));
#endif

    simu.run(5);
    std::cout << simu.covered_distance() << " " << simu.arrival_angle() << " "<<simu.body_avg_height()<<std::endl;
    std::cout << simu.energy() << std::endl;
    std::vector<double> v;
    simu.get_descriptor<rhex_dart::descriptors::DutyCycle>(v);
	
	std::cout << "Duty Cycle:" << std::endl;
    for (size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    
    std::cout << std::endl;
    std::vector<double> vv;
    simu.get_descriptor<rhex_dart::descriptors::BodyOrientation>(vv);
    
    std::cout << "Body Orientation" << std::endl;
    for (size_t i = 0; i < vv.size(); i++) {
        std::cout << vv[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}
