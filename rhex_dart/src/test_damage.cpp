
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

int main(int argc, char** argv)
{
    // using the same model as the hexapod and so the robot has a damages parameter but is set to 0

    std::vector<rhex_dart::RhexDamage> damages(0);


    damages.push_back(rhex_dart::RhexDamage("leg_removal", "1"));
	

    // loads the robot with name Rhex tels it that it is not a URDF file and give it the blank damages
    // raised.skel, skinny.skel, Rhex8.skel
    auto global_robot = std::make_shared<rhex_dart::Rhex>(std::string(std::getenv("RESIBOTS_DIR")) + "/share/rhex_models/SKEL/" + argv[2], "Rhex", false, damages);

    // sets the control vector up
    // ./waf && ./build/test 1 raised.skel 1 0.1 0 0.3 0.85 0.6 0 0.15 1 0.75 0.5 0.8
    std::vector<double> ctrl = {atof(argv[3]), atof(argv[4]), atof(argv[5]),
                                atof(argv[6]), atof(argv[7]), atof(argv[8]),
                                atof(argv[9]), atof(argv[10]), atof(argv[11]),
                                atof(argv[12]), atof(argv[13]), atof(argv[14])};

    using desc_t = boost::fusion::vector<rhex_dart::descriptors::DutyCycle, rhex_dart::descriptors::BodyOrientation>;

    using viz_t = boost::fusion::vector<rhex_dart::visualizations::HeadingArrow, rhex_dart::visualizations::PointingArrow<Params>>;
    rhex_dart::RhexDARTSimu<rhex_dart::desc<desc_t>, rhex_dart::viz<viz_t>> simu(ctrl, global_robot, atof(argv[1]), damages);

#ifdef GRAPHIC
    simu.fixed_camera(Eigen::Vector3d(3, 0, 0.5));
    simu.follow_rhex();
#endif
    simu.run(15);

    std::cout << "Covered distance | Arrival angle | Body avg height" << std::endl;
    std::cout << simu.covered_distance() << " " << simu.arrival_angle() << " " << simu.body_avg_height() << std::endl;

    std::cout << "Energy" << std::endl;
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
