
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

    assert(argc == 28);
    damages.push_back(rhex_dart::RhexDamage("leg_removal", "5"));
	

    // loads the robot with name Rhex tels it that it is not a URDF file and give it the blank damages
    // raised.skel, skinny.skel, Rhex8.skel
    auto global_robot = std::make_shared<rhex_dart::Rhex>(std::string(std::getenv("RESIBOTS_DIR")) + "/share/rhex_models/SKEL/" + argv[3], "Rhex", false, damages);

    // sets the control vector up
    std::vector<double> ctrl = { atof(argv[4]), atof(argv[5]),
                                atof(argv[6]), atof(argv[7]), atof(argv[8]),
                                atof(argv[9]),atof(argv[10]),atof(argv[11]),
                                atof(argv[12]),atof(argv[13]),atof(argv[14]),
                                atof(argv[15]),atof(argv[16]),atof(argv[17]),
                                atof(argv[18]),atof(argv[19]),atof(argv[20]),
                                atof(argv[21]),atof(argv[22]),atof(argv[23]),
                                atof(argv[24]),atof(argv[25]),atof(argv[26])};

    using desc_t = boost::fusion::vector<rhex_dart::descriptors::DutyCycle,
                    rhex_dart::descriptors::BodyOrientation,
                    rhex_dart::descriptors::SpecificResistance,
                    rhex_dart::descriptors::AvgCOMVelocities>;

    using viz_t = boost::fusion::vector<rhex_dart::visualizations::HeadingArrow, rhex_dart::visualizations::PointingArrow<Params>>;
    rhex_dart::RhexDARTSimu<rhex_dart::desc<desc_t>, rhex_dart::viz<viz_t>> simu(ctrl, global_robot, atof(argv[1]), atof(argv[2]), damages);

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

    double sr;
    simu.get_descriptor<rhex_dart::descriptors::SpecificResistance>(sr);
    std::cout << "Specific Resistance: " << sr << std::endl;

    Eigen::Vector3d vels;
    simu.get_descriptor<rhex_dart::descriptors::AvgCOMVelocities>(vels);

    std::cout << "Avg COM Velocities" << std::endl;
    for (size_t i = 0; i < vels.size(); i++) {
        std::cout << vels[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}
