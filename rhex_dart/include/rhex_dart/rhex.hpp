#ifndef RHEX_DART_RHEX_HPP
#define RHEX_DART_RHEX_HPP

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <string>
#include <fstream>
#include <streambuf>

#include <dart/utils/SkelParser.hpp>
#include <dart/utils/sdf/SdfParser.hpp>

namespace rhex_dart {

    struct RhexDamage {
        RhexDamage() {}
        RhexDamage(const std::string& type, const std::string& data, void* extra = nullptr) : type(type), data(data), extra(extra) {}

        std::string type;
        std::string data;
        void* extra = nullptr;
    };

    class Rhex {
    public:
        Rhex() {}

        Rhex(const std::string& model_file, const std::string& robot_name, const std::vector<std::pair<std::string, std::string>>& packages, bool is_urdf_string, std::vector<RhexDamage> damages) : _robot_name(robot_name), _skeleton(_load_model(robot_name, model_file, packages, is_urdf_string))
        {
            assert(_skeleton != nullptr);

            // lock_hip_joints();
            _set_damages(damages);

            // Set all coefficients to default values
            set_friction_coeff();
            set_restitution_coeff();
        }

        Rhex(const std::string& model_file, const std::string& robot_name, bool is_urdf_string, std::vector<RhexDamage> damages) : Rhex(model_file, robot_name, std::vector<std::pair<std::string, std::string>>(), is_urdf_string, damages) {}

        Rhex(dart::dynamics::SkeletonPtr skeleton, std::vector<RhexDamage> damages) : _skeleton(skeleton)
        {
            assert(_skeleton != nullptr);
            _set_damages(damages);

        }

        std::shared_ptr<Rhex> clone() const
        {
            // safely clone the skeleton
            _skeleton->getMutex().lock();
            auto tmp_skel = _skeleton->clone();
            _skeleton->getMutex().unlock();
            auto rhex = std::make_shared<Rhex>();
            rhex->_skeleton = tmp_skel;
            rhex->_damages = _damages;
            rhex->_broken_legs = _broken_legs;
            return rhex;
        }

        dart::dynamics::SkeletonPtr skeleton()
        {
            return _skeleton;
        }

        bool is_broken(int leg) const
        {
            for (size_t j = 0; j < _broken_legs.size(); j++) {
                if (leg == _broken_legs[j]) {
                    return true;
                }
            }
            return false;
        }

        std::vector<int> broken_legs() const
        {
            return _broken_legs;
        }

        std::vector<RhexDamage> damages() const
        {
            return _damages;
        }

        Eigen::Vector3d pos()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            return {pos_and_rot(3), pos_and_rot(4), pos_and_rot(5)};
        }

        Eigen::Vector3d rot()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            return {pos_and_rot(0), pos_and_rot(1), pos_and_rot(2)};
        }

        Eigen::Vector6d pose()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            Eigen::Vector6d tmp;
            tmp << pos_and_rot(0), pos_and_rot(1), pos_and_rot(2), pos_and_rot(3), pos_and_rot(4), pos_and_rot(5);
            return tmp;
        }

        void set_friction_coeff(double friction = 1.0)
        {
            if (friction < 0.0)
                return;

            for (size_t i = 0; i < _skeleton->getNumBodyNodes(); i++) {
                auto bd = _skeleton->getBodyNode(i);
                bd->setFrictionCoeff(friction);
            }
        }

        double get_friction_coeff()
        {
            return _skeleton->getBodyNode(0)->getFrictionCoeff();
        }

        void set_restitution_coeff(double restitution = 0.0)
        {
            if (restitution < 0.0 || restitution > 1.0)
                return;

            for (size_t i = 0; i < _skeleton->getNumBodyNodes(); i++) {
                auto bd = _skeleton->getBodyNode(i);
                bd->setRestitutionCoeff(restitution);
            }
        }

        double get_restitution_coeff()
        {
            return _skeleton->getBodyNode(0)->getRestitutionCoeff();
        }

        void lock_hip_joints() {
            for(size_t i = 0; i < 6; i++){
                auto jnt = _skeleton->getJoint("body_joint_" + std::string(1, i +'0'));
                jnt->setActuatorType(dart::dynamics::Joint::LOCKED);
            }
        }

        void unlock_hip_joints() {
            for(size_t i = 0; i < 6; i++){
                auto jnt = _skeleton->getJoint("body_joint_" + std::string(1, i +'0'));
                jnt->setActuatorType(dart::dynamics::Joint::FORCE);
            }
        }

    protected:
        dart::dynamics::SkeletonPtr _load_model(const std::string& robot_name, const std::string& filename, const std::vector<std::pair<std::string, std::string>>& packages, bool is_urdf_string)
        {
            // useful for knowing if you are running the latest version
            std::cout << "Version: 2307.1" << std::endl;

            // Remove spaces from beginning of the filename/path
            std::string model_file = filename;

            model_file.erase(model_file.begin(), std::find_if(model_file.begin(), model_file.end(), [](int ch) {
                    return !std::isspace(ch);
            }));

            if (model_file[0] != '/') {
                constexpr size_t max_size = 512;
                char buff[max_size];
                model_file = std::string(buff) + "/" + model_file;
            }

            dart::dynamics::SkeletonPtr tmp_skel;
            if (!is_urdf_string) {
                std::string extension = model_file.substr(model_file.find_last_of(".") + 1);
                if (extension == "urdf") {
                    dart::utils::DartLoader loader;
                    for (size_t i = 0; i < packages.size(); i++) {
                        loader.addPackageDirectory(std::get<0>(packages[i]), std::get<1>(packages[i]));
                    }
                    tmp_skel = loader.parseSkeleton(model_file);
                }
                else if (extension == "sdf")
                    tmp_skel = dart::utils::SdfParser::readSkeleton(model_file);
                else if (extension == "skel") {
                    tmp_skel = dart::utils::SkelParser::readSkeleton(model_file);
                    // if the skel file contains a world
                    // try to read the skeleton with name 'robot_name'
                    if (!tmp_skel) {
                        dart::simulation::WorldPtr world = dart::utils::SkelParser::readWorld(model_file);
                        tmp_skel = world->getSkeleton(robot_name);
                    }

                    for (size_t i = 0; i < tmp_skel->getNumJoints(); ++i) {
                        tmp_skel->getJoint(i)->setPositionLimitEnforced(true);
                    }

                }

                else
                    return nullptr;
            }
            else {
                // Load from URDF string
                dart::utils::DartLoader loader;
                for (size_t i = 0; i < packages.size(); i++) {
                    loader.addPackageDirectory(std::get<0>(packages[i]), std::get<1>(packages[i]));
                }
                tmp_skel = loader.parseSkeletonString(filename, "");
            }

            if (tmp_skel == nullptr){
                std::cout << "returning null pointer" << std::endl;
                return nullptr;
            }

            tmp_skel->setName(robot_name);

            // Set joint limits
            for (size_t i = 0; i < tmp_skel->getNumJoints(); ++i) {
                tmp_skel->getJoint(i)->setPositionLimitEnforced(true);
            }


            // Fix for mesh materials
            for (size_t i = 0; i < tmp_skel->getNumBodyNodes(); ++i) {
                dart::dynamics::BodyNode* bn = tmp_skel->getBodyNode(i);
                for (size_t j = 0; j < bn->getNumShapeNodes(); ++j) {
                    dart::dynamics::ShapeNode* sn = bn->getShapeNode(j);
                    if (sn->getVisualAspect()) {
                        dart::dynamics::MeshShape* ms = dynamic_cast<dart::dynamics::MeshShape*>(sn->getShape().get());
                        if (ms)
                            ms->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
                    }
                }
            }
            return tmp_skel;
        }

        void _set_damages(const std::vector<RhexDamage>& damages)
        {
            _broken_legs.clear();
            _damages = damages;
            for (auto dmg : _damages) {
                if (dmg.type == "leg_removal") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _broken_legs.push_back(l);

                        std::string leg_bd_name = "leg_" + std::string(1, dmg.data[i]) + "_1";
                        auto bd = _skeleton->getBodyNode(leg_bd_name);
                        bd->removeAllShapeNodes();
                        bd->remove();
                    }
                }
                else if (dmg.type == "leg_shortening") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _broken_legs.push_back(l);

                        std::string leg_bd_name = "leg_" + std::string(1, dmg.data[i]) + "_6";
                        auto bd = _skeleton->getBodyNode(leg_bd_name);
                        bd->removeAllShapeNodes();
                        bd->remove();
                    }
                }
                else if (dmg.type == "blocked_joint") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _broken_legs.push_back(l);

                        auto jnt = _skeleton->getJoint("body_joint_" + std::string(1, dmg.data[i]));
                        if (dmg.extra)
                            jnt->setPosition(0, *((double*)dmg.extra));
                        jnt->setActuatorType(dart::dynamics::Joint::LOCKED);
                    }
                }
                else if (dmg.type == "free_joint") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _broken_legs.push_back(l);

                        _skeleton->getJoint("body_joint_" + std::string(1, dmg.data[i]))->setActuatorType(dart::dynamics::Joint::PASSIVE);
                    }
                }
            }

            std::sort(_broken_legs.begin(), _broken_legs.end());
        }

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<RhexDamage> _damages;
        std::vector<int> _broken_legs;
        std::vector<int> _removed_joints;
        std::string _robot_name;
    };
}

#endif
