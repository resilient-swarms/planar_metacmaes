#ifndef PLANAR_DART_PLANAR_HPP
#define PLANAR_DART_PLANAR_HPP

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <string>
#include <fstream>
#include <streambuf>

#include <dart/utils/SkelParser.hpp>
#include <dart/utils/sdf/SdfParser.hpp>

#define JOINT_SIZE 8

namespace planar_dart {

    struct planarDamage {
        planarDamage() {}
        planarDamage(const std::string& type, const std::string& data, void* extra = nullptr) : type(type), data(data), extra(extra) {}

        std::string type;
        std::string data;
        void* extra = nullptr;
    };

    class planar {
    public:
     
        planar() {
           
        }

        planar(const std::string& model_file, const std::string& robot_name, const std::vector<std::pair<std::string, std::string>>& packages, bool is_urdf_string, std::vector<planarDamage> damages)
        {
            //assert(_skeleton != nullptr);
            _robot_name = robot_name;
            _skeleton = _load_model(robot_name, model_file, packages, is_urdf_string);

            // lock_hip_joints();
            _set_damages(damages);
            _set_gripper();

            // Set all coefficients to default values
            //set_friction_coeff();
            //set_restitution_coeff();
        }

        planar(const std::string& model_file, const std::string& robot_name, bool is_urdf_string, std::vector<planarDamage> damages) : planar(model_file, robot_name, std::vector<std::pair<std::string, std::string>>(), is_urdf_string, damages) {
            _set_gripper();
        }

        planar(dart::dynamics::SkeletonPtr skeleton, std::vector<planarDamage> damages)
        {
            //assert(_skeleton != nullptr);

             _skeleton = skeleton;
            _set_damages(damages);
            _set_gripper();

        }

        ~planar(){

        }

        std::shared_ptr<planar> clone() const
        {
            // safely clone the skeleton
            _skeleton->getMutex().lock();
            auto tmp_skel = _skeleton->clone();
            _skeleton->getMutex().unlock();
            auto _planar = std::make_shared<planar>();
            _planar->_skeleton = tmp_skel;
            _planar->_damages = _damages;
            _planar->_stuck_joints = _stuck_joints;
            _planar->_gripper_body = _gripper_body;
            _planar->_joint_1_marker = _joint_1_marker; 
            _planar->_joint_2_marker = _joint_2_marker; 
            _planar->_joint_3_marker = _joint_3_marker; 
            _planar->_joint_4_marker = _joint_4_marker; 
            _planar->_joint_5_marker = _joint_5_marker; 
            _planar->_joint_6_marker = _joint_6_marker; 
            _planar->_joint_7_marker = _joint_7_marker; 
            _planar->_joint_8_marker = _joint_8_marker; 
            return _planar;
        }

        dart::dynamics::SkeletonPtr skeleton()
        {
            return _skeleton;
        }

        bool is_stuck(int joint) const
        {
            for (size_t j = 0; j < _stuck_joints.size(); j++) {
                if (joint == _stuck_joints[j]) {
                    return true;
                }
            }
            return false;
        }
        // note that default arguments are createMarker(
        // const std::string& name = "marker",
        // const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
        // const Eigen::Vector4d& color = Eigen::Vector4d::Constant(1.0))
        dart::dynamics::Marker* gripper(const std::string& tag){
            return _skeleton->getBodyNode("link_8")->createMarker(std::string("grippermarker" + tag));
        }

        dart::dynamics::Marker* joint(int i,const std::string& tag){
        
            return _skeleton->getBodyNode("link_" + std::to_string(i+1))->createMarker(std::string("link_" + std::to_string(i+1)+"marker" + tag));
        }

        std::vector<int> stuck_joints() const
        {
            return _stuck_joints;
        }

        std::vector<planarDamage> damages() const
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

        void lock_joints() {
            for(size_t i = 0; i < JOINT_SIZE; i++){
                auto jnt = _skeleton->getJoint("joint_" + std::string(1, i +'0'));
                jnt->setActuatorType(dart::dynamics::Joint::LOCKED);
            }
        }

				//make it stuck at 45
		void lock_single_joint(size_t index) {
                auto jnt = _skeleton->getJoint("joint_" + std::string(1, index +'0'));
                jnt->setActuatorType(dart::dynamics::Joint::LOCKED);
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

        void _set_damages(const std::vector<planarDamage>& damages)
        {
            _stuck_joints.clear();
            _damages = damages;
            /*for (auto dmg : _damages) {
                if (dmg.type == "stuck_at_45") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _stuck_joints.push_back(l);

                        lock_single_joint(l);
                    }
                }
                if (dmg.type == "stuck_at_minus45") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _stuck_joints.push_back(l);

                        lock_single_joint(l);
                    }
                }
            }*/

            std::sort(_stuck_joints.begin(), _stuck_joints.end());
        }

        void _set_gripper() {
            std::string gripper_index = "link_8";
            _gripper_body = (_skeleton->getMarker(std::string("gripMarker"))==NULL)?_skeleton->getBodyNode(gripper_index)->createMarker(std::string("gripMarker")):_skeleton->getMarker(std::string("gripMarker"));

            _joint_1_marker = (_skeleton->getMarker(std::string("oneMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_1"))->createMarker(std::string("oneMarker")):_skeleton->getMarker(std::string("oneMarker"));
            _joint_2_marker = (_skeleton->getMarker(std::string("twoMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_2"))->createMarker(std::string("twoMarker")):_skeleton->getMarker(std::string("twoMarker"));
            _joint_3_marker = (_skeleton->getMarker(std::string("threeMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_3"))->createMarker(std::string("threeMarker")):_skeleton->getMarker(std::string("threeMarker"));
            _joint_4_marker = (_skeleton->getMarker(std::string("fourMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_4"))->createMarker(std::string("fourMarker")):_skeleton->getMarker(std::string("fourMarker"));
            _joint_5_marker = (_skeleton->getMarker(std::string("fiveMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_5"))->createMarker(std::string("fiveMarker")):_skeleton->getMarker(std::string("fiveMarker"));
            _joint_6_marker = (_skeleton->getMarker(std::string("sixMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_6"))->createMarker(std::string("sixMarker")):_skeleton->getMarker(std::string("sixMarker"));
            _joint_7_marker = (_skeleton->getMarker(std::string("sevenMarker"))==NULL)?_skeleton->getBodyNode(std::string("link_7"))->createMarker(std::string("sevenMarker")):_skeleton->getMarker(std::string("sevenMarker"));
            _joint_8_marker = _gripper_body;
        }

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<planarDamage> _damages;
        std::vector<int> _stuck_joints;
        std::string _robot_name;
        dart::dynamics::Marker* _gripper_body;
        dart::dynamics::Marker* _joint_1_marker; 
        dart::dynamics::Marker* _joint_2_marker; 
        dart::dynamics::Marker* _joint_3_marker;
        dart::dynamics::Marker* _joint_4_marker;
        dart::dynamics::Marker* _joint_5_marker;
        dart::dynamics::Marker* _joint_6_marker;
        dart::dynamics::Marker* _joint_7_marker;
        dart::dynamics::Marker* _joint_8_marker;
    };
}

#endif
