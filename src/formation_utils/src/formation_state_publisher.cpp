#include <formation_utils/formation_state_publisher.h>

namespace FormationUtils {

    bool FormationStatePublisher::init() {
        std::string robot_desc;
        if(fh->nh.hasParam("robot_description")) {
            fh->nh.getParam("robot_description", robot_desc);
        }
        else {
            ROS_ERROR("param robot_description not advertised. Make sure robot_description is set in the launch file.");
        }
        urdf::Model robot_model;
        if(!robot_model.initString(robot_desc)){
            ROS_ERROR("Failed to parse URDF robot model from XML String.");
            return false;
        };
        
        if(!kdl_parser::treeFromUrdfModel(robot_model, robot_desc_KDL_tree)) {
            ROS_ERROR("Conversion from URDF::Model to KDL::Tree failed.");
            return false;
        };

        if(!fh->nh.getParam("/formation_config/num_bots", num_bots)) {
            ROS_ERROR("Failed to fetch number of robots from parameter server. Make sure /formation_config/num_bots is defined.");
            return false;
        }
        robotStatePublisher = robot_state_publisher::RobotStatePublisher(robot_desc_KDL_tree);
        uid_list_ptr = fh->getUIDListPtr();
        if(!fh->nh.getParam("/robot_state_publisher/USE_STATIC_TF", USE_STATIC_TF)) {
            ROS_ERROR("Failed to fetch the USE_STATIC_TF parameter for fixed transforms.");
            return false;
        }
        return true;
    }

    bool FormationStatePublisher::publishFixedTransforms(){
        for (int i = 0; i < num_bots; i++) {
            robotStatePublisher.publishFixedTransforms(uid_list_ptr->at(i), USE_STATIC_TF);
        }
        return true;
    }

    bool FormationStatePublisher::publishTransforms(const std::vector<std::map<std::string, double>>& joint_positions, const ros::Time& time){
        for (int i = 0; i < num_bots; i++) {
            std::string current_uid = uid_list_ptr->at(i);
            fh->nh.setParam("tf_prefix", current_uid);
            robotStatePublisher.publishTransforms(joint_positions[i], time);
        }
        return true;
    }

}