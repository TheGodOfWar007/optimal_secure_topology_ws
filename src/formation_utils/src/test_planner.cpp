#include <formation_utils/test_planner.h>

namespace FormationUtils {
    void TestMotionPlanner2D::load_params() {
        nh.getParam("/test_motion_planner/num_bots", num_bots);
        nh.getParam("/test_motion_planner/uid_list", uid_list);
        nh.getParam("/test_motion_planner/USE_STATIC_TRANSFORM", USE_STATIC_TRANSFORM);
        nh.getParam("/test_motion_planner/planner_center", planner_center);
        XmlRpc::XmlRpcValue FormationShapeConfig;
        nh.getParam("/test_motion_planner/static_formation", FormationShapeConfig);
        xmlrpc_to_matrix<Eigen::MatrixXdRowMajor>(num_bots, 2, FormationShapeConfig, formation_shape);
    }

    void TestMotionPlanner2D::publishShapeTransforms() {
        std::vector<std::string>::iterator leader_it;
        leader_it = std::find(uid_list.begin(), uid_list.end(), planner_center);
        int leader_idx = std::distance(uid_list.begin(), leader_it);
        ROS_ASSERT(leader_idx < uid_list.size());
        std::vector<geometry_msgs::TransformStamped> tf_transforms;
        geometry_msgs::TransformStamped tf_transform;
        for(int i = 0; i < num_bots; i++) {
            if (i == leader_idx) {
                continue;
            }
            tf_transform.header.frame_id = planner_center;
            tf_transform.child_frame_id = uid_list[i];

            tf::vectorEigenToMsg(Eigen::Vector3d(formation_shape(i,0) - formation_shape(leader_idx,0), formation_shape(i,1) - formation_shape(leader_idx,1), 0), tf_transform.transform.translation);
            tf_transform.transform.rotation = tf2::toMsg(Eigen::Quaterniond::Identity());
            
            tf_transforms.push_back(tf_transform);
        }

        if(USE_STATIC_TRANSFORM) {
            static_tf_broadcaster.sendTransform(tf_transforms);
        }
        else {
            tf_broadcaster.sendTransform(tf_transforms);
        }
    }

    void TestMotionPlanner2D::advertiseCircularTrajectory2D(Eigen::Vector2d center, Eigen::Vector2d start_point, double radius, double time_period, double rate, bool CONTINUE_LOOPING) {
        
    }
}