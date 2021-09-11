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

        planner_center_idx = findVectorElement<std::string>(uid_list, planner_center);
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

    void TestMotionPlanner2D::init(){
        for (int i = 0; i < num_bots; i++) {
            ros::Publisher traj_pub;
            formation_msgs::PoseUID empty_pose_uid_msg;
            std::string traj_topic_str = uid_list[i] + "/" + traj_topic_name;
            traj_pub = nh.advertise<formation_msgs::PoseUID>(traj_topic_str, traj_queue_size);
            traj_pub_vec.push_back(traj_pub);
            traj_goals.push_back(empty_pose_uid_msg);
        }
    }

    void TestMotionPlanner2D::updateTfBuffers() {
        tf2_ros::TransformListener tf_listener(tf_buffer);
        for (int i = 0; i < num_bots; i++) {
            if (i == planner_center_idx) {
                continue;
            }
            tf_vector[i] = tf_buffer.lookupTransform(planner_center, uid_list[i], ros::Time::now());
        }
    }

    void TestMotionPlanner2D::transformAndPublishTrajectory() {
        traj_goals[planner_center_idx] = planner_center_goal;
        // Transforming the coords to other bots;
        for (int i = 0; i < num_bots; i++) {
            if (i == planner_center_idx) {
                continue;
            }
            traj_goals[i] = planner_center_goal;
            tf2::doTransform<geometry_msgs::Pose>(traj_goals[i].pose, traj_goals[i].pose, tf_vector[i]);
        }
        // Publishing th trajectories now.
        for (int i = 0; i < num_bots; i++) {
            traj_pub_vec[i].publish<formation_msgs::PoseUID>(traj_goals[i]);
        }
    }

    void CircleTrajectory2D::generateNextWaypoint(){
        if (!_stop_iteration) {
            if (planner_iteration == 0) {
                t_ini = ros::Time::now();
            }
            t_curr = ros::Time::now();
            t = (t_curr - t_ini).toSec();
            // Generating using the parametric law.
            last_coord = current_coord;
            theta_current = theta_initial + omega*t;
            current_coord.x() = radius*cos(theta_current) + circle_center.x();
            current_coord.y() = radius*sin(theta_current) + circle_center.y();
            // If Looping around the circle is disabled then continue broadcasting the end point.
            if(!CONTINUE_LOOPING) {
                if(abs(theta_current - theta_initial) < comparison_multiplier*omega) {
                    _stop_iteration = true;
                    ROS_INFO("Circle Loop Tolerance hit, stopping further iterations since CONTINUE_LOOPING is not set.");
                }
            }
            planner_center_goal.pose.position.x = current_coord.x();
            planner_center_goal.pose.position.y = current_coord.y();
            planner_center_goal.pose.position.z = 0;
            tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), planner_center_goal.pose.orientation);
        }
        else {
            current_coord = last_coord;
        }
    }
}