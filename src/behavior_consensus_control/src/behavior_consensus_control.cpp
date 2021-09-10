#include <behavior_consensus_control/behavior_consensus_control.h>

namespace FormationControl {

    void BehaviorConsensus2D::init()  {
        setUidList(uid_list);
        ros::service::waitForService("/formation_graph/interface", -1);
        graph_interface_client = nh.serviceClient<formation_msgs::FormationGraphParams>("/formation_graph/interface");
        formation_msgs::FormationGraphParams graph_interface_msg;

        graph_interface_msg.request.REQUESTING_PARAMS = true;
        graph_interface_client.call(graph_interface_msg);

        if(!graph_interface_msg.response.success) {
            ROS_ERROR("Fetching the graph parameters failed.");
        }

        FormationUtils::arrayMsgtoMatrixEigen<Eigen::MatrixXdRowMajor>(graph_interface_msg.response.A_, A);
        FormationUtils::arrayMsgtoMatrixEigen<Eigen::MatrixXdRowMajor>(graph_interface_msg.response.L_, L);

        for (int i = 0; i < num_bots; i++) {
            nav_msgs::Odometry empty_odom_msg;
            formation_msgs::PoseUID empty_traj_msg;
            bot_odoms.push_back(empty_odom_msg);
            bot_odom_poses.push_back(empty_odom_msg.pose.pose);
            traj_goals.push_back(empty_traj_msg);
            std::string odom_topic_str = uid_list[i] + "/" + odom_topic_name;
            std::string traj_topic_str = uid_list[i] + "/" + traj_topic_name;
            std::string cmd_vel_topic_name = uid_list[i] + "/cmd_vel";
            ros_subscriber = nh.subscribe(odom_topic_str, odom_queue_size, &BehaviorConsensus2D::odomSubscriberCallback, this);
            odom_sub.push_back(ros_subscriber);
            ros_subscriber = nh.subscribe(traj_topic_str, traj_queue_size, &BehaviorConsensus2D::trajSubscriberCallback, this);
            traj_sub.push_back(ros_subscriber);
            ros_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, cmd_vel_queue_size);
            cmd_vel_pub.push_back(ros_publisher);
        }
    }

    void BehaviorConsensus2D::odomSubscriberCallback(const nav_msgs::Odometry &odom_msg){
        // Depending on the message metadata and assuming that the right namespace is
        // used for the odom topic of the bot. We will push the update at the right
        // position in the Odometry messages vector. The default frame name for odometry
        // is /odom and is assumed to be the default here as well.
        std::string uid = deleteSubstr(odom_msg.header.frame_id, "/odom");
        int idx = FormationUtils::findVectorElement<std::string>(uid_list, uid);
        bot_odoms[idx] = odom_msg;
        bot_odom_poses[idx] = odom_msg.pose.pose;
    }

    void BehaviorConsensus2D::trajSubscriberCallback(const formation_msgs::PoseUID &traj_msg) {
        std::string uid = traj_msg.uid;
        int idx = FormationUtils::findVectorElement<std::string>(uid_list, uid);
        traj_goals[idx] = traj_msg;
    }

    void BehaviorConsensus2D::applyControlLaw(){
        // Step 1: Transforming the pose of the robots to the projection point.
        BotToPointTfByUID<geometry_msgs::Pose>(bot_odom_poses);
        // Step 2: We will now apply the consensus law on the points which follow
        // single integrator dynamics.
        std::vector<Eigen::Vector2d> bot_twists_2d(num_bots);
        std::vector<Eigen::Vector2d> u_i_point(num_bots);
        std::vector<double> theta_vec(num_bots);
        Eigen::Vector2d p_i;
        Eigen::Vector2d p_i_st;
        Eigen::Vector2d p_j;
        Eigen::Vector2d p_j_st;
        for (int i = 0; i < num_bots; i++) {
            Eigen::Vector2d consensus_term = Eigen::Vector2d::Zero();
            // The backstepping term should be replacable by a PID term.
            // Since it is the one responsible for the trajectory tracking action.
            Eigen::Vector2d backstep_term;
            
            p_i.x() = bot_odom_poses[i].position.x;
            p_i.y() = bot_odom_poses[i].position.y;
            p_i_st.x() = traj_goals[i].pose.position.x;
            p_i_st.y() = traj_goals[i].pose.position.y;
            for (int j = 0; j < num_bots; j++) {
                p_j.x() = bot_odom_poses[j].position.x;
                p_j.y() = bot_odom_poses[j].position.y;
                p_j_st.x() = traj_goals[j].pose.position.x;
                p_j_st.y() = traj_goals[j].pose.position.y;
                consensus_term = consensus_term + A(i,j)*((p_i - p_i_st) - (p_j - p_j_st));
            }
            backstep_term = beta*(p_i - p_i_st);
            u_i_point[i] = -backstep_term - consensus_term;
            Eigen::Vector3d rpy = extractRPYfromQuaternionMsg(bot_odom_poses[i].orientation);
            theta_vec[i] = rpy(2); // Extracting yaw from r,p,y.
            // u_i_point is in si dynamics. We will now change the single integrator control input
            // back to unicycle dynamics control input and use it to populate the cmd_vel vector.
            bot_twists_2d[i] = SiToUniDynamicsTwistTf(u_i_point[i], theta_vec[i]);

            // Now filling the cmd_vel messages appropriately. Note that the twist 2d vector consists
            // of V w.r.t base_footprint x frame i.e. bot forward direction and omega i.e. about z axis
            // angular velocity component.
            cmd_vel[i].linear.x = bot_twists_2d[i].x();
            cmd_vel[i].linear.y = 0;
            cmd_vel[i].linear.z = 0;
            cmd_vel[i].angular.x = 0;
            cmd_vel[i].angular.y = 0;
            cmd_vel[i].angular.z = bot_twists_2d[i].y();
        }

        // Step 3: Now publishing the cmd_vel of all the bots together.
        for (int i = 0; i < num_bots; i++){
            cmd_vel_pub[i].publish(cmd_vel[i]);
        }
    }
}