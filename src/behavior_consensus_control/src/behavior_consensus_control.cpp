#include <behavior_consensus_control/behavior_consensus_control.h>
#include <tf2_eigen/tf2_eigen.h>

namespace FormationControl {

    void BehaviorConsensus2D::init()  {
        num_bots = uid_list.size();
        ros::service::waitForService("/formation_graph/interface", -1);
        graph_interface_client = nh.serviceClient<formation_msgs::FormationGraphParams>("/formation_graph/interface");
        formation_msgs::FormationGraphParams graph_interface_msg;

        graph_interface_msg.request.REQUESTING_PARAMS = true;
        graph_interface_msg.request.UPDATING_ADJACENCY_MATRIX = false;
        graph_interface_msg.request.A_.layout.data_offset = 0;
        graph_interface_msg.request.A_.data.push_back(0);
        std_msgs::MultiArrayDimension dim_;
        dim_.label = '\0';
        dim_.size = 0;
        dim_.stride = 0;
        graph_interface_msg.request.A_.layout.dim.push_back(dim_);
        graph_interface_client.call(graph_interface_msg);

        if(!graph_interface_msg.response.success) {
            ROS_ERROR("Fetching the graph parameters failed.");
        }
        ROS_INFO("Graph Parameters received from interface service.");

        FormationUtils::arrayMsgtoMatrixEigen<std_msgs::Float64MultiArray, Eigen::MatrixXdRowMajor>(graph_interface_msg.response.A_, A);
        FormationUtils::arrayMsgtoMatrixEigen<std_msgs::Float64MultiArray, Eigen::MatrixXdRowMajor>(graph_interface_msg.response.L_, L);

        nav_msgs::Odometry empty_odom_msg;
        tf::pointEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.pose.pose.position);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        empty_odom_msg.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond::Identity());
        geometry_msgs::Pose empty_pose_msg;
        empty_pose_msg = empty_odom_msg.pose.pose;
        formation_msgs::PoseUID empty_traj_msg;
        empty_traj_msg.pose = empty_pose_msg;
        empty_traj_msg.twist = empty_odom_msg.twist.twist;
        bot_odoms.resize(num_bots);
        bot_odom_poses.resize(num_bots);
        traj_goals.resize(num_bots);
        cmd_vel.resize(num_bots);

        for (int i = 0; i < num_bots; i++) {
            
            bot_odoms[i] = empty_odom_msg;
            bot_odom_poses[i] = empty_pose_msg;
            traj_goals[i] = empty_traj_msg;
            cmd_vel[i] = empty_odom_msg.twist.twist;
            std::string odom_topic_str = uid_list[i] + "/" + odom_topic_name;
            std::string traj_topic_str = uid_list[i] + "/" + traj_topic_name;
            std::string cmd_vel_topic_name = uid_list[i] + "/cmd_vel";
            ros_subscriber = nh.subscribe<nav_msgs::Odometry>(odom_topic_str, odom_queue_size, boost::bind(&BehaviorConsensus2D::odomSubscriberCallback, this, _1, i));
            odom_sub.push_back(ros_subscriber);
            ros_subscriber = nh.subscribe<formation_msgs::PoseUID>(traj_topic_str, traj_queue_size, boost::bind(&BehaviorConsensus2D::trajSubscriberCallback, this, _1, i));
            traj_sub.push_back(ros_subscriber);
            ros_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, cmd_vel_queue_size);
            cmd_vel_pub.push_back(ros_publisher);
        }
    }

    void BehaviorConsensus2D::init(Eigen::MatrixXdRowMajor _A)  {
        A = _A;
        L = FormationUtils::calculateLaplacianMatrix<Eigen::MatrixXdRowMajor>(A);
        nav_msgs::Odometry empty_odom_msg;
        tf::pointEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.pose.pose.position);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        empty_odom_msg.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond::Identity());
        geometry_msgs::Pose empty_pose_msg;
        empty_pose_msg = empty_odom_msg.pose.pose;
        formation_msgs::PoseUID empty_traj_msg;
        empty_traj_msg.pose = empty_pose_msg;
        empty_traj_msg.twist = empty_odom_msg.twist.twist;
        bot_odoms.resize(num_bots);
        bot_odom_poses.resize(num_bots);
        traj_goals.resize(num_bots);
        cmd_vel.resize(num_bots);
        
        for (int i = 0; i < num_bots; i++) {
            
            bot_odoms[i] = empty_odom_msg;
            bot_odom_poses[i] = empty_pose_msg;
            traj_goals[i] = empty_traj_msg;
            cmd_vel[i] = empty_odom_msg.twist.twist;
            std::string odom_topic_str = uid_list[i] + "/" + odom_topic_name;
            std::string traj_topic_str = uid_list[i] + "/" + traj_topic_name;
            std::string cmd_vel_topic_name = uid_list[i] + "/cmd_vel";
            // Binding the callbacks to take the index and fill the right vector element thus allowing the vector
            // of subscribers to send their information to the right element of the storing vector.
            ros_subscriber = nh.subscribe<nav_msgs::Odometry>(odom_topic_str, odom_queue_size, boost::bind(&BehaviorConsensus2D::odomSubscriberCallback, this, _1, i));
            odom_sub.push_back(ros_subscriber);
            ros_subscriber = nh.subscribe<formation_msgs::PoseUID>(traj_topic_str, traj_queue_size, boost::bind(&BehaviorConsensus2D::trajSubscriberCallback, this, _1, i));
            traj_sub.push_back(ros_subscriber);
            ros_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, cmd_vel_queue_size);
            cmd_vel_pub.push_back(ros_publisher);
        }
        
    }

    void BehaviorConsensus2D::init(Eigen::MatrixXdRowMajor _A, Eigen::MatrixXdRowMajor initial_pose)  {
        A = _A;
        L = FormationUtils::calculateLaplacianMatrix<Eigen::MatrixXdRowMajor>(A);
        nav_msgs::Odometry empty_odom_msg;
        tf::pointEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.pose.pose.position);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), empty_odom_msg.twist.twist.linear);
        empty_odom_msg.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond::Identity());
        formation_msgs::PoseUID empty_traj_msg;
        empty_traj_msg.pose = empty_odom_msg.pose.pose;
        empty_traj_msg.twist = empty_odom_msg.twist.twist;
        bot_odoms.resize(num_bots);
        bot_odom_poses.resize(num_bots);
        traj_goals.resize(num_bots);
        cmd_vel.resize(num_bots);
        
        for (int i = 0; i < num_bots; i++) {
            tf::pointEigenToMsg(Eigen::Vector3d(initial_pose(i,0), initial_pose(i,1), initial_pose(i,2)), empty_odom_msg.pose.pose.position);
            Eigen::Quaterniond q;
            q = FormationUtils::euler_to_quaternion(initial_pose(i,3), initial_pose(i,4), initial_pose(i,5));
            empty_odom_msg.pose.pose.orientation = tf2::toMsg(q);
            bot_odoms[i] = empty_odom_msg;
            bot_odom_poses[i] = empty_odom_msg.pose.pose;
            traj_goals[i] = empty_traj_msg;
            cmd_vel[i] = empty_odom_msg.twist.twist;
            std::string odom_topic_str = uid_list[i] + "/" + odom_topic_name;
            std::string traj_topic_str = uid_list[i] + "/" + traj_topic_name;
            std::string cmd_vel_topic_name = uid_list[i] + "/cmd_vel";
            // Binding the callbacks to take the index and fill the right vector element thus allowing the vector
            // of subscribers to send their information to the right element of the storing vector.
            ros_subscriber = nh.subscribe<nav_msgs::Odometry>(odom_topic_str, odom_queue_size, boost::bind(&BehaviorConsensus2D::odomSubscriberCallback, this, _1, i));
            odom_sub.push_back(ros_subscriber);
            ros_subscriber = nh.subscribe<formation_msgs::PoseUID>(traj_topic_str, traj_queue_size, boost::bind(&BehaviorConsensus2D::trajSubscriberCallback, this, _1, i));
            traj_sub.push_back(ros_subscriber);
            ros_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, cmd_vel_queue_size);
            cmd_vel_pub.push_back(ros_publisher);
        }
        
    }

    void BehaviorConsensus2D::odomSubscriberCallback(const boost::shared_ptr<nav_msgs::Odometry const> odom_msg, int bot_idx){
        // Depending on the message metadata and assuming that the right namespace is
        // used for the odom topic of the bot. We will push the update at the right
        // position in the Odometry messages vector. The default frame name for odometry
        // is /odom and is assumed to be the default here as well.
        
        bot_odoms[bot_idx] = *odom_msg;
        bot_odom_poses[bot_idx] = odom_msg->pose.pose;
    }

    void BehaviorConsensus2D::trajSubscriberCallback(const boost::shared_ptr<formation_msgs::PoseUID const> traj_msg, int bot_idx) {
        traj_goals[bot_idx] = *traj_msg;
    }

    void BehaviorConsensus2D::applyControlLaw(){
        // Step 1: Transforming the pose of the robots to the projection point.
        // pp_2dtf.BotToPointTfByUID<geometry_msgs::Pose>(bot_odom_poses);
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
            
            pp_2dtf.BotToPointTfStored<geometry_msgs::Pose>(bot_odom_poses[i]);
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
            backstep_term = beta*(p_i - p_i_st); // Can even be a PID law.
            u_i_point[i] = -backstep_term - clsm*consensus_term;
            
            Eigen::Vector3d rpy = pp_2dtf.extractRPYfromQuaternionMsg(bot_odom_poses[i].orientation);
            theta_vec[i] = rpy(2); // Extracting yaw from r,p,y.
            
            // u_i_point is in si dynamics. We will now change the single integrator control input
            // back to unicycle dynamics control input and use it to populate the cmd_vel vector.
            bot_twists_2d[i] = pp_2dtf.SiToUniDynamicsTwistTf(u_i_point[i], theta_vec[i]);
            

            // Now filling the cmd_vel messages appropriately. Note that the twist 2d vector consists
            // of V w.r.t base_footprint x frame i.e. bot forward direction and omega i.e. about z axis
            // angular velocity component.
            double max_angular_vel;
            double min_angular_vel;
            pp_2dtf.getAngularVelocityLimits(max_angular_vel, min_angular_vel);
            FormationUtils::saturation_function(bot_twists_2d[i].y(), max_angular_vel, min_angular_vel);
            double min_fwd_vel = 0;
            FormationUtils::saturation_function(bot_twists_2d[i].x(), max_fwd_vel, min_fwd_vel);
            
            cmd_vel[i].linear.x = bot_twists_2d[i].x();
            
            cmd_vel[i].linear.y = 0;
            cmd_vel[i].linear.z = 0;
            cmd_vel[i].angular.x = 0;
            cmd_vel[i].angular.y = 0;
            cmd_vel[i].angular.z = bot_twists_2d[i].y();
            ROS_INFO_STREAM("BOT " << i << " cmd_vel V : " << bot_twists_2d[i].x() << " w : " << bot_twists_2d[i].y());
        }

        // Step 3: Now publishing the cmd_vel of all the bots together.
        for (int i = 0; i < num_bots; i++){
            cmd_vel_pub[i].publish(cmd_vel[i]);
        }
    }
}