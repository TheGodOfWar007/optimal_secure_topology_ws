#include <behavior_consensus_control/behavior_consensus_control.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <XmlRpcValue.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "consensus_controller_node");
    ros::NodeHandle n;

    bool DEBUG_MODE;
    n.getParam("SELECT_DEBUG_MODE", DEBUG_MODE);

    // Setting the verbosity level:
    if (DEBUG_MODE) {
        if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    FormationControl::BehaviorConsensus2D bcc(n);

    std::string traj_topic_name = "cmd_trajectory";
    std::string odom_topic_name = "odom";
    // std::vector<std::string> uid_list = {"BOT1", "BOT2", "BOT3", "BOT4", "BOT5"};
    std::vector<std::string> uid_list = {"BOT1"};
    bcc.setTopicNames(traj_topic_name, odom_topic_name);
    ROS_INFO_STREAM("UID_LIST size given: " << uid_list.size());
    bcc.setParams(uid_list);

    // The most important part is the rate, it should be judiciously decided;
    double rate = 80; // Hz
    ros::Rate r(rate); 
    Eigen::MatrixXdRowMajor A;
    XmlRpc::XmlRpcValue AConfig;
    n.getParam("/formation_graph/A", AConfig);
    FormationUtils::xmlrpc_to_matrix<Eigen::MatrixXdRowMajor>(uid_list.size(), uid_list.size(), AConfig, A);
    XmlRpc::XmlRpcValue initialPoseConfig;
    int num_bots;
    n.getParam("/formation_config/num_bots", num_bots);
    n.getParam("/formation_config/initial_pose", initialPoseConfig);
    Eigen::MatrixXdRowMajor initial_pose;
    FormationUtils::xmlrpc_to_matrix<Eigen::MatrixXdRowMajor>(num_bots, 6, initialPoseConfig, initial_pose);

    bcc.init(A, initial_pose);

    // Using the velocity limits for Turtlebot3 Burger as specified by the hardware specs.

    bcc.pp_2dtf.setAngularVelocityLimits(BURGER_MAX_ANG_VEL, -BURGER_MAX_ANG_VEL); // rad/sec

    bcc.setMaxForwardVelocity(BURGER_MAX_VEL); // m/s

    bcc.pp_2dtf.setProjectionDistance(0.1);

    bcc.pp_2dtf.setProjectionDistanceLimits(1, 0.01); // This one doesn't matter unless the projection distance is being governed by any adaptive law.

    double beta = 0.001;
    double clsm = 0.01;
    bcc.setControllerConstants(beta, clsm, rate);
    double kp = 0.1;
    double kd = 0.01;
    double ki = 0.001;
    bcc.setPIDControllerConstants(kp, ki, kd, rate);

    bcc.pp_2dtf.setFrameIDs("projection_point", "base_footprint"); // Setting the tf to be from base_footprint to the point.

    bcc.pp_2dtf.setDyamicsTfFlags(false, true); // Using default arguments, could have left it empty as well.
    
    bcc.pp_2dtf.publishTransformsByUID();
    // Lets wait till all the bots finish spawning. 
    ROS_INFO("Waiting on the Trajectory Broadcast Trigger.");
    ros::service::waitForService("test_controller/trajectory_broadcast_trigger_srv");
    ROS_INFO("Calling the control law.");
    // This hard coded wait will be replaced by a wait on a service
    // indicating the exact completion of the bot spawn.
    
    while (n.ok()) {
        // Apply the control law within here since it will publish the cmd_vel.
        // The velocity publishing will be at the frequency of this node which
        // is why the publish rate is such an important parameter.

        bcc.applyControlLaw();
        // bcc.applyVanillaPID();

        ros::spinOnce();
        r.sleep();
    }

}