#include <behavior_consensus_control/behavior_consensus_control.h>
#include <ros/ros.h>
#include <ros/console.h>

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
    std::vector<std::string> uid_list = {"BOT1", "BOT2", "BOT3", "BOT4", "BOT5"};
    bcc.setTopicNames(traj_topic_name, odom_topic_name);
    bcc.setParams(uid_list);

    // The most important part is the rate, it should be judiciously decided;

    ros::Rate r(30); // 30 Hz
    bcc.init();
    bcc.setAngularVelocityLimits(0.6, -0.6);
    bcc.setMaxForwardVelocity(1.0);
    bcc.setProjectionDistance(0.01);
    bcc.setProjectionDistanceLimits(1, -1); // This one doesn't matter unless the projection distance is being governed by any adaptive law.
    double beta = 2.0;
    bcc.setControllerConstants(beta);
    bcc.setFrameIDs("projection_point", "base_footprint"); // Setting the tf to be from base_footprint to the point.
    bcc.setDyamicsTfFlags(false, true); // Using default arguments, could have left it empty as well.

    bcc.publishTransformsByUID();

    // Lets wait till all the bots finish spawning. 
    ros::Duration(10).sleep();
    // This hard coded wait will be replaced by a wait on a service
    // indicating the exact completion of the bot spawn.

    while (n.ok()) {
        // Apply the control law within here since it will publish the cmd_vel.
        // The velocity publishing will be at the frequency of this node which
        // is why the publish rate is such an important parameter.

        bcc.applyControlLaw();

        ros::spinOnce();
        r.sleep();
    }

}