#include <formation_utils/test_planner.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv) {
    // Initializing the test motion planner object.
    ros::init(argc, argv, "test_planner_node");
    ros::NodeHandle n;

    bool DEBUG_MODE;
    n.getParam("SELECT_DEBUG_MODE", DEBUG_MODE);

    // Setting the verbosity level:
    if (DEBUG_MODE) {
        if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    FormationUtils::CircleTrajectory2D circle_traj2d(n);
    
    circle_traj2d.load_params();

    circle_traj2d.setFlags(true, true);

    circle_traj2d.publishShapeTransforms();

    std::string traj_topic_name = "cmd_trajectory";
    circle_traj2d.setTrajectoryPublishParams(traj_topic_name);

    circle_traj2d.init(); // initializes all the important publishers and subscribers.

    Eigen::Vector2d circle_center(5.0, 5.0);
    double theta_initial = 0; // rad
    double radius = 5.0;
    double time_period = 62.831; // sec
    double rate = 30;

    circle_traj2d.setParams(circle_center, theta_initial, radius, time_period, rate);

    ros::Rate r(rate); // Running this node at 30Hz.
    // The rate of a node should be judiciously decided after experimental 
    // tests for considering the correct value of the rate so that you don't
    // lose out on important data or fill up the buffers with a lot more data
    // than needed. A moderate amount is fine.

    while (n.ok()){
        // This node can make do with a single threaded spinner for now. 
        // Adding a multithreaded spinner will be considered in the near future.

        // Step 1: Updating the Tf Buffers.
        circle_traj2d.updateTfBuffers();

        // Step 2: Updating the waypoint.
        circle_traj2d.generateNextWaypoint();

        // Step 3: Assuming the waypoint to be for the formation center. Transform and publish
        circle_traj2d.transformAndPublishTrajectory();

        ros::spinOnce();
        r.sleep();
    }

}