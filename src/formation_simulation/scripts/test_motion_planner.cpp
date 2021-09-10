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

    FormationUtils::TestMotionPlanner2D test_planner2d(n);

    test_planner2d.load_params();

    test_planner2d.setFlags(true);

    test_planner2d.publishShapeTransforms();

    ros::Rate r(30); // Running this node at 30Hz.
    // The rate of a node should be judiciously decided after experimental 
    // tests for considering the correct value of the rate so that you don't
    // lose out on important data or fill up the buffers with a lot more data
    // than needed. A moderate amount is fine.

    while (n.ok()){
        // This node can make do with a single threaded spinner for now. 
        // Adding a multithreaded spinner will be considered in the near future.

        ros::spinOnce();
        r.sleep();
    }

}