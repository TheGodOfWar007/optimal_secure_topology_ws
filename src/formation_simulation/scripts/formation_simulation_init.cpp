#include <ros/ros.h>
#include <ros/console.h>
#include <bits/stdc++.h>
#include <formation_utils/formation_handle.h>
#include <formation_utils/formation_graph_util.h>

#define pass (void)0

int main(int argc, char** argv) {
    // Initializing the ROS node handle
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    bool DEBUG_MODE;
    n.getParam("SELECT_DEBUG_MODE", DEBUG_MODE);

    // Setting the verbosity level:
    if (DEBUG_MODE) {
        if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    // Declaring the FormationHandle to spawn the robots.
    FormationUtils::FormationHandle formation_handle(n);
    formation_handle.initializeAndLoad(true);

    // Now entering the spinning part.
    FormationUtils::GraphInterface gi(&formation_handle);

    gi.InterfaceServiceAdvertise();

    ros::Rate r(10);

    while(n.ok()) {
        // For now we will go by single threaded spinning. But as the number of packages
        // changes, we will consider changing it to multi threaded spinning with custom
        // callback queues. Check: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning

        ros::spinOnce();
        r.sleep();
    }
}