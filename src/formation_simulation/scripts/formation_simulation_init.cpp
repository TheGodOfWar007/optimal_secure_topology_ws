#include <ros/ros.h>
#include <ros/console.h>
#include <bits/stdc++.h>
#include <formation_utils/formation_handle.h>

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
}