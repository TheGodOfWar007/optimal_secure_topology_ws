#include <ros/ros.h>
#include <ros/console.h>
#include <bits/stdc++.h>
#include <formation_utils/formation_handle.h>
#include <formation_utils/formation_graph_util.h>
#include <formation_utils/formation_state_publisher.h>
#include <formation_utils/turtlebot3_fake_transforms.hpp>

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

    FormationUtils::FormationStatePublisher fsp(&formation_handle);

    FormationUtils::Turtlebot3FakeLocalization tb3_fake_loc(&formation_handle);

    tb3_fake_loc.init();

    tb3_fake_loc.setOdomMode(ODOM_IS_SAME_AS_MAP);


    if (!fsp.init()){
        ROS_ERROR("Formation State Publisher failed to initialize.");
    }

    gi.InterfaceServiceAdvertise();

    fsp.publishFixedTransforms();

    tb3_fake_loc.advertiseTransforms();

    ros::Rate r(10);

    while(n.ok()) {
        // For now we will go by single threaded spinning. But as the number of packages
        // changes, we will consider changing it to multi threaded spinning with custom
        // callback queues. Check: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
        
        ros::spinOnce();
        r.sleep();
    }
}