#include <ros/ros.h>
#include <bits/stdc++.h>
#include <formation_utils/formation_handle.h>

#define pass (void)0

int main(int argc, char** argv) {
    // Initializing the ROS node handle
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    // Declaring the FormationHandle to spawn the robots.
    FormationUtils::FormationHandle formation_handle(n);
}