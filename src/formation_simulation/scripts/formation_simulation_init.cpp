#include <ros/ros.h>
#include <bits/stdc++.h>

#define pass (void)0

int main(int argc, char** argv) {
    // Initializing the ROS node handle
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    // Fetching the robot information from the parameter server 
    // and spawning the robots.
}