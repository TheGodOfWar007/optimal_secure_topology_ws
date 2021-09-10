#ifndef TEST_PLANNER_H
#define TEST_PLANNER_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <formation_utils/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/publisher.h>

namespace FormationUtils {
    
    class TestMotionPlanner2D {
        public:
            TestMotionPlanner2D(ros::NodeHandle _nh): nh(_nh) { };

            ~TestMotionPlanner2D(){ };

            void load_params();

            void publishShapeTransforms();

            void setFlags(bool _use_static_transform) {
                USE_STATIC_TRANSFORM = _use_static_transform;
            }

            void advertiseCircularTrajectory2D(Eigen::Vector2d center, Eigen::Vector2d start_point, double radius, double time_period, double rate, bool CONTINUE_LOOPING);

        protected:
            ros::NodeHandle nh;
            tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
            tf2_ros::TransformBroadcaster tf_broadcaster;
            std::vector<std::string> uid_list;
            std::string planner_center;
            Eigen::MatrixXdRowMajor formation_shape;
            int num_bots;
            ros::Publisher traj_pub;

        protected:
            bool USE_STATIC_TRANSFORM;
    };
}

#endif // TEST_PLANNER_H