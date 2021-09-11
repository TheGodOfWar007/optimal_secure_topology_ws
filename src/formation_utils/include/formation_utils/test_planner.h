#ifndef TEST_PLANNER_H
#define TEST_PLANNER_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <formation_msgs/PoseUID.h>
#include <formation_utils/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <math.h>

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

            void setTrajectoryPublishParams(std::string _traj_topic_name, uint32_t _traj_queue_size=10) {
                traj_topic_name = _traj_topic_name;
                traj_queue_size = _traj_queue_size;
            }

            void init();

            void updateTfBuffers();

            void transformAndPublishTrajectory();

        protected:
            ros::NodeHandle nh;
            tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
            tf2_ros::TransformBroadcaster tf_broadcaster;
            std::vector<std::string> uid_list;
            std::vector<ros::Publisher> traj_pub_vec;
            std::vector<formation_msgs::PoseUID> traj_goals;
            formation_msgs::PoseUID planner_center_goal;
            std::vector<geometry_msgs::TransformStamped> tf_vector;
            tf2_ros::Buffer tf_buffer;
            std::string planner_center;
            std::string traj_topic_name;
            Eigen::MatrixXdRowMajor formation_shape;
            int num_bots;

        protected:
            bool USE_STATIC_TRANSFORM;
            int planner_center_idx;
            uint32_t traj_queue_size;
    };

       class CircleTrajectory2D : public TestMotionPlanner2D {
        public:
            CircleTrajectory2D(ros::NodeHandle _nh) : TestMotionPlanner2D(_nh) { };

            ~CircleTrajectory2D() { };

            void setParams(Eigen::Vector2d &_circle_center, double &_theta_initial, double &_radius, double &_time_period, double &_rate, double _comparison_multiplier=2.00){
                circle_center = _circle_center;
                theta_initial = _theta_initial;
                radius = _radius;
                time_period = _time_period;
                rate = _rate;
                comparison_multiplier = _comparison_multiplier;
                planner_iteration = 0;
                omega = 2*M_PI/time_period;
                t_ini = ros::Time::now();
                t_curr = ros::Time::now();
                t = 0;
                current_coord = Eigen::Vector2d::Zero();
                last_coord = current_coord;
                _stop_iteration = false;
            }

            void setFlags(bool _CONTINUE_LOOPING, bool _use_static_transforms) {
                CONTINUE_LOOPING = _CONTINUE_LOOPING;
                // Setting the falgs of the derived function through the overload.
                TestMotionPlanner2D::setFlags(_use_static_transforms);
            }

            void generateNextWaypoint();

        protected:
            Eigen::Vector2d circle_center;
            double theta_initial;
            double radius;
            double time_period;
            double rate;

        protected:
            double omega;
            Eigen::Vector2d current_coord;
            Eigen::Vector2d last_coord;
            uint64_t planner_iteration;
            ros::Time t_ini;
            ros::Time t_curr;
            double t;
            double theta_current;

        protected:
            bool CONTINUE_LOOPING;
            bool _stop_iteration;
            double comparison_multiplier;
    };
}

#endif // TEST_PLANNER_H