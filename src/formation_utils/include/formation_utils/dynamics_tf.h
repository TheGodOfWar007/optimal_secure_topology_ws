#ifndef DYNAMICS_TF_H
#define DYNAMICS_TF_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <formation_utils/utils.h>
#include <math.h>

namespace FormationUtils{

    class DynamicsTfBase{
        public:
            DynamicsTfBase() { };

            ~DynamicsTfBase() { };

            geometry_msgs::Pose getLatestPoseInput() {
                return input_pose;
            }

            geometry_msgs::Twist getLatestTwistInput() {
                return input_twist;
            }

            geometry_msgs::Pose getLatestTranslatedPose() {
                return translated_pose;
            }

            geometry_msgs::Twist getLatestTranslatedTwist() {
                return translated_twist;
            }

        protected:
            Eigen::Vector3d extractRPYfromQuaternionMsg(geometry_msgs::Quaternion &m){
                Eigen::Quaterniond q;
                tf::quaternionMsgToEigen(m, q);
                Eigen::Vector3d rpy = quaternion_to_euler(q);
                return rpy;
            }
        protected:
            geometry_msgs::Pose input_pose;
            geometry_msgs::Twist input_twist;
            Eigen::Vector3d input_rpy;
            geometry_msgs::Pose translated_pose;
            geometry_msgs::Twist translated_twist;
            Eigen::Vector3d translated_rpy;
    };

    class ProjectionPoint2DTf : public DynamicsTfBase {
        public:
            ProjectionPoint2DTf() { 
                proj_distance = 0.1; // metres
                max_angular_vel = M_PI; // rad/s
            }

            ProjectionPoint2DTf(double _proj_distance) : proj_distance(_proj_distance)
            {
                ROS_ASSERT(proj_distance > 0);
                max_angular_vel = M_PI;
                min_angular_vel = -M_PI;
            };

            ProjectionPoint2DTf(double _proj_distance, double _max_angular_vel)
            : proj_distance(_proj_distance),
              max_angular_vel(_max_angular_vel),
              min_angular_vel(-_max_angular_vel)
            {
                ROS_ASSERT(proj_distance > 0 && max_angular_vel > 0);
            };

            ~ProjectionPoint2DTf() { };

            Eigen::Vector2d UniToSiDynamicsStateTf(Eigen::Vector2d &bot_pos2d, double &theta);

            Eigen::Vector2d UniToSiDynamicsTwistTf(Eigen::Vector2d &bot_twist2d, double &theta);

            // Eigen::Vector2d SiToUniDynamicsStateTf(Eigen::Vector2d &point_pos, double &theta);

            Eigen::Vector2d SiToUniDynamicsTwistTf(Eigen::Vector2d &point_vel, double &theta);

            void setProjectionDistance(double _proj_distance){ proj_distance = _proj_distance; };

            void setMaxAngularVelocity(double _max_ang_vel){ max_angular_vel = _max_ang_vel; };

        protected:
            double proj_distance;
            double max_angular_vel;
            double min_angular_vel;
    };
}

#endif DYNAMICS_TF_H