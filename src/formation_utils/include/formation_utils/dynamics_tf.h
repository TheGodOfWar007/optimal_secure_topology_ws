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
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <formation_msgs/PoseUID.h>


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

            Eigen::Vector3d extractRPYfromQuaternionMsg(geometry_msgs::Quaternion &m){
                Eigen::Quaterniond q;
                tf::quaternionMsgToEigen(m, q);
                ROS_INFO_STREAM("Extract RPY Q: " << q.vec());
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
                min_angular_vel = -M_PI;
                max_projection_dist = 1.00;
                min_projection_dist = 0.01;
            }

            ProjectionPoint2DTf(double _proj_distance) : proj_distance(_proj_distance)
            {
                ROS_ASSERT(proj_distance > 0);
                max_angular_vel = M_PI;
                min_angular_vel = -M_PI;
                max_projection_dist = 1.00;
                min_projection_dist = 0.01;
            };

            ProjectionPoint2DTf(double _proj_distance, double _max_angular_vel)
            : proj_distance(_proj_distance),
              max_angular_vel(_max_angular_vel),
              min_angular_vel(-_max_angular_vel)
            {
                ROS_ASSERT(proj_distance > 0 && max_angular_vel > 0);
                max_projection_dist = 1.00;
                min_projection_dist = 0.01;
            };

            ProjectionPoint2DTf(double _proj_distance, double _max_angular_vel, double _min_angular_vel) 
            : proj_distance(_proj_distance),
              max_angular_vel(_max_angular_vel),
              min_angular_vel(_min_angular_vel)
            {
                ROS_ASSERT(proj_distance > 0 && max_angular_vel > 0);
                max_projection_dist = 1.00;
                min_projection_dist = 0.01;
            }

            ~ProjectionPoint2DTf() { };

            Eigen::Vector2d UniToSiDynamicsStateTf(Eigen::Vector2d bot_pos2d, double theta);

            Eigen::Vector2d UniToSiDynamicsTwistTf(Eigen::Vector2d bot_twist2d, double theta);

            // Eigen::Vector2d SiToUniDynamicsStateTf(Eigen::Vector2d &point_pos, double &theta);

            Eigen::Vector2d SiToUniDynamicsTwistTf(Eigen::Vector2d point_vel, double theta);

            void setNodeHandle(ros::NodeHandle &_nh) {
                nh = _nh;
            }

            void setProjectionDistance(double _proj_distance){
                proj_distance = _proj_distance; 
                saturation_function(proj_distance, max_projection_dist, min_projection_dist);
            }

            void setProjectionDistanceLimits(double _max_proj_dist, double _min_proj_dist) {
                max_projection_dist = _max_proj_dist;
                min_projection_dist = _min_proj_dist;

                ROS_ASSERT(max_projection_dist > min_projection_dist);
                ROS_ASSERT((max_projection_dist > 0) && (min_projection_dist > 0));
            }

            void setAngularVelocityLimits(double _max_angular_velocity, double _min_angular_velocity){
                max_angular_vel = _max_angular_velocity; 
                min_angular_vel = _min_angular_velocity;
            }

            void getProjectionDistance(double &_proj_distance) {
                _proj_distance = proj_distance;
            }

            void getAngularVelocityLimits(double &_max_angular_velocity, double &_min_angular_velocity) {
                _max_angular_velocity = max_angular_vel;
                _min_angular_velocity = min_angular_vel;
            }

            void getProjectionDistanceLimits(double &_max_proj_dist, double &_min_proj_dist) {
                _max_proj_dist = max_projection_dist;
                _min_proj_dist = min_projection_dist;
            }

            void setUidList(std::vector<std::string> _uid_list) {
                num_bots = _uid_list.size();
                for (int i = 0; i < num_bots; i++) {
                    uid_list.push_back(_uid_list[i]);
                }
            }

            void setFrameIDs(const std::string _point_frame_id, const std::string _parent_frame_id) {
                point_frame_id = _point_frame_id;
                parent_frame_id = _parent_frame_id;
            }

            void publishTransform(const std::string prefix_tf_with);

            void publishTransformsByUID();

            template <class T>
            void BotToPointTf(T &pose, std::string prefix_tf_with){
                tf2_ros::TransformListener tflistener(tf_buffer);
                transform_container = tf_buffer.lookupTransform(prefix_tf_with + "/" + parent_frame_id, prefix_tf_with + "/" + point_frame_id, ros::Time::now());
                tf2::doTransform<T>(pose, pose, transform_container);
            }

            template <class T>
            void BotToPointTfStored(T &pose){
                tf2::doTransform<T>(pose, pose, bot_to_point_tf);
            }

            template <class T>
            void BotToPointTfByUID(std::vector<T> &fpose_uid){
                ROS_INFO_STREAM("FPose Size: " << fpose_uid.size());
                ROS_ASSERT(fpose_uid.size() == num_bots);
                for (int i = 0; i < uid_list.size(); i++) {
                    BotToPointTf<T>(fpose_uid[i], uid_list[i]);
                }
            }

            template <class T>
            void PointToBotTf(T &point_pose, std::string prefix_tf_with){
                tf2_ros::TransformListener tflistener(tf_buffer);
                transform_container = tf_buffer.lookupTransform(prefix_tf_with + "/" + point_frame_id, prefix_tf_with + "/" + parent_frame_id, ros::Time::now());
                tf2::doTransform<T>(point_pose, point_pose, transform_container);
            }

            template <class T>
            void PointToBotTfStored(T &point_pose){
                tf2::doTransform<T>(point_pose, point_pose, point_to_bot_tf);
            }

            template <class T>
            void PointToBotTfByUID(std::vector<T> &fpoint_pose_uid){
                ROS_ASSERT(fpoint_pose_uid.size() == num_bots);
                for (int i = 0; i < uid_list.size(); i++) {
                    PointToBotTf<T>(fpoint_pose_uid[i], uid_list[i]);
                }
            }

            void setDyamicsTfFlags(bool _USE_ADAPTIVE_PROJECTION_DISTANCE = false, bool _USE_STATIC_TRANSFORMS = true) {
                USE_ADAPTIVE_PROJECTION_DISTANCE = _USE_ADAPTIVE_PROJECTION_DISTANCE;
                USE_STATIC_TRANSFORMS = _USE_STATIC_TRANSFORMS;
            }
        
        public: 
            std::vector<geometry_msgs::TransformStamped> tf_vector;
            geometry_msgs::TransformStamped bot_to_point_tf;
            geometry_msgs::TransformStamped point_to_bot_tf;
        protected:
            ros::NodeHandle nh;
            double proj_distance;
            double max_angular_vel;
            double min_angular_vel;
            double max_projection_dist;
            double min_projection_dist;
            std::vector<std::string> uid_list;
            tf2_ros::TransformBroadcaster tf_broadcaster;
            tf2_ros::StaticTransformBroadcaster tf_static_broadcaster;
            tf2_ros::Buffer tf_buffer;
            geometry_msgs::TransformStamped transform_container;
            std::string point_frame_id;
            std::string parent_frame_id;
            int num_bots;
        
        protected:
            bool USE_ADAPTIVE_PROJECTION_DISTANCE;
            bool USE_STATIC_TRANSFORMS;
    };
}

#endif // DYNAMICS_TF_H