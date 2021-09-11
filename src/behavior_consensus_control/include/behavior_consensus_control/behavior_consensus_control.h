#ifndef BEHAVIOR_CONSENSUS_CONTROL_H
#define BEHAVIOR_CONSENSUS_CONTROL_H

#include <formation_utils/utils.h>
#include <formation_utils/dynamics_tf.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/subscriber.h>
#include <ros/service_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <formation_msgs/PoseUID.h>
#include <formation_msgs/FormationGraphParams.h>
#include <formation_msgs/FormationGraphParamsRequest.h>
#include <formation_msgs/FormationGraphParamsResponse.h>


namespace FormationControl {
    std::string deleteSubstr(std::string main_str, std::string sub_str) {
        std::string::size_type i = main_str.find(sub_str);
        if(i != std::string::npos) {
            main_str.erase(i, sub_str.size());
        }
        return main_str;
    }
    class BehaviorConsensus2D : public FormationUtils::ProjectionPoint2DTf {
        public:
            BehaviorConsensus2D(ros::NodeHandle _nh)
            : nh(_nh)
            { max_fwd_vel = 1.0; }

            ~BehaviorConsensus2D() { };

            void setParams(std::vector<std::string> &_uid_list, uint32_t _odom_queue_size=10, uint32_t _traj_queue_size=10, uint32_t _cmd_vel_queue_size=10) {
                uid_list = _uid_list;
                num_bots = uid_list.size();
                odom_queue_size = _odom_queue_size;
                traj_queue_size = _traj_queue_size;
                cmd_vel_queue_size = _cmd_vel_queue_size;
            }

            void setTopicNames(std::string &_traj_topic_name, std::string &_odom_topic_name) {
                traj_topic_name = _traj_topic_name;
                odom_topic_name = _odom_topic_name;
            }

            void init();

            void odomSubscriberCallback(const nav_msgs::Odometry &odom_msg);

            void trajSubscriberCallback(const formation_msgs::PoseUID &traj_msg);

            void setControllerConstants(double &_beta) {
                beta = _beta;
            }

            void applyControlLaw();

            void setMaxForwardVelocity(double _max_fwd_vel) {
                max_fwd_vel = _max_fwd_vel;
            }


        protected:
            ros::NodeHandle nh;
            int num_bots;
            std::string traj_topic_name;
            std::string odom_topic_name;
            std::vector<ros::Subscriber> traj_sub;
            std::vector<ros::Publisher> cmd_vel_pub;
            std::vector<std::string> uid_list;
            std::vector<ros::Subscriber> odom_sub;
            std::vector<nav_msgs::Odometry> bot_odoms;
            std::vector<geometry_msgs::Pose> bot_odom_poses;
            std::vector<formation_msgs::PoseUID> traj_goals;
            std::vector<Eigen::Vector2d> bot_twists2d;
            std::vector<Eigen::Vector2d> point_velocities2d;
            std::vector<geometry_msgs::Twist> cmd_vel;
            ros::Subscriber ros_subscriber;
            ros::Publisher ros_publisher;
            ros::ServiceClient graph_interface_client;
            Eigen::MatrixXdRowMajor A;
            Eigen::MatrixXdRowMajor L;
        
        protected:
            uint32_t odom_queue_size;
            uint32_t traj_queue_size;
            uint32_t cmd_vel_queue_size;
            double beta;
            double max_fwd_vel; // m/s

            int current_odom_idx;
            int current_traj_idx;
    };
}

#endif // BEHAVIOR_CONSENSUS_CONTROL_H