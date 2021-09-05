#ifndef TURTLEBOT3_FAKE_TRANSFORMS_HPP
#define TURTLEBOT3_FAKE_TRANSFORMS_HPP

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <formation_utils/formation_handle.h>
#include <formation_utils/utils.h>
#include <geometry_msgs/TransformStamped.h>

#define ODOM_IS_SAME_AS_MAP             (int)0
#define ODOM_IS_AT_INITIAL_POSITION     (int)1

namespace FormationUtils {
    class Turtlebot3FakeLocalization {
        public:
            Turtlebot3FakeLocalization(FormationHandle *_fh) : fh(_fh) { };

            virtual ~Turtlebot3FakeLocalization() { };

            bool init(){
                if(!fh->nh.getParam("/formation_config/num_bots", num_bots)) {
                    ROS_ERROR("Failed to fetch number of robots from parameter server. Make sure /formation_config/num_bots is defined.");
                    return false;
                }
                XmlRpc::XmlRpcValue initialPoseConfig;
                fh->nh.getParam("/formation_config/initial_pose", initialPoseConfig);
                ROS_ASSERT(initialPoseConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
                if ((initialPoseConfig.size() / 6) != num_bots) {
                    ROS_ERROR("Incorrect number of initial poses defined.");
                }
                xmlrpc_to_matrix<Eigen::MatrixXdRowMajor>(num_bots, 6, initialPoseConfig, initial_pose);
                uid_list_ptr = fh->getUIDListPtr();
                return true;
            }

            void setOdomMode(int _odom_mode) {
                ODOM_MODE = _odom_mode;
            }

            int getOdomMode() {
                return ODOM_MODE;
            }

            void advertiseTransforms(){
                std::vector<geometry_msgs::TransformStamped> tf_transforms;
                geometry_msgs::TransformStamped tf_transform;
                if (ODOM_MODE==ODOM_IS_SAME_AS_MAP) {
                    for (int i = 0; i < num_bots; i++) {
                        tf_transform.header.frame_id = "/map";
                        tf_transform.child_frame_id = uid_list_ptr->at(i) + "/odom";
                        tf_transform.header.stamp = ros::Time::now();
                        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
                        tf_transform.transform.translation.x = 0;
                        tf_transform.transform.translation.y = 0;
                        tf_transform.transform.translation.z = 0;
                        tf_transform.transform.rotation.w = q.w();
                        tf_transform.transform.rotation.x = q.x();
                        tf_transform.transform.rotation.y = q.y();
                        tf_transform.transform.rotation.z = q.z();
                        tf_transforms.push_back(tf_transform);
                    }
                }
                else if (ODOM_MODE==ODOM_IS_AT_INITIAL_POSITION) {
                    for (int i = 0; i < num_bots; i++) {
                        tf_transform.header.frame_id = "/map";
                        tf_transform.child_frame_id = uid_list_ptr->at(i) + "/odom";
                        tf_transform.header.stamp = ros::Time::now();
                        Eigen::Quaterniond q = euler_to_quaternion(initial_pose(i,3), initial_pose(i,4), initial_pose(i,5));
                        tf_transform.transform.translation.x = initial_pose(i,0);
                        tf_transform.transform.translation.y = initial_pose(i,1);
                        tf_transform.transform.translation.z = initial_pose(i,2);
                        tf_transform.transform.rotation.w = q.w();
                        tf_transform.transform.rotation.x = q.x();
                        tf_transform.transform.rotation.y = q.y();
                        tf_transform.transform.rotation.z = q.z();
                        tf_transforms.push_back(tf_transform);
                    }
                }
                else {
                    ROS_ERROR("Invalid ODOM_MODE set. Please make sure that it matched one of the defined statements in the turtlebot3_fake_transforms.hpp");
                }
                tf_static_broadcaster.sendTransform(tf_transforms);
            }

        protected:
            FormationHandle *fh;
            tf2_ros::StaticTransformBroadcaster tf_static_broadcaster;
            const std::vector<std::string>* uid_list_ptr;
            Eigen::MatrixXdRowMajor initial_pose;
            int num_bots;

        protected:
            int ODOM_MODE;
            
    };
}

#endif // TURTLEBOT3_FAKE_TRANSFORMS_HPP