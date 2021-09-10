#include <formation_utils/dynamics_tf.h>
#include <tf2_eigen/tf2_eigen.h>

namespace FormationUtils{

    Eigen::Vector2d ProjectionPoint2DTf::UniToSiDynamicsStateTf(Eigen::Vector2d bot_pos2d, double theta){
        Eigen::Vector2d point_pos;
        point_pos.x() = bot_pos2d.x() + proj_distance*cos(theta);
        point_pos.y() = bot_pos2d.y() + proj_distance*sin(theta);
        return point_pos;
    }

    Eigen::Vector2d ProjectionPoint2DTf::UniToSiDynamicsTwistTf(Eigen::Vector2d bot_twist2d, double theta) {
        Eigen::Vector2d point_vel;
        point_vel.x() = bot_twist2d(0)*cos(theta) - bot_twist2d(1)*sin(theta)*proj_distance;
        point_vel.y() = bot_twist2d(0)*sin(theta) + bot_twist2d(1)*cos(theta)*proj_distance;
        return point_vel;
    }

    Eigen::Vector2d ProjectionPoint2DTf::SiToUniDynamicsTwistTf(Eigen::Vector2d point_vel, double theta){
        Eigen::Vector2d bot_twist2d;
        bot_twist2d.x() = point_vel.x()*cos(theta) + point_vel.y()*sin(theta);
        bot_twist2d.y() = (-point_vel.x()*sin(theta) + point_vel.y()*cos(theta))/proj_distance;
        saturation_function(bot_twist2d.y(), max_angular_vel, min_angular_vel);
        return bot_twist2d;
    }

    void ProjectionPoint2DTf::publishTransform(const std::string prefix_tf_with) {
        std::vector<geometry_msgs::TransformStamped> tf_transforms;
        geometry_msgs::TransformStamped tf_transform;
        tf_transform.header.stamp = ros::Time::now();
        if (!USE_STATIC_TRANSFORMS) {
            tf_transform.header.stamp += ros::Duration(0.5);
        }
        tf_transform.header.frame_id = prefix_tf_with + "/" + parent_frame_id;
        tf_transform.child_frame_id = prefix_tf_with + "/" + point_frame_id;

        tf::vectorEigenToMsg(Eigen::Vector3d(proj_distance, 0, 0), tf_transform.transform.translation);
        tf_transform.transform.rotation = tf2::toMsg(Eigen::Quaterniond::Identity());

        tf_transforms.push_back(tf_transform);

        if(USE_STATIC_TRANSFORMS && !USE_ADAPTIVE_PROJECTION_DISTANCE) {
            tf_static_broadcaster.sendTransform(tf_transforms);
        }
        if (!USE_STATIC_TRANSFORMS) {
            tf_broadcaster.sendTransform(tf_transforms);
        }
        if (USE_STATIC_TRANSFORMS && USE_ADAPTIVE_PROJECTION_DISTANCE) {
            ROS_WARN("Using Static Transforms while using ADAPTIVE Projection distance. The flags will be set to allow for non-static transforms with adaptive distance.");
            USE_STATIC_TRANSFORMS = false;
            tf_broadcaster.sendTransform(tf_transforms);
        }
    }

    void ProjectionPoint2DTf::publishTransformsByUID() {
        for (int i = 0 ; i < uid_list.size(); i++) {
            publishTransform(uid_list[i]);
        }
    }
}