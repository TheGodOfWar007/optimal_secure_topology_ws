#include <formation_utils/dynamics_tf.h>


namespace FormationUtils{
    Eigen::Vector2d ProjectionPoint2DTf::UniToSiDynamicsStateTf(Eigen::Vector2d &bot_pos2d, double &theta){
        Eigen::Vector2d point_pos;
        point_pos.x() = bot_pos2d.x() + proj_distance*cos(theta);
        point_pos.y() = bot_pos2d.y() + proj_distance*sin(theta);
        return point_pos;
    }

    Eigen::Vector2d ProjectionPoint2DTf::UniToSiDynamicsTwistTf(Eigen::Vector2d &bot_twist2d, double &theta) {
        Eigen::Vector2d point_vel;
        point_vel.x() = bot_twist2d(0)*cos(theta) - bot_twist2d(1)*sin(theta)*proj_distance;
        point_vel.y() = bot_twist2d(0)*sin(theta) + bot_twist2d(1)*cos(theta)*proj_distance;
        return point_vel;
    }

    Eigen::Vector2d ProjectionPoint2DTf::SiToUniDynamicsTwistTf(Eigen::Vector2d &point_vel, double &theta){
        Eigen::Vector2d bot_twist2d;
        bot_twist2d.x() = point_vel.x()*cos(theta) + point_vel.y()*sin(theta);
        bot_twist2d.y() = (-point_vel.x()*sin(theta) + point_vel.y()*cos(theta))/proj_distance;
        saturation_function(bot_twist2d.y(), max_angular_vel, min_angular_vel);
        return bot_twist2d;
    }
}