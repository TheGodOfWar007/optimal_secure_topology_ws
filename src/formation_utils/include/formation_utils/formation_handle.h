#ifndef FORMATION_HANDLE_H
#define FORMATION_HANDLE_H  

#include <ros/ros.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace FormationUtils {

    class FormationHandle {
        public:

            /**
             * @brief Default constructor for the FormationHandle.
             * 
             */
            FormationHandle(ros::NodeHandle nh_);

            /**
             * @brief Destroy the Formation Handle object
             * 
             */
            virtual ~FormationHandle();

            bool _spawn_bots_gazebo();

            bool _gen_uid_strict();

        protected:
            ros::NodeHandle nh;

            bool SPAWN_IN_GAZEBO = false;
            bool SPAWN_IN_RVIZ = false;
            bool BOTS_SPAWNED = false;
            bool GENERATED_CUSTOM_UID = false;
            bool SELF_CONNECTIONS = false;
            bool DIRECTED_GRAPH = false;
            bool USING_CUSTOM_UID = false;
            bool USING_FORMATION_CENTER = false;
            bool USING_COMMON_FRAME = false;
            
            int num_bots;
            int len_uid;
            std::vector<std::string> uid_list;
            std::vector<std::string> leader_uid;
            Eigen::MatrixXf A;
            std::vector<Eigen::Vector3f> initial_pose;
            Eigen::MatrixXf L;
    };
}

#endif // FORMATION_HANDLE_H