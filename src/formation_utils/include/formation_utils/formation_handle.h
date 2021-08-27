#ifndef FORMATION_HANDLE_H
#define FORMATION_HANDLE_H  

#include <ros/ros.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <string>
#include <XmlRpcValue.h>

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

        private:

            bool _spawn_bots_gazebo();

            bool _gen_uid_strict();

            void _xmlrpc_to_matrix(int rows, int cols, XmlRpc::XmlRpcValue& XmlConfig, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& mat);

            Eigen::Quaternionf _euler_to_quaternion(float r, float p, float y);

        protected:
            ros::NodeHandle nh;
            
            bool SELF_CONNECTIONS = false;
            bool DIRECTED_GRAPH = false;
            bool USING_FORMATION_CENTER = false;
            bool USING_COMMON_FRAME = false;
            
            int num_bots;
            int len_uid;
            std::vector<std::string> uid_list;
            std::vector<std::string> leader_uid;
            /**
             * @brief The n x n graph adjacency matrix. n = num_bots. Stores in Eigen::RowMajor format.
             */
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A;
            /**
             * @brief Stores the initial_pose in an n x 6 matrix in Eigen::RowMajor format. n = num_bots. 
             */
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> initial_pose;
            Eigen::MatrixXf L;

        private:
            bool SPAWN_IN_GAZEBO = false;
            bool SPAWN_IN_RVIZ = false;
            bool BOTS_SPAWNED = false;
            bool GENERATED_CUSTOM_UID = false;
            bool USING_CUSTOM_UID = false;

            ros::ServiceClient gazebo_spawn_client;
            
    };
}

#endif // FORMATION_HANDLE_H