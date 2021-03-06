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
#include <formation_utils/utils.h>

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

            virtual bool initializeAndLoad(bool INITIALIZE_PARAMS);

            virtual Eigen::MatrixXfRowMajor getAdjacencyMatrix() {
                return A;
            }

            virtual Eigen::MatrixXfRowMajor getLaplacianMatrix() {
                return L;
            }

            virtual bool setAdjacencyMatrix(Eigen::MatrixXfRowMajor A_) {
                A = A_;
                return true;
            }

            virtual bool setLaplacianMatrix(Eigen::MatrixXfRowMajor L_) {
                L = L_;
                return true;
            }

            virtual const std::vector<std::string>* getUIDListPtr() {
                return &uid_list;
            }

        private:
            
            virtual bool _initialize_params();

            virtual bool _spawn_bots_gazebo();

            virtual bool _gen_uid_strict();

        public:
            ros::NodeHandle nh;

        protected:
            
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
            Eigen::MatrixXfRowMajor A;
            /**
             * @brief Stores the initial_pose in an n x 6 matrix in Eigen::RowMajor format. n = num_bots. 
             */
            Eigen::MatrixXfRowMajor initial_pose;
            Eigen::MatrixXfRowMajor L;

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