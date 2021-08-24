#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <bits/stdc++.h>
#include <formation_utils/formation_handle.h>

namespace FormationUtils {

    FormationHandle::FormationHandle(ros::NodeHandle nh_):
        nh(nh_)
        {
            // Start by fetching the important values from the parameter server
            // and initializing the class.
            if (nh.hasParam("/formation_config/num_bots")) {
                nh.getParam("/formation_config/num_bots", num_bots);
                ROS_INFO("Number of Bots in the Swarm: %u", num_bots);
            }
            else {
                ROS_WARN("Number of Bots not specified, reverting to default value of 5");
                num_bots = 5;
            }

            if (nh.hasParam("len_uid")) {
                nh.getParam("len_uid", len_uid);
                ROS_INFO("UID length used: %u", len_uid);
            }
            else {
                ROS_WARN("UID length not specified, reverting to default value of 5");
                len_uid = 5;
            }

            // Setting the flags, using the flags in the parameter sever is mandatory.
            // If any flag is not specified, the default value is used.
            nh.getParam("/formation_config/USING_CUSTOM_UID", USING_CUSTOM_UID);
            nh.getParam("/formation_config/USING_FORMATION_CENTER", USING_FORMATION_CENTER);
            nh.getParam("/formation_config/USING_COMMON_FRAME", USING_COMMON_FRAME);
            nh.getParam("/formation_graph/DIRECTED_GRAPH", DIRECTED_GRAPH);
            nh.getParam("/formation_graph/SELF_CONNECTIONS", SELF_CONNECTIONS);
            nh.getParam("SPAWN_BOTS_GAZEBO", SPAWN_IN_GAZEBO);
            nh.getParam("SPAWN_BOTS_RVIZ", SPAWN_IN_RVIZ);

            // Get leader list irrespective of whether it is empty or not.
            nh.getParam("/formation_config/leader_uid", leader_uid);

            // Filling the UIDs.
            if (USING_CUSTOM_UID) {
                ROS_INFO("Using custom UIDs. Anything specified within the uid_list array will be ignored.");
                if (!GENERATED_CUSTOM_UID) {
                    GENERATED_CUSTOM_UID = _gen_uid_strict();
                }
                if (GENERATED_CUSTOM_UID){
                    ROS_INFO("UIDs generated successfully.");
                }
                else {
                    ROS_ERROR("UID Generation failed.");
                }
            }
            else {
                nh.getParam("/formation_config/uid_list", uid_list);
                if (uid_list.size() != num_bots) {
                    ROS_ERROR("Using custom UIDs but incorrect number of UIDs of bots specified.");
                }
                else {
                    ROS_INFO("UIDs loaded.");
                }
            }

            // Filling the initial pose.

            if (nh.hasParam("/formation_config/initial_pose")) {
                XmlRpc::XmlRpcValue initialPoseConfig;
                nh.getParam("/formation_config/initial_pose", initialPoseConfig);
                ROS_ASSERT(initialPoseConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
                if ((initialPoseConfig.size() / 3) != num_bots) {
                    ROS_ERROR("Incorrect number of initial poses defined.");
                }
                for (int i = 0; i < num_bots; i++) {
                    Eigen::Vector3f vec;
                    for (int j = 0; j < 3; j++) {
                        std::ostringstream ostr;
                        ostr << initialPoseConfig[num_bots * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> vec(i);
                    }
                    initial_pose.push_back(vec);
                }
            }
            else {
                ROS_ERROR("Initial Pose not defined for the robots.");
            }

            // Fetching the adjacency matrix.

            if (nh.hasParam("/formation_graph/A")) {
                XmlRpc::XmlRpcValue AConfig;
                nh.getParam("/formation_graph/A", AConfig);
                ROS_ASSERT(AConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
                if ((AConfig.size() / num_bots) != num_bots) {
                    ROS_ERROR("Incorrect adjacency matrix.");
                }
                Eigen::MatrixXf mat(num_bots, num_bots);
                for (int i = 0; i < num_bots; i++) {
                    for (int j = 0; j < num_bots; j++) {
                        std::ostringstream ostr;
                        ostr << AConfig[num_bots * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> mat(i, j);
                    }
                }
                A = mat;
            }

            // An integrity check for A will be added soon.
            // Calculating L from A.
            Eigen::MatrixXf D_out = Eigen::MatrixXf::Zero(A.rows(), A.cols());
            D_out.diagonal() = A.rowwise().sum();
            L = D_out - A;

            // Spawning the models in gazebo if asked to spawn.
            if (SPAWN_IN_GAZEBO) {
                bool spawn_success = _spawn_bots_gazebo();
                if (spawn_success) {
                    ROS_INFO("Bots successfully spawned.");
                }
                else {
                    ROS_INFO("Spawn failed. Check exceptions.");
                }
            }
            // Spawing in Rviz will be added later.

            // CONSTRUCTOR ENDS.
        }
    
    bool FormationHandle::_spawn_bots_gazebo() {
        if (BOTS_SPAWNED) {
            ROS_ERROR("BOTS_SPAWNED is set. Spawn aborted.");
            return false;
        }
        else {
            ROS_INFO("Spawning the bots. BOTS_SPAWNED will be set to prevent further spawns from the same handle.");
            std::string robot_description;
            if (nh.hasParam("robot_description")) {
                nh.getParam("robot_description", robot_description);
            }
            else {
                ROS_ERROR("Robot Description not defined. Define the robot_description parameter in the launch file");
            }
            for(int i = 0; i < num_bots; i++){
                std::string spawn_command = "rosrun gazebo_ros spawn_model -model " + uid_list[i] + "-robot_namespace " + uid_list[i] + \
                "-x " + std::to_string(initial_pose[i][1]) + "-y " + std::to_string(initial_pose[i][2]) + "-z " + std::to_string(initial_pose[i][3]) + \
                "-param " + robot_description;
                const char *spawn_command_c = spawn_command.c_str();
                system(spawn_command_c);
            }
            BOTS_SPAWNED = true;
            SPAWN_IN_GAZEBO = false;
            return true;
        }
    }

    bool FormationHandle::_gen_uid_strict() {
        std::string uid;
        static const char alphanum[] =
            "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        
        // using the time and get process ID system call as a seed to generate 
        // random sequences.
        srand( (unsigned) time(NULL) * getpid());

        // Integrity check of the generation call.
        if (!GENERATED_CUSTOM_UID){
            for (int j = 0; j < num_bots; ++j) {
                uid.reserve(len_uid);
                for (int i = 0; i < len_uid; ++i){
                    uid += alphanum[rand() % (sizeof(alphanum) - 1)];
                }
                uid_list.push_back(uid);
                uid.clear();
            }
            ROS_ASSERT(num_bots == uid_list.size());
            return true;
        }
        else {
            ROS_ERROR("GENERATED_CUSTOM_UID is set. Aborting further generations.");
            return false;
        }
    }

    FormationHandle::~FormationHandle() { };

}
