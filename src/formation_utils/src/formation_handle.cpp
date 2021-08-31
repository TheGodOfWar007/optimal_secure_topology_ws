#include <ros/ros.h>
#include <ros/console.h>
#include <bits/stdc++.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <formation_utils/formation_handle.h>

namespace FormationUtils {

    FormationHandle::FormationHandle(ros::NodeHandle nh_): nh(nh_) { }
    
    bool FormationHandle::initializeAndLoad(bool INITIALIZE_PARAMS) {
        if(INITIALIZE_PARAMS) {
            _initialize_params();
            // Spawning the models in gazebo if asked to spawn.
            if (SPAWN_IN_GAZEBO) {
                bool spawn_success = _spawn_bots_gazebo();
                if (!spawn_success) {
                    ROS_ERROR("Spawn failed. Check exceptions.");
                }
            }
            // Spawing in Rviz will be added later.
        }
        else {
            ROS_WARN("The parameters must be loaded at least once. Remeber to keep INITIALIZE_PARAMS when calling the handle for the first time.");
        }

        return true;
    }

    bool FormationHandle::_initialize_params() {
        // Start by fetching the important values from the parameter server
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
        if (!USING_CUSTOM_UID) {
            ROS_INFO("Using custom UIDs. Anything specified within the uid_list array will be ignored.");
            if (!GENERATED_CUSTOM_UID) {
                GENERATED_CUSTOM_UID = _gen_uid_strict();
            }
            if (!GENERATED_CUSTOM_UID){
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
            ROS_DEBUG_STREAM("initialPoseConfig:" << initialPoseConfig);
            if ((initialPoseConfig.size() / 6) != num_bots) {
                ROS_ERROR("Incorrect number of initial poses defined.");
            }
            xmlrpc_to_matrix<Eigen::MatrixXfRowMajor>(num_bots, 6, initialPoseConfig, initial_pose);
            ROS_DEBUG_STREAM("Initial Pose: \n" << initial_pose);                    
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
            xmlrpc_to_matrix<Eigen::MatrixXfRowMajor>(num_bots, num_bots, AConfig, A);
            ROS_DEBUG_STREAM("\"A\" stored successfully: \n" << A);
        }

        // An integrity check for A will be added soon.
        // Calculating L from A.
        Eigen::MatrixXfRowMajor D_out = Eigen::MatrixXfRowMajor::Zero(A.rows(), A.cols());
        D_out.diagonal() = A.rowwise().sum();
        L = D_out - A;

        return true;
    }

    bool FormationHandle::_spawn_bots_gazebo() {
        if (BOTS_SPAWNED) {
            ROS_ERROR("BOTS_SPAWNED is set. Spawn aborted.");
            return false;
        }
        else {
            std::string robot_description;
            gazebo_spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
            if (nh.hasParam("robot_description")) {
                nh.getParam("robot_description", robot_description);
            }
            else {
                ROS_ERROR("robot_description not found. Check that robot_description param is defined in the launch file or adverstised as a parameter.");
            }
            ROS_INFO("Spawning the bots. BOTS_SPAWNED will be set to prevent further spawns from the same handle.");
            for(int i = 0; i < num_bots; i++){
                // std::string spawn_command = "rosrun gazebo_ros spawn_model -model " + uid_list[i] + " -robot_namespace " + uid_list[i] + \
                // " -x " + std::to_string(initial_pose(i,1)) + " -y " + std::to_string(initial_pose(i,2)) + " -z " + std::to_string(initial_pose(i,3)) + \
                // " -urdf -param robot_description";
                // const char *spawn_command_c = spawn_command.c_str();
                // ROS_DEBUG_STREAM("Using system call spawn string:" << spawn_command);
                // system(spawn_command_c);
                // spawn_command.clear();

                // Converting Euler Angles to Quaternion.
                Eigen::Quaternionf q;
                q = euler_to_quaternion(initial_pose(i, 3), initial_pose(i, 4), initial_pose(i, 5));
                gazebo_msgs::SpawnModel gazebo_spawn_msg;
                geometry_msgs::Pose pose;
                pose.position.x = initial_pose(i,0);
                pose.position.y = initial_pose(i,1);
                pose.position.z = initial_pose(i,2);
                pose.orientation.w = q.w();
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                gazebo_spawn_msg.request.model_name = uid_list[i];
                gazebo_spawn_msg.request.model_xml = robot_description;
                gazebo_spawn_msg.request.reference_frame = "world";
                gazebo_spawn_msg.request.robot_namespace = uid_list[i];
                gazebo_spawn_msg.request.initial_pose = pose;

                // Calling the gazebo spawn_model client.
                gazebo_spawn_client.call(gazebo_spawn_msg);
                
                if (gazebo_spawn_msg.response.success) {
                    ROS_INFO_STREAM("NS: " << uid_list[i] << " Msg: " << gazebo_spawn_msg.response.status_message);
                }
                else {
                    ROS_ERROR_STREAM("NS: " << uid_list[i] << " Received: " << gazebo_spawn_msg.response.status_message);
                }
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
        
        int offset = 0; // for preventing the first digit of UID from being an alphabet, otherwise the model won't spawn.
        
        // using the time and get process ID system call as a seed to generate 
        // random sequences.
        srand( (unsigned) time(NULL) * getpid());

        // Integrity check of the generation call.
        if (!GENERATED_CUSTOM_UID){
            for (int j = 0; j < num_bots; ++j) {
                uid.reserve(len_uid);
                for (int i = 0; i < len_uid; ++i){
                    if (i == 0) {
                        offset = 10;
                    }
                    uid += alphanum[(rand() % (sizeof(alphanum) - 1 - offset)) + offset];
                    offset = 0;
                }
                uid_list.push_back(uid);
                ROS_DEBUG_STREAM("" << uid);
                uid.clear();
            }
            ROS_ASSERT(num_bots == uid_list.size());
            ROS_INFO("UIDs generated successfully.");
            return true;
        }
        else {
            ROS_ERROR("GENERATED_CUSTOM_UID is set. Aborting further generations.");
            return false;
        }
    }

    FormationHandle::~FormationHandle() { };
}
