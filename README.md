# optimal_secure_topology_ws

A framework for simulating multi-robot formation control algorithms using ROS and Gazebo.

WORK IN PROGRESS

### Project Description
Due to the absolute dearth of multi-agent simulations frameworks for ROS (as opposed to ROS2, which recently had ChoiRbot added to its arsenal), this repository was created with the intention to create a unifying simulation framework for testing Networked Control System algorithms on multi-robot systems. This was originally made for simulating attacks on multi-agent formation control algorithms of the Secure Topology Design project under Prof. Vaibhav Katewa's guidance.

The key idea is to reduce the workload required in setting up the multi-robot simulation stack minimal and allow for plug-and-play testing of distributed control algorithms. To launch a simulation all one has to do is edit the configuration file, called `formation_params.yaml in the formation_simulation package` to specify the number of robots of simulate and their initial position in the XY world frame of Gazebo simulator. An example of the formation params file can be seen here:

```yaml
# Specifying the formation specific parameters excluding the information related 
# to the communication graphs. Remember that the numerical paramaters unless specified
# with a decimal will have to be treated by stringstream. The matrices need to be manually
# translated from XmlRpc to EigenXd.

# Selecting the verbosity levels for nodes.
SELECT_DEBUG_MODE: true

# Initial pose format, row n = [x_n, y_n, z_n]

SPAWN_BOTS_GAZEBO: true
SPAWN_BOTS_RVIZ: false
len_uid: 5

TURTLEBOT3_MODEL: burger

formation_config: {
  USING_CUSTOM_UID: true,
  USING_FORMATION_CENTER: true,
  USING_COMMON_FRAME: false,
  num_bots: 5,
  uid_list: [BOT1, BOT2, BOT3, BOT4, BOT5],
  leader_uid: [],
  #              x    y    z    r    p    y
  initial_pose: [4.0, -1.0, 0.0, 0.0, 0.0, 0.0,
                6.0, -1.0, 0.0, 0.0, 0.0, 0.0,
                6.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                4.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# Type can strictly be either directed or undirected. The self connections
# parameter is required for verification of the adjacency matrix as well as
# initializing one in case the matrix is not declared or the dimensions are
# mismatched.

formation_graph: {
  DIRECTED_GRAPH: false,
  SELF_CONNECTIONS: false,
  A: [0.0,  1.0,  1.0,  1.0,  1.0,
      1.0,  0.0,  1.0,  1.0,  1.0,
      1.0,  1.0,  0.0,  1.0,  1.0,
      1.0,  1.0,  1.0,  0.0,  1.0,
      1.0,  1.0,  1.0,  1.0,  0.0,]
}

robot_state_publisher: {
  USE_STATIC_TF: true,
}

test_motion_planner: {
  num_bots: 5,
  USE_STATIC_TRANSFORM: true,
  uid_list: [BOT1, BOT2, BOT3, BOT4, BOT5],
  planner_center: BOT5,
  #                   x   y 
  static_formation: [0.0, 0.0,
                     2.0, 0.0,
                     2.0, 2.0,
                     0.0, 2.0,
                     1.0, 1.0]
}
```
