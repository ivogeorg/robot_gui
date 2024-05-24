### robot_gui

A simple GUI to control and monitor a Mir robot.

#### Simulation controller bug

On `roslaunch mir_gazebo mir_maze_world.launch`:

```
... logging to /home/user/.ros/log/bfd10f88-1a08-11ef-8559-0242ac120007/roslaunch-1_xterm-2281.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://1_xterm:35981/

SUMMARY
========

CLEAR PARAMETERS
 * /ekf_localization_node/

PARAMETERS
 * /ekf_localization_node/acceleration_gains: [0.8, 0.0, 0.0, 0...
 * /ekf_localization_node/acceleration_limits: [1.3, 0.0, 0.0, 0...
 * /ekf_localization_node/base_link_frame: base_footprint
 * /ekf_localization_node/control_config: [True, False, Fal...
 * /ekf_localization_node/control_timeout: 0.2
 * /ekf_localization_node/debug: False
 * /ekf_localization_node/debug_out_file: /path/to/debug/fi...
 * /ekf_localization_node/deceleration_gains: [1.0, 0.0, 0.0, 0...
 * /ekf_localization_node/deceleration_limits: [1.3, 0.0, 0.0, 0...
 * /ekf_localization_node/frequency: 40
 * /ekf_localization_node/imu0: imu_data
 * /ekf_localization_node/imu0_config: [False, False, Fa...
 * /ekf_localization_node/imu0_differential: False
 * /ekf_localization_node/imu0_nodelay: False
 * /ekf_localization_node/imu0_queue_size: 10
 * /ekf_localization_node/imu0_relative: True
 * /ekf_localization_node/imu0_remove_gravitational_acceleration: False
 * /ekf_localization_node/initial_estimate_covariance: [100.0, 0, 0, 0, ...
 * /ekf_localization_node/map_frame: map
 * /ekf_localization_node/odom0: odom
 * /ekf_localization_node/odom0_config: [False, False, Fa...
 * /ekf_localization_node/odom0_differential: False
 * /ekf_localization_node/odom0_nodelay: False
 * /ekf_localization_node/odom0_queue_size: 10
 * /ekf_localization_node/odom0_relative: False
 * /ekf_localization_node/odom_frame: odom
 * /ekf_localization_node/print_diagnostics: True
 * /ekf_localization_node/process_noise_covariance: [0.05, 0, 0, 0, 0...
 * /ekf_localization_node/publish_acceleration: False
 * /ekf_localization_node/publish_tf: True
 * /ekf_localization_node/sensor_timeout: 0.1
 * /ekf_localization_node/stamped_control: False
 * /ekf_localization_node/transform_time_offset: 0.0
 * /ekf_localization_node/transform_timeout: 0.0
 * /ekf_localization_node/two_d_mode: True
 * /ekf_localization_node/use_control: False
 * /ekf_localization_node/world_frame: odom
 * /gazebo/enable_ros_network: True
 * /joint_state_controller/publish_rate: 50
 * /joint_state_controller/type: joint_state_contr...
 * /joint_state_publisher/rate: 200.0
 * /joint_state_publisher/source_list: ['mir/joint_states']
 * /mobile_base_controller/angular/z/has_acceleration_limits: True
 * /mobile_base_controller/angular/z/has_velocity_limits: True
 * /mobile_base_controller/angular/z/max_acceleration: 2.5
 * /mobile_base_controller/angular/z/max_velocity: 1.5
 * /mobile_base_controller/base_frame_id: base_footprint
 * /mobile_base_controller/cmd_vel_timeout: 0.5
 * /mobile_base_controller/enable_odom_tf: False
 * /mobile_base_controller/left_wheel: left_wheel_joint
 * /mobile_base_controller/linear/x/has_acceleration_limits: True
 * /mobile_base_controller/linear/x/has_velocity_limits: True
 * /mobile_base_controller/linear/x/max_acceleration: 2.0
 * /mobile_base_controller/linear/x/max_velocity: 1.0
 * /mobile_base_controller/odom_frame_id: odom
 * /mobile_base_controller/pose_covariance_diagonal: [1e-05, 1e-05, 10...
 * /mobile_base_controller/publish_rate: 41.2
 * /mobile_base_controller/right_wheel: right_wheel_joint
 * /mobile_base_controller/twist_covariance_diagonal: [0.1, 0.1, 100000...
 * /mobile_base_controller/type: diff_drive_contro...
 * /mobile_base_controller/wheel_radius_multiplier: 1.0
 * /mobile_base_controller/wheel_separation_multiplier: 1.0
 * /robot_description: <?xml version="1....
 * /rosdistro: noetic
 * /rosversion: 1.15.11
 * /rqt_robot_steering/default_topic: cmd_vel
 * /rqt_robot_steering/default_vw_max: 1.5
 * /rqt_robot_steering/default_vw_min: -1.5
 * /rqt_robot_steering/default_vx_max: 1.0
 * /rqt_robot_steering/default_vx_min: -1.0
 * /use_sim_time: True

NODES
  /
    b_rep117_laser_filter (mir_driver/rep117_filter.py)
    b_scan_relay (topic_tools/relay)
    controller_spawner (controller_manager/spawner)
    ekf_localization_node (robot_localization/ekf_localization_node)
    f_rep117_laser_filter (mir_driver/rep117_filter.py)
    f_scan_relay (topic_tools/relay)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    joint_state_publisher (joint_state_publisher/joint_state_publisher)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rqt_robot_steering (rqt_robot_steering/rqt_robot_steering)
    spawn_maze (gazebo_ros/spawn_model)
    spawn_urdf (gazebo_ros/spawn_model)

auto-starting new master
process[master]: started with pid [2357]
ROS_MASTER_URI=http://1_xterm:11311

setting /run_id to bfd10f88-1a08-11ef-8559-0242ac120007
process[rosout-1]: started with pid [2376]
started core service [/rosout]
process[gazebo-2]: started with pid [2380]
process[gazebo_gui-3]: started with pid [2382]
process[spawn_urdf-4]: started with pid [2384]
process[controller_spawner-5]: started with pid [2391]
process[ekf_localization_node-6]: started with pid [2407]
process[joint_state_publisher-7]: started with pid [2426]
process[robot_state_publisher-8]: started with pid [2429]
process[rqt_robot_steering-9]: started with pid [2440]
[INFO] [1716581055.418193, 0.000000]: Controller Spawner: Waiting for service controller_manager/load_controller
process[b_scan_relay-10]: started with pid [2449]
process[f_scan_relay-11]: started with pid [2458]
process[b_rep117_laser_filter-12]: started with pid [2459]
process[f_rep117_laser_filter-13]: started with pid [2460]
process[spawn_maze-14]: started with pid [2461]
[INFO] [1716581056.124318, 0.000000]: Loading model XML from file /home/user/catkin_ws/src/mir_robot/mir_gazebo/sdf/maze/model.sdf
[INFO] [1716581056.126860, 0.000000]: Waiting for service /gazebo/spawn_sdf_model
++ ls /usr/bin/gzclient-11.5.1
+ gzclient_path=/usr/bin/gzclient-11.5.1
+ DISPLAY=:2
+ /usr/bin/gzclient-11.5.1 -g /opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so -g /opt/ros/noetic/lib/libgazebo_ros_api_plugin.so joint_states:=mir/joint_states mobile_base_controller/cmd_vel:=cmd_vel mobile_base_controller/odom:=odom __name:=gazebo_gui __log:=/home/user/.ros/log/bfd10f88-1a08-11ef-8559-0242ac120007/gazebo_gui-3.log
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-user'
[WARN] [1716581085.578922, 0.000000]: Controller Spawner couldn't find the expected controller_manager ROS interface.
[controller_spawner-5] process has finished cleanly
log file: /home/user/.ros/log/bfd10f88-1a08-11ef-8559-0242ac120007/controller_spawner-5*.log
[ INFO] [1716581091.733964115]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1716581091.737126460]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[ INFO] [1716581091.782975544]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1716581091.784417169]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[ INFO] [1716581094.075888436]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[INFO] [1716581094.124624, 0.000000]: Calling service /gazebo/spawn_sdf_model
[INFO] [1716581094.322245, 0.000000]: Spawn status: SpawnModel: Successfully spawned entity
[ INFO] [1716581094.335586767]: Physics dynamic reconfigure ready.
[spawn_maze-14] process has finished cleanly
log file: /home/user/.ros/log/bfd10f88-1a08-11ef-8559-0242ac120007/spawn_maze-14*.log
[ INFO] [1716581095.164273781]: Laser Plugin: Using the 'robotNamespace' param: '/'
[ INFO] [1716581095.164385071]: Starting Laser Plugin (ns = /)
[ INFO] [1716581095.165728831]: Laser Plugin: Using the 'robotNamespace' param: '/'
[ INFO] [1716581095.165758811]: Starting Laser Plugin (ns = /)
[ INFO] [1716581095.168059088]: Laser Plugin (ns = /)  <tf_prefix_>, set to ""
[ INFO] [1716581095.168335723]: Laser Plugin (ns = /)  <tf_prefix_>, set to ""
[spawn_urdf-4] process has finished cleanly
log file: /home/user/.ros/log/bfd10f88-1a08-11ef-8559-0242ac120007/spawn_urdf-4*.log
[ INFO] [1716581096.209708237]: Loading gazebo_ros_control plugin
[ INFO] [1716581096.209827849]: Starting gazebo_ros_control plugin in namespace: /
[ INFO] [1716581096.210731833]: gazebo_ros_control plugin is waiting for model URDF in parameter [robot_description] on the ROS param server.
[ERROR] [1716581096.418839624]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/left_wheel_joint
[ERROR] [1716581096.419882119]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/right_wheel_joint
[ INFO] [1716581096.501019570]: Loaded gazebo_ros_control.
```

Lines called out:
`[WARN] [1716581085.578922, 0.000000]: Controller Spawner couldn't find the expected controller_manager ROS interface.`  
and later   
```
[ERROR] [1716581096.418839624]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/left_wheel_joint
[ERROR] [1716581096.419882119]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/right_wheel_joint
```
Following those:  
```
[ WARN] [1716581335.290455005, 7.026000000]: Failed to meet update rate! Took 0.051000000000000003664
[ WARN] [1716581504.009411347, 170.229000000]: Failed to meet update rate! Took 0.054000000000000006328
[ WARN] [1716581596.203703398, 259.627000000]: Failed to meet update rate! Took 0.052000000000000004552
[ WARN] [1716581604.479448187, 267.627000000]: Failed to meet update rate! Took 0.052000000000000004552
[ WARN] [1716581630.003116263, 292.327000000]: Failed to meet update rate! Took 0.052000000000000004552
[ WARN] [1716581652.062719712, 313.627000000]: Failed to meet update rate! Took 0.052000000000000004552
Node::Advertise(): Error advertising topic [/mir/joint_cmd]. Did you forget to start the discovery service?
[ WARN] [1716582339.248689887, 975.328000000]: Failed to meet update rate! Took 0.05300000000000000544
```

#### Implementation notes

