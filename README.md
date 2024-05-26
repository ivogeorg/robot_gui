### robot_gui

A simple GUI to control and monitor a Mir robot.

#### Submission notes

##### Run
1. Compile all packages with `catkin_make`. Otherwise the Mir Gazebo maze simulation won't work and messages published to `/cmd_vel` won't have any effect.
2. If necessary, compile the robot GUI control node with `catkin_make --only-pkg-with-deps robot_gui`.
3. Launch the Mir Gazebo maze simulation with `roslaunch mir_gazebo mir_maze_world.launch`.
4. Run the distance tracker service node with `rosrun distance_tracker_service distance_tracker_service`.
5. Run the robot info node with `rosrun robot_info agv_robot_info_node`.
6. Run the robot control GUI node with `rosrun robot_gui robot_gui_node`.

##### General notes
1. All the required elements were implemented and tested per the grading guide.
2. There are files under `include/robot_gui` and `src` that start with `tiny_`. They were used to hunt down a subtle bug where including the `ros::NodeHandle` in the `RobotGUI` constructor init list with a name invalidated the object and any subsequent subscriptions, advertisements, and service client registrations were ineffective. Please, ignore them.
3. There are several tags that follow the progress of the implementation. They are all before the tag required for the submission. It will be added after these notes are finished.
4. I opted out of the optional reset distance service, but it would have been a straightforward addition to the `distance_tracker_service` package files and the addition of a "Reset" CVUI button to the robot control GUI.

#### Unresponsiveness to `/cmd_vel`

Initially the simulation didn't respond to any `/cmd_vel`. It took (re)building all the packages with `catkin_make`. There are still spawning failures, missing frames passed to `lookupTransform`, and update rate failures, but the Mir robot responds to the **Robot Steering** GUI. 

#### Implementation notes

##### Table

Element | CVUI | ROS | Notes
--- | --- | --- | ---
Gen Info Area | Window with title | Sub `/robot_info` | Print multiple lines
Teleop Buttons | Buttons | Pub `/cmd_vel` | By increments vertical/horizontal
Current velocity | Window with title | Sub `/odom` | Linear `x`, angular `z`
Robot position | Window with title | Sub `/odom` | Linear `x`, `y`, `z` separate
Distance traveled | Trigger button + window with title | Service client | `/get_distance`
Reset distance | Trigger button | Service client | _(Optional)_ `/reset_distance`

##### CVUI Stuff

1. The frame is defined as `cv::Mat(int rows, int cols, int type)`, so vertical dimension (`y`) is first, and horizontal (`x`) is second.
2. `cvui` functions (`button`, `text`, `window`, etc.), however, all take the position of the element as (`x`, `y`).
3. Origin is UP-LEFT, so `x` grows to the **right** from the left, and `y` grows **down** from the top.
