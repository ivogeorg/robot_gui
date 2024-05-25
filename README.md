### robot_gui

A simple GUI to control and monitor a Mir robot.

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
