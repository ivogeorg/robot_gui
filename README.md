### robot_gui

A simple GUI to control and monitor a Mir robot.

#### Unresponsiveness to `/cmd_vel`

Initially the simulation didn't respond to any `/cmd_vel`. It took (re)building all the packages with `catkin_make`. There are still spawning failures, missing frames passed to `lookupTransform`, and update rate failures, but the Mir robot responds to the **Robot Steering** GUI. 

#### Implementation notes

Element | CVUI | ROS | Notes
--- | --- | --- | ---
Gen Info Area | Window with title | Sub `/robot_info` | Print multiple lines
Teleop Buttons | Buttons | Pub `/cmd_vel` | By increments vertical/horizontal
Current velocity | Window with title | Sub `/odom` | Linear `x`, angular `z`
Robot position | Window with title | Sub `/odom` | Linear `x`, `y`, `z` separate
Distance traveled | Trigger button + window with title | Service client | `/get_distance`
Reset distance | Trigger button | Service client | _(Optional)__ `/reset_distance`
