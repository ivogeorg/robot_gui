### robot_gui

A simple GUI to control and monitor a Mir robot.

#### Unresponsiveness to `/cmd_vel`

Initially the simulation didn't respond to any `/cmd_vel`. It took (re)building all the packages with `catkin_make`. There are still spawning failures, missing frames passed to `lookupTransform`, and update rate failures, but the Mir robot responds to the **Robot Steering** GUI. 

#### Implementation notes

