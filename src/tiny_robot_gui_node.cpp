#include "robot_gui/tiny_robot_gui.h"
#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_control_gui_node");

  std::string info_topic{"robot_info"};

  RobotGUI robotGUI(info_topic);

  ros::spin();

  return 0;
}
