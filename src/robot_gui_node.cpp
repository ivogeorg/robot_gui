#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/spinner.h"
#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_control_gui_node");

  std::string info_topic{"robot_info"};
  std::string odom_topic{"odom"};
  std::string vel_topic{"cmd_vel"};
  std::string dist_svc_name{"get_distance"};
  std::string reset_svc_name{"reset_distance"};

  RobotGUI robotGUI(info_topic, odom_topic, vel_topic, dist_svc_name,
                    reset_svc_name);

  robotGUI.run();

//   ros::spin();

  return 0;
}
