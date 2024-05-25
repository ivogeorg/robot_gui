#include "robot_gui/tiny_robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

#include <string>

RobotGUI::RobotGUI(std::string info_topic)
    : robot_info_topic_name_{info_topic},
      robot_info_sub_{nh_.subscribe<robotinfo_msgs::RobotInfo10Fields>(robot_info_topic_name_, 10,
                                    &RobotGUI::robot_info_callback, this)} {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO_STREAM("Tiny Robot Control GUI: initialized");
}

void RobotGUI::robot_info_callback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info_data) {
  robot_info_data_ = *robot_info_data;
  ROS_INFO_STREAM("Tiny Robot Control GUI: received robot info data");
}