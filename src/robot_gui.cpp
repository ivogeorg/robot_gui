#include "robot_gui/robot_gui.h"
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "ros/init.h"
#define CVUI_IMPLEMENTATION

#include <string>

RobotGUI::RobotGUI(std::string into_topic, std::string odom_topic,
                   std::string vel_topic, std::string dist_svc_name,
                   std::string reset_svc_name)
    : robot_info_topic_name_{into_topic}, odometry_topic_name_{odom_topic},
      velocity_topic_name_{vel_topic},
      get_distance_service_name_{dist_svc_name},
      reset_distance_service_name_{reset_svc_name},
      robot_info_sub_{nh_.subscribe<robotinfo_msgs::RobotInfo10Fields>(
          robot_info_topic_name_, 10, &RobotGUI::robot_info_callback, this)},
      odom_sub_{nh_.subscribe<nav_msgs::Odometry>(
          odometry_topic_name_, 10, &RobotGUI::odom_callback, this)},
      cmd_vel_pub_{
          nh_.advertise<geometry_msgs::Twist>(velocity_topic_name_, 10)},
      get_distance_client_{
          nh_.serviceClient<std_srvs::Trigger>(get_distance_service_name_)},
      reset_distance_client_{
          nh_.serviceClient<std_srvs::Trigger>(reset_distance_service_name_)} {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO_STREAM("Robot Control GUI: initialized");
}

void RobotGUI::robot_info_callback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info_data) {
  robot_info_data_ = *robot_info_data;
  ROS_INFO_STREAM("Robot Control GUI: received robot info data");
}

void RobotGUI::odom_callback(
    const nav_msgs::Odometry::ConstPtr &robot_odom_data) {
  odom_data_ = *robot_odom_data;
  ROS_INFO_STREAM("Robot Control GUI: received robot odometry data");
}

void RobotGUI::run() {
  // Named window

  // Frame

  // while()
  //   Window: General info area
  //   Buttons: Teleoperation buttons
  //   Windows: Current velocity (linear, angular)
  //   Windows: Robot position (x, y, z)
  //   Buttons: Get/reset distance
  //   Window: Distance traveled
  //
  //   update
  //   show
  //   if (ESC) break
  ros::Rate rate(10);
  while (ros::ok()) {
    ROS_INFO_STREAM("Robot Control GUI: ran loop");    
    ros::spinOnce();
    rate.sleep();
  }
}
