#include "robot_gui/robot_gui.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "ros/init.h"
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

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
  ROS_DEBUG_STREAM("Robot Control GUI: received robot info data");
}

void RobotGUI::odom_callback(
    const nav_msgs::Odometry::ConstPtr &robot_odom_data) {
  odom_data_ = *robot_odom_data;
  ROS_DEBUG_STREAM("Robot Control GUI: received robot odometry data");
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(900, 300, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  // while:
  //   Windows: Robot position (x, y, z) (cvui-position)
  //   Buttons: Get/reset distance (cvui-distance-svc)
  //   Window: Distance traveled (cvui-distance-show)
  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 20, 20, 260, 225, "Robot information");

    // Show the robot info (cvui-info)
    int y_step = 20, y_start = 45, info_color = 0xf0f0f0;
    cvui::printf(frame, 25, y_start, 0.4, info_color, "%s",
                 robot_info_data_.data_field_01.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_02.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_03.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_04.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_05.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_06.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_07.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_08.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_09.c_str());
    cvui::printf(frame, 25, y_start += y_step, 0.4, info_color, "%s",
                 robot_info_data_.data_field_10.c_str());

    // Velocity teleoperation buttons (cvui-teleop)
    int origin_x = 5, origin_y = 255, btn_width = 90, btn_height = 70;
    if (cvui::button(frame, origin_x + 100, origin_y + 0, btn_width, btn_height,
                     " Forward ")) {
      // The button was clicked, update the Twist message
      twist_msg_.linear.x = twist_msg_.linear.x + linear_x_step_;
    }

    if (cvui::button(frame, origin_x + 100, origin_y + 75, btn_width,
                     btn_height, "   Stop  ")) {
      // The button was clicked, update the Twist message
      twist_msg_.linear.x = 0.0;
      twist_msg_.angular.z = 0.0;
    }

    if (cvui::button(frame, origin_x + 5, origin_y + 75, btn_width, btn_height,
                     " Left ")) {
      // The button was clicked, update the Twist message
      twist_msg_.angular.z = twist_msg_.angular.z + angular_z_step_;
    }

    if (cvui::button(frame, origin_x + 195, origin_y + 75, btn_width,
                     btn_height, " Right ")) {
      // The button was clicked, update the Twist message
      twist_msg_.angular.z = twist_msg_.angular.z - angular_z_step_;
    }

    if (cvui::button(frame, origin_x + 100, origin_y + 150, btn_width,
                     btn_height, "Backward")) {
      // The button was clicked,update the Twist message
      twist_msg_.linear.x = twist_msg_.linear.x - linear_x_step_;
    }
 
    // Publish velocity (topic /cmd_vel takes a Twist message)
    cmd_vel_pub_.publish(twist_msg_);

    // Current velocity (cvui-velocity)
    int vel_color = 0xf0f0f0;
    cvui::window(frame, 10, 485, 140, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 25, 510, 0.4, vel_color, "%.02f m/sec",
                 odom_data_.twist.twist.linear.x);

    cvui::window(frame, 150, 485, 140, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 165, 510, 0.4, vel_color, "%.02f rad/sec",
                 odom_data_.twist.twist.angular.z);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}
