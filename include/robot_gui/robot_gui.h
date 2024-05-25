#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "ros/service_client.h"
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <ros/ros.h>
#include <string>


class RobotGUI {
    std::string robot_info_topic_name_;
    std::string odometry_topic_name_;
    std::string velocity_topic_name_;
    std::string get_distance_service_name_;
    std::string reset_distance_service_name_;

    ros::NodeHandle *nh_;

    ros::Subscriber robot_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient get_distance_client_;
    ros::ServiceClient reset_distance_client_;

    void robot_info_callback();
    void odom_callback();

public:
    RobotGUI();
    ~RobotGUI();

    void run();
};

#endif