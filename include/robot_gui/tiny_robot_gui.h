#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "ros/service_client.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>
#include <string>


class RobotGUI {
    // topic and service names
    std::string robot_info_topic_name_;

    // node handle
    ros::NodeHandle nh_;

    // ROS pub/sub/svc client
    ros::Subscriber robot_info_sub_;

    // to hold the data for subscribers
    robotinfo_msgs::RobotInfo10Fields robot_info_data_;

    // callbacks are private
    void robot_info_callback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info_data);

public:
    RobotGUI(
        std::string into_topic);
    ~RobotGUI() = default;
};

#endif