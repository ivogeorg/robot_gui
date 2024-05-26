#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "ros/service_client.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>


class RobotGUI {
    // topic and service names
    std::string robot_info_topic_name_;
    std::string odometry_topic_name_;
    std::string velocity_topic_name_;
    std::string get_distance_service_name_;
    std::string reset_distance_service_name_;

    // node handle
    ros::NodeHandle nh_;

    // ROS pub/sub/svc client
    ros::Subscriber robot_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient get_distance_client_;
    ros::ServiceClient reset_distance_client_;
    
    std_srvs::Trigger svc_trigger_;

    // to hold the data for subscribers
    robotinfo_msgs::RobotInfo10Fields robot_info_data_;
    nav_msgs::Odometry odom_data_;
    double distance_traveled_ = 0;

    // CVUI stuff
    const std::string WINDOW_NAME = "MIR ROBOT CTRL";

    // callbacks are private
    void robot_info_callback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info_data);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &robot_odom_data);

public:
    RobotGUI(
        std::string into_topic, 
        std::string odom_topic, 
        std::string vel_topic,
        std::string dist_svc_name,
        std::string reset_svc_name);
    ~RobotGUI() = default;

    // all the action happens here
    void run();
};

#endif