#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

/// Class of the Odom Listener. Used to get the current x and y koordinates of each robot 
class OdomListener{ //Instanz notwendig um auf callback zugreifen zu können 
  public:
    volatile int robot_id;
    volatile float current_x, current_y;
    float quaternion_x, quaternion_y, quaternion_z, quaternion_w;  
    //bool start = false;

    void callback_odom_list(const nav_msgs::Odometry::ConstPtr& msg);
    void print_info();

    OdomListener(int id);
};

