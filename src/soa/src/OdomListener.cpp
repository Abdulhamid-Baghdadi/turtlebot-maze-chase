#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "OdomListener.hpp"

/// Odometry Listener to get the current position 
void OdomListener::callback_odom_list(const nav_msgs::Odometry::ConstPtr& msg){
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;

  //std::cout << "robot" << robot_id << " Current: " << current_x << " | " << current_y << std::endl;

  quaternion_x = msg->pose.pose.orientation.x;
  quaternion_y = msg->pose.pose.orientation.y;
  quaternion_z = msg->pose.pose.orientation.z;
  quaternion_w = msg->pose.pose.orientation.w;
}
/// Ofometry Listener Class 
OdomListener::OdomListener(int id) : robot_id{id} {
    //std::cout << "robot_id = " << robot_id << std::endl;
    //current_x = -1;
    //current_y = -1;
}

/// Print info function to get the current information with robot ID 
void OdomListener::print_info() {
  std::cout << "print_info: robot" << robot_id << " Current: " << current_x << " | " << current_y << std::endl;
}
