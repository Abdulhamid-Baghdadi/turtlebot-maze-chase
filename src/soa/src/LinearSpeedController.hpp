#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "iostream"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "RobotClass.hpp"
#include <time.h>
#include "OdomListener.hpp"

/// Nodes/Map is a discrete calculation -> needs an offset so the robot does not "drive" in the wall
#define PATHOFFSET_X 0.5
/// Nodes/Map is a discrete calculation -> needs an offset so the robot does not "drive" in the wall
#define PATHOFFSET_Y 0.5
/// Speed Multiplication for angular speed to tune the whole system
#define SpeedMultiplyAng 1
/// Speed Multiplication for linear speed to tune the whole system
#define SpeedMultiplyLin 1


// LINEARE REGELUNG TESTS START

/// Class for linear speed controll. Inspired by the BMR6 Project of linear controlls
class LinearSpeedController{
  public: 
    LinearSpeedController(){};

    // Berechnung der Fahrbefehle
    float calcv();
    float calc_alpha();
    geometry_msgs::Twist get_cmd_vel(nav_msgs::Path PATH, OdomListener odom_data);

    float PI = 3.14159265;
    //Variablen: 
    //winkel
    float current_theta, dtheta,beta;
    float quaternion_x, quaternion_y, quaternion_z, quaternion_w;  
    //Werte
    float current_x, current_y; 
    //Soll-Position
    float goal_x, goal_y; 
    //Drive-Data
    geometry_msgs::Twist drive_data;
};

// LINEARE REGELUNG TESTS ENDE