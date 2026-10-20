#include "LinearSpeedController.hpp"

// LINEARE REGELUNG TESTS START
// get "goal" -> last entry from Path
// get odom 
// calc linear controller vel 
//Defines: k[] ändern für andere Gewichtung


/// Calculates the linear speed based on the distance
float LinearSpeedController::calcv(){
  float dX, dY, roh;
  dX = goal_x - current_x; 
  dY = goal_y - current_y;
  roh = sqrt((dX*dX) + (dY*dY));

  return (SpeedMultiplyLin * roh);   
}

/// Calculates the angle to later set an angular speed accordingly 
float LinearSpeedController::calc_alpha(){
    double roll, pitch, yaw;

    tf::Quaternion q(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    current_theta = yaw* (180/PI);
    current_theta = current_theta < 0 ? current_theta+360 : current_theta;

    float dX = goal_x - current_x; 
    float dY = goal_y - current_y;
    float goal_theta;

    int speed = 1;
    float from_angle = current_theta;

    float to_angle = atan2(dY, dX)*(180 / PI);
    to_angle = to_angle < 0 ? to_angle+360 : to_angle;

    if(from_angle == to_angle ||
      from_angle == 0  && to_angle == 360 ||
      from_angle == 360 && to_angle == 0)
    {
        return 0;
    }

    float left = (360 - from_angle) + to_angle;
    float right = from_angle - to_angle;


    if(from_angle < to_angle)  {
        if(to_angle > 0) {
            left = to_angle - from_angle;
            right = (360 - to_angle) + from_angle;
        } else {
            left = (360 - to_angle) + from_angle;
            right = to_angle - from_angle;
        }
    }
  
    // Get the shortest angular direction and return it 
    return ((left <= right) ? left : (right * -1));
}

/// calculates the cmd_vel data and returns it so the publisher can publish the data to the matching topic
geometry_msgs::Twist LinearSpeedController::get_cmd_vel(nav_msgs::Path PATH, OdomListener odom_data){
  if(!PATH.poses.empty()) {
    int goal_path_idx = 0;
    goal_x = PATH.poses[goal_path_idx].pose.position.x + PATHOFFSET_X; 
    goal_y = PATH.poses[goal_path_idx].pose.position.y + PATHOFFSET_Y;

    quaternion_x = odom_data.quaternion_x;
    quaternion_y = odom_data.quaternion_y;
    quaternion_z = odom_data.quaternion_z;
    quaternion_w = odom_data.quaternion_w;

    // get current data   
    current_x = odom_data.current_x;
    current_y = odom_data.current_y;


    if(goal_x > 100 || goal_y > 100) {
      drive_data.angular.z = 0;
      drive_data.linear.x = 0;
      return drive_data; 
    }

    // drive command 
    float alpha = calc_alpha();
    float angular_speed;
    if(alpha > 0) {
      if (alpha > 10) {
        angular_speed = 0.5;
      } else {
        angular_speed = 0.2;
      } 
    } else {
      if(alpha < -10) {
        angular_speed = -0.5;
      } else {
        angular_speed = -0.2;
      }
    }
    drive_data.angular.z = angular_speed * SpeedMultiplyAng;

    // Static aproximation of a PID Controller for our P-Controller -> we added 2 speed stages so on a linear path it drives faster 
    if( PATH.poses.size() > 0) {
      if (abs(calc_alpha()) > 10) {
          drive_data.linear.x = 0.03 ;   
      } else {
          drive_data.linear.x = 0.1 ; 
      }
    }
  }
  return drive_data; 
}