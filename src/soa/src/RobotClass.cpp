#include "nav_msgs/Odometry.h"
#include <nav_msgs/GetMap.h> 
#include "iostream"
#include <nav_msgs/Path.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "RobotClass.hpp"

/// Callback for velocity - not used but needed as place holder
void callback_vel(const std_msgs::String::ConstPtr& msg) {
}
/// Callback for odom - not used but needed as place holder
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
}

/// Robot class Constructor Function, requires the node handle and robot id
RobotClass::RobotClass (ros::NodeHandle &nh, int id){
    nh_ = nh;
    id_ = id; 
    alive = true; 
    std::cout << "Constructor Robot Class called" << std::endl;
    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/rosbot/cmd_vel", 1);

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/rosbot/odom_msg_calculated", 1);

    // Subscribe to topics
    ros::Subscriber sub1 = nh_.subscribe("/cmd_vel", 1000, &callback_vel);
    std::string robot_topic = "/robot" + std::to_string(id_) + "/odom";
    ros::Subscriber odom_sub = nh_.subscribe(robot_topic, 1000, &callback_odom);
  
}

/// hard coded map to convert x and y coordinates to one dimensional id 
int RobotClass::koords_to_node_id(int x, int y){
  return map_ids[y][x];  //This array is "mirrored" along its diagonal to match the map
}

/// Prints the Map Array
void RobotClass::printMapArray(int x_start, int y_start) {
  std::string ObstaclArrayString[MAP_SIZE_X][MAP_SIZE_Y]; //Karte als 2D-Array mit Hindernissen

  printf("\n");
  for (int i = 0; i < MAP_SIZE_X; i++) {
    for (int j = 0; j < MAP_SIZE_Y; j++) {
      if (i == x_start && j == y_start) {
        printf("S ");
      } else if (ObstaclAarray[i][j] == 2) { //path
        printf("* ");
      } else if (ObstaclAarray[i][j] == 1) { //Obstacle or wall
        printf("X ");
      } else {
          printf("  ");
      }
    }
    printf("\n");
  }
}

/// function to find a path based on a start node and goal node - inspired and based on the BMR6 Project
nav_msgs::Path RobotClass::findPath(int start_idx, int ziel_idx, int catching_robot_x, int catching_robot_y) {  // Inspired by mmr6 
  // subscribe to /static_map
  ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv_map;
  nav_msgs::Path final_path;
  geometry_msgs::PoseStamped pose;

  final_path.header.stamp = ros::Time::now();
  final_path.header.frame_id = "map";

  if (client.call(srv_map)) {
    const nav_msgs::OccupancyGrid& map_one_d (srv_map.response.map); //one dimensional map array 

    int DistanceArrayOneDim[INT16_MAX]; //one dimensional distance array
    int OneDimMap[INT16_MAX] = {0};
    std::copy(std::begin(map_one_d.data), std::end(map_one_d.data), std::begin(OneDimMap));

    int k = 0;
    for (int i = 0; i < MAP_SIZE_X; i++) {

      for (int j = 0; j < MAP_SIZE_Y; j++) {
        if (map_one_d.data[k] == 0) {
          ObstaclAarray[j][i] = 0; //swapped i&j
        } else {
          ObstaclAarray[j][i] = 1; //swapped i&j
        }
        if(j == catching_robot_x && i == catching_robot_y) {
          OneDimMap[k] = 1;
        }
        k++;
      }
    }

    if(catching_robot_x >= 0 && catching_robot_y >= 0) {
      ObstaclAarray[catching_robot_x][catching_robot_y] = 1; //x and y already mirrored
    }

    // **************** //
    //  Breadth first search    //

    //Step 1: Set the distance to every node to the maximum value
    //Step 2: initialise queue
    int queue[INT16_MAX]; //Node ID wird hier gespeichert
    for (int i = 0; i < INT16_MAX; i++) {
      queue[i] = -1;
      DistanceArrayOneDim[i] = INT16_MAX;
    }
    //Step 3: set queue starting point (Index 0) to the start position
    queue[0] = start_idx;
    DistanceArrayOneDim[start_idx] = 0;

    // Step 4: execute algorithm
    int iteration_counter = 0;
    int added_elements_counter = 1;
    int v, n;
    while (iteration_counter < added_elements_counter)
    {
      /*1*/
      v = queue[iteration_counter];
      //Check node neighbors! diagonal moves not allowed
      //right +1; left -1; up -MAP_SIZE_Y; down +MAP_SIZE_Y
      /*2*/

      //right
      n =  queue[iteration_counter] + 1; //index
      if (OneDimMap[n] == 0 && (DistanceArrayOneDim[n] > (DistanceArrayOneDim[v] + 1))) {
        DistanceArrayOneDim[n] = DistanceArrayOneDim[v] + 1;
        queue[added_elements_counter] = n;
        added_elements_counter++;
      } else {
        //Obstacle
      }

      //down
      n =  queue[iteration_counter] + MAP_SIZE_Y; //index
      if (OneDimMap[n] == 0 && (DistanceArrayOneDim[n] > (DistanceArrayOneDim[v] + 1))) {
        DistanceArrayOneDim[n] = DistanceArrayOneDim[v] + 1;
        queue[added_elements_counter] = n;
        added_elements_counter++;
      } else {
        //Obstacle
      }

      // left
      n =  queue[iteration_counter] - 1; //index
      if (OneDimMap[n] == 0 && (DistanceArrayOneDim[n] > (DistanceArrayOneDim[v] + 1))) {
        DistanceArrayOneDim[n] = DistanceArrayOneDim[v] + 1;
        queue[added_elements_counter] = n;
        added_elements_counter++;
      } else {
        //Obstacle
      }

      //up
      n =  queue[iteration_counter] - MAP_SIZE_Y; //index
      if (OneDimMap[n] == 0 && (DistanceArrayOneDim[n] > (DistanceArrayOneDim[v] + 1))) {
        DistanceArrayOneDim[n] = DistanceArrayOneDim[v] + 1;
        queue[added_elements_counter] = n;
        added_elements_counter++;
      } else {
        //Obstacle
      }

      /*3*/
      iteration_counter++;
    }

    //Map is generated
    //Find back the way to the goal node

    int pathqueue[300]; // Path node indexes
    int i = 0;
    int cur, next;

    //Start at the goal node
    pathqueue[i] = ziel_idx;

    //Path in der DistanceArray
    while (pathqueue[i] != start_idx) { 
      cur = pathqueue[i];
      //rechts
      if (DistanceArrayOneDim[cur + 1] < DistanceArrayOneDim[cur]) {
        pathqueue[i + 1] = cur + 1;
      }
      else if (DistanceArrayOneDim[cur - 1] < DistanceArrayOneDim[cur]) { //links
        pathqueue[i + 1] = cur - 1;
      } else if (DistanceArrayOneDim[cur - MAP_SIZE_Y] < DistanceArrayOneDim[cur]) { //oben
        pathqueue[i + 1] = cur - MAP_SIZE_Y;
      } else if (DistanceArrayOneDim[cur + MAP_SIZE_Y] < DistanceArrayOneDim[cur]) { //oben
        pathqueue[i + 1] = cur + MAP_SIZE_Y;
      }
      i++;
    }
    //Draw path in obstacle array
    int queuesteps = i;
    k = 0;
    for (int i = 0; i < MAP_SIZE_X; i++) {
      for (int j = 0; j < MAP_SIZE_Y; j++) {
        for (int l = 0; l < queuesteps; l++) {
          if (k == pathqueue[l]) {
            ObstaclAarray[j][i] = 2; //Achtung Swaped i&j
          }
        }
        k++;
      }
    }



    //Publish path
    //pathqueue[] contains the idxs of nodes
    std::vector<geometry_msgs::PoseStamped> reversed_path_vector;
    for (int l = 0; l < queuesteps; l++) {
      int node_id = pathqueue[l];
      
      for (int i = 0; i < MAP_SIZE_X; i++) {
        for (int j = 0; j < MAP_SIZE_Y; j++) {
          if(node_id == map_ids[j][i]) {
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = (i);
            pose.pose.position.y = (j);

            //final_path.poses.push_back(pose);
            reversed_path_vector.push_back(pose);
          }
        }
      }
    }
    // reverse the sequence of the path nodes (from start to goal)
    std::reverse(reversed_path_vector.begin(),reversed_path_vector.end());
    if(id_ != 1) {
      reversed_path_vector.pop_back();
    }

    for(const auto& value: reversed_path_vector) {
      final_path.poses.push_back(value);
    }
  }

  return final_path;
}
