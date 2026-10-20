#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "iostream"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include "LinearSpeedController.hpp"
#include "OdomListener.hpp"
#include <vector>
//#include <gazebo/DeleteModel.h>



/**

\mainpage

# SOA MMR3 Projekt 6
### Moritz Abdank & Abdulhamid Baghdadi


## Zielsetzung
n Roboter werden in einer ROS/Gazebo umgebung gespawned. Roboter A soll alle anderen Roboter B fangen. Diese laufen allerdings vor Roboter A weg. 

## Aufbau
- n Roboter werden gespawned (User Input via Terminal nbr_robots:=n)
- Roboter 0 wird zu Roboter A und versucht die anderen (n-1) Roboter B zu fangen 
- Roboter A sucht sich einen Pfad zu dem nähesten Roboter B und fährt dort hin
- In jedem zyklus wird evaluiert ob ein näherer Roboter existiert
- Pfad wird entsprechend angepasst 
- Wenn Roboter A nahe an Roboter B ist beginnt dieser weg zu laufen.
- Dazu sucht Roboter B eine Ziel Pose die nicht in dem Quadranten des Roboters A liegt.
- Roboter B flüchtet.
- Fängt Roboter A Roboter B wird dieser aus der Gazebo Welt gelöscht und in der Steuerung deaktiviert. -> so kommt es zu keinen Kollisionen mit "toten" Robotern.
- Sind alle Roboter gefangen gibt das Programm am Terminal eine Info aus und das "Spiel" ist zu ende.  

## Kompilieren und Ausführen
Das Package einfach mittels _>> catkin_make <<_ kompilieren und mittels _>> roslaunch soa launch_all.launch nbr_robots:=n <<_ launchen. Wobei n mit einer gewünschten natürlichen Zahl ersetzt werden soll. 

Als Zusatzargument gibt es noch den speed_factor (optional):  
Float Wert zum einstellen der Geschwindigkeitsunterschiede der Roboter A und B (Factor: Robot_A/Robot_B)
0-0.99: Roboter B ist schnellr
1: Roboter sind gleich schnell
>1.1: Roboter A ist schneller 
Default: 2.0 

**/




/// Defines the numbers of used Roboters (Argument from command line nbr_robots)
int nbr_robots = 0;
/// Defines the speed difference between Robot A and Robot B (0-0.99 Robot B is faster | 1 Robots are equal | >1 Robot A is faster)  
float speed_factor = 2; 
/// Vector of Instances of Robot Class 
std::vector<RobotClass *> ROBOTS; 
/// Vector of Instances of the Odometry listener -> used to get the koordinates of each robt
std::vector<OdomListener *> ROBOTS_ODOM_LISTENER;  
/// Vector of Instances of the odom subscriber -> is needed to link each topic for each robot instance
std::vector<ros::Subscriber> ODOM_SUBSCRIBER; 
/// Vector to publish a path for each robot 
std::vector<ros::Publisher> CREATE_PATH_ROBOT;
/// Vector to publish a drive command for each robot 
std::vector<ros::Publisher> VEL_PUB_ROBOT;
/// Vector to instances of the speed controller for each robot (could probably be reduced to just one for all)
std::vector<LinearSpeedController *> CONTROLLER_LINEAR_SPEED_ROBOT;
/// Vector of all the drive commands which will be published
std::vector<geometry_msgs::Twist> VEL_MSG_ROBOTS;
/// defined geometry msg to stop a robot -> can be used with a simple loop to stop all robots 
geometry_msgs::Twist stop_robot;


/// topic string to create a topic for each robot with his ID  
std::string topic; 
/// base string for namespace -> robot+ID+topic
std::string base_topic = "robot" ;
/// topic to add on for the odometry
std::string add_on_topic_odom = "/odom"; 
/// topic to add on for the drive commands
std::string add_on_topic_vel = "/cmd_vel"; 


/// Hard coded map for easy check if new goal is available 
int hard_coded_map[16][16] = {
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,},
{0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,},
{0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,},
{0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,},
{0,1,0,0,1,1,1,1,0,0,1,0,0,0,1,0,},
{0,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0,},
{0,1,0,0,0,0,0,1,0,0,0,1,0,0,1,0,},
{0,1,1,1,0,0,0,1,0,0,0,1,0,0,1,0,},
{0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,0,},
{0,1,0,0,1,1,0,1,0,0,0,1,0,0,1,0,},
{0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,},
{0,1,0,0,0,0,0,1,0,0,0,1,0,0,1,0,},
{0,1,0,0,0,0,0,1,0,0,0,1,0,0,1,0,},
{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,}};


/// Function to delete the robot model from the gazebo world, takes the Robot ID 
void kill_robot(int id){
  // Issues with adding Gazebo to the CMAKE
  // if there were no issues and more time we would have implemented a direct service call to kill the robots instead of a command call 
  // Code for the service call: 
  /*
  std::string model_name = "robot" + std::to_string(id); 
  ros::service::waitForService("gazebo/delete_model");
  ros::ServiceClient deleteModelClient = nh.serviceClient<gazebo::DeleteModel>("gazebo/delete_model");
  gazebo::DeleteModel deleteModel;
  deleteModel.request.model_name = model_name;
  deleteModelClient.call(deleteModel);
  */
  std::string command = std::string("gnome-terminal -e \"rosservice call gazebo/delete_model \'{model_name: robot" + std::to_string(id+1)  + "}\'\"");
  system(command.c_str());
  std::cout << command << std::endl; 
  std::cout << "kill robot aufgerufen " << std::endl; 

}

/// function for creating all instances needed for the programm. 
void spawn_robots(ros::NodeHandle nh) {
  stop_robot.linear.x = 0;
  stop_robot.angular.z = 0;

  for (int i=0; i< nbr_robots; i++){
    // ROBOT CLASS
    ROBOTS.push_back(new RobotClass(nh, (i+1)));
    // ROBOT LISTENERS
    ROBOTS_ODOM_LISTENER.push_back(new OdomListener(i+1)); 
    // ROBOT ODOM SUBSCRIBERS
    topic = base_topic+std::to_string(i+1)+add_on_topic_odom; 
    std::cout << "TOPICS: " << base_topic+std::to_string(i+1)+add_on_topic_odom << std::endl; 
    ODOM_SUBSCRIBER.push_back(nh.subscribe(topic, 1000, &OdomListener::callback_odom_list, ROBOTS_ODOM_LISTENER[i])); //TODO: Brauchen wir hier auch einen pointer
    // ROBOT PATH PUBLISHER
    CREATE_PATH_ROBOT.push_back(nh.advertise<nav_msgs::Path>(("/path_robot"+std::to_string(i+1)), 1));
    // ROBOT VELOCITY PUBLISHERS 
    topic = base_topic+std::to_string(i+1)+add_on_topic_vel; 
    VEL_PUB_ROBOT.push_back(nh.advertise<geometry_msgs::Twist>(topic, 10));
    // linear speed controller
    CONTROLLER_LINEAR_SPEED_ROBOT.push_back(new LinearSpeedController());  //TODO: BRAUCHEN WIR HIER ALLE INSTANZEN ODER REICHT EINER? 
    // vel messages
    VEL_MSG_ROBOTS.push_back(stop_robot);
  }


  // spawning all roboters
  std::string command; 
  int rand_spawn_x, rand_spawn_y; 
  for(int i=0; i<nbr_robots; i++){
    do{
      rand_spawn_x = rand() %13 +1; 
      rand_spawn_y = rand() %13 +1;
    }while(hard_coded_map[rand_spawn_x][rand_spawn_y] != 1);  

    command = command + std::string("gnome-terminal -e \"roslaunch soa spawn_one_robot.launch robot_name:=robot")+std::to_string(i+1)+" tf_prefix:=robot"+std::to_string(i+1)+"_tf "
  + "x:=" + std::to_string(float(rand_spawn_x + 0.5)) + " y:=" + std::to_string(float(rand_spawn_y+0.5)) + "\";";
  }
  system(command.c_str());

  //stop all robots
  for (int i=0; i<nbr_robots; i++){
    VEL_PUB_ROBOT[i].publish(VEL_MSG_ROBOTS[i]);
  }
}

/// Function to convert the x and y coordinates to the matching quadrant. Is used to get a goal outside of the Quadrant of Robot A
int get_quadrant(int x, int y){
  // 16x16 map 
  // idxs:  x   |  y
  // Q1 -> 0-7  | 0-7
  // Q2 -> 0-7  | 8-15
  // Q3 -> 8-15 | 0-7
  // Q4 -> 8-15 | 8-15
  
  
  int quadrant; 
  if(x<8){
    // has to be Q1 or Q2
    if(y<8){
      quadrant = 1;
    }else{
      quadrant = 2; 
    }
  }else{
    // has to be Q3 or Q4
    if(y<8){
      quadrant = 3;
    }else{
      quadrant = 4; 
    }
  }  
  
  return quadrant; 
}




/// Main Function
int main(int argc, char **argv) {
	srand(time(NULL));   // Initialization, should only be called once.
  // TODO: Errorhandling!!!
  nbr_robots = *argv[1] - '0'; //convert char to int
  speed_factor = std::atof(argv[2]); 
  speed_factor = std::abs(speed_factor);
  std::cout << "SPEED FACTOR: " << speed_factor << std::endl; 
  int start_robot1, fin_robot1, quadrant_robot_1; 
  int start_neares_robot, fin_neares_robot, quadrant_nearest_robot;
  int robots_left = nbr_robots-1; 

  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

  spawn_robots(nh);

  nav_msgs::Path path_robot1; 
  nav_msgs::Path path_nearest_robot; 
  
  // PUBLISH den Path
  int activate_vel = 0; 
  ros::Rate loop_rate(10);
  std::cout << "BOOTING [ ";
  int fleeingtime = 200;
  int fleeingtime_counter = fleeingtime; 
  int nearest_robot = 2;  
  int rand_x, rand_y;
  int smallest_path; 
  int goal_quadrant = 0;
  while ( ros::ok( ) && (robots_left>0)) {        // Do till ROS error occurs (e.g.: strg+C or ROS shutdown signal)
    //robotClass_1.printMapArray(robotClass_1.ObstaclAarray);   
    if(activate_vel > 20){
      start_robot1 = ROBOTS.at(0)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(0)->current_x, ROBOTS_ODOM_LISTENER.at(0)->current_y); //achtung wenn odom nicht erreicht wird -> node dies speicherzugrifffehler
      quadrant_robot_1 = get_quadrant(ROBOTS_ODOM_LISTENER.at(0)->current_x, ROBOTS_ODOM_LISTENER.at(0)->current_y); 
      //find nearest robot!
      nav_msgs::Path tmp_path; 
      int robot_k_location; 
      smallest_path = 10000;
      for(int k = 1; k<nbr_robots; k++){
        if(ROBOTS[k]-> alive){ //only if robot is still active
          robot_k_location = ROBOTS.at(0)->koords_to_node_id(ROBOTS_ODOM_LISTENER[k]->current_x, ROBOTS_ODOM_LISTENER[k]->current_y);
          path_robot1 = ROBOTS.at(0)->findPath(start_robot1, robot_k_location, -1, -1);
          if(path_robot1.poses.size() < smallest_path){
            smallest_path = path_robot1.poses.size();
            nearest_robot = k;
            //std::cout << "nearest Robot: " << k+1 << " with path size: " << smallest_path << std::endl;  
          }
        }
      }
      fin_robot1 = ROBOTS.at(0)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_x, ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_y);
      quadrant_nearest_robot = get_quadrant(ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_x, ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_y);
      path_robot1 = ROBOTS.at(0)->findPath(start_robot1, fin_robot1, -1, -1);
      VEL_MSG_ROBOTS.at(0) = CONTROLLER_LINEAR_SPEED_ROBOT.at(0)->get_cmd_vel(path_robot1, *(ROBOTS_ODOM_LISTENER.at(0)));
      VEL_MSG_ROBOTS.at(0).linear.x = VEL_MSG_ROBOTS.at(0).linear.x * speed_factor; //change here for advantage of robot1
      VEL_PUB_ROBOT.at(0).publish(VEL_MSG_ROBOTS.at(0));
      
      //handle robots when cought
      if (path_robot1.poses.size() < 2 ){ //set when "cought"
        
        ROBOTS[nearest_robot]->alive = false; 
        kill_robot(nearest_robot); 
        robots_left = robots_left-1; 
      }

      //stop every robot except for the catching and nearest robot
      for(int k = 1; k<nbr_robots; k++){
        if(k != nearest_robot) {
          VEL_PUB_ROBOT[k].publish(stop_robot);
          nav_msgs::Path empty_path;
          CREATE_PATH_ROBOT.at(k).publish (empty_path);
        }
      }

      // fleeing algorithm 
      if(path_robot1.poses.size() < 300){ //if robot1 to close -> find new goal
        //std::cout << "robot2 random goal" << std::endl;
        //rand_goal = rand() %254 +1; 
         
        //int path_size = 0;
        if(fleeingtime_counter >= fleeingtime){ //"Timer" Based
          fleeingtime_counter = 0; 
          do{
            rand_x = rand() %13 +1; 
            rand_y = rand() %13 +1; 
            goal_quadrant = get_quadrant(rand_x, rand_y);

            //start_neares_robot = ROBOTS.at(nearest_robot)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_x, ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_y); 
            //fin_neares_robot = ROBOTS.at(nearest_robot)->koords_to_node_id(rand_x, rand_y);
            //path_nearest_robot = ROBOTS.at(nearest_robot)->findPath(start_neares_robot, fin_neares_robot, ROBOTS_ODOM_LISTENER.at(0)->current_x, ROBOTS_ODOM_LISTENER.at(0)->current_y);          
            //path_size = path_nearest_robot.poses.size();
          } while((hard_coded_map[rand_x][rand_y] != 1) && (goal_quadrant == quadrant_robot_1));
          //} while(hard_coded_map[rand_x][rand_y] != 1); //working well :)
        }

        fin_neares_robot = ROBOTS.at(nearest_robot)->koords_to_node_id(rand_x, rand_y);
        start_neares_robot = ROBOTS.at(nearest_robot)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_x, ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_y);
        path_nearest_robot = ROBOTS.at(nearest_robot)->findPath(start_neares_robot, fin_neares_robot, ROBOTS_ODOM_LISTENER.at(0)->current_x, ROBOTS_ODOM_LISTENER.at(0)->current_y);
        VEL_MSG_ROBOTS.at(nearest_robot) = CONTROLLER_LINEAR_SPEED_ROBOT.at(nearest_robot)->get_cmd_vel(path_nearest_robot, *(ROBOTS_ODOM_LISTENER.at(nearest_robot)));   
        VEL_PUB_ROBOT.at(nearest_robot).publish(VEL_MSG_ROBOTS.at(nearest_robot));
      }

      
      //int start_robot1 = ROBOTS.at(0)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(0)->current_x, ROBOTS_ODOM_LISTENER.at(0)->current_y); //achtung wenn odom nicht erreicht wird -> node dies speicherzugrifffehler
      //ROBOTS.at(0)->printMapArray(start_robot1);

      //int start_robot2 = ROBOTS.at(1)->koords_to_node_id(ROBOTS_ODOM_LISTENER.at(1)->current_x, ROBOTS_ODOM_LISTENER.at(1)->current_y); //achtung wenn odom nicht erreicht wird -> node dies speicherzugrifffehler
      ROBOTS.at(nearest_robot)->printMapArray(ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_x, ROBOTS_ODOM_LISTENER.at(nearest_robot)->current_y);

      fleeingtime_counter++; 
    }else{
      // BOORING LOOP - stalls the process to let Gazebo boot
      activate_vel++;
      std::cout << "#";
      if(activate_vel == 21){
        std::cout << " ]" << std::endl; 
      }
    }
    

    
    CREATE_PATH_ROBOT.at(0).publish (path_robot1);
    CREATE_PATH_ROBOT.at(nearest_robot).publish (path_nearest_robot);
    
    std::cout << "\n    **** Game Stats: ****" << std::endl; 
    std::cout << ">> Currently chasing Robot" << nearest_robot << std::endl;
    std::cout << ">> Distance to Robot" << nearest_robot << ": " << smallest_path << "m" << std::endl;   
    std::cout << ">> Robot A in Quadrant: " << quadrant_robot_1 << std::endl;
    std::cout << ">> Robot" << nearest_robot << " trying to reach Quadrant: " << goal_quadrant << std::endl;
    std::cout << ">> Life Status: " << std::endl; 
    for(int y = 1; y<nbr_robots; y++){
      if(ROBOTS[y]->alive){
        std::cout << ">> Robot" << y << ": still in game" << std::endl; 
      }else{
        std::cout << ">> Robot" << y << ": cought" << std::endl; 
      }
    }
 std::cout << "    **** *********** ****" << std::endl; 

    ros::spinOnce();           // Update buffer
    loop_rate.sleep();
  }


  std::cout << "***************************" << std::endl; 
  std::cout << "*                         *" << std::endl; 
  std::cout << "*  ALL ROBOTS ARE COUGHT  *" << std::endl; 
  std::cout << "                          *" << std::endl; 
  std::cout << "***************************\n\n" << std::endl; 
  
  std::cout << "[Node INFO] >> Game Controler Node closed << " << std::endl; 

}
//