//////////////////////////////////////////////////////
//  droneObstacleDistanceCalculatorROSModuleNode.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////





//I/O stream
//std::cout
#include <iostream>


// ROS
#include "ros/ros.h"

//Drone Obstacle Distance Calculator ROS module
#include "droneObstacleDistanceCalculatorROSModule.h"


#include "nodes_definition.h"



using namespace std;


int main(int argc, char **argv)
{
    //Init ros
    ros::init(argc, argv, MODULE_NAME_OBSTACLE_DISTANCE_CALCULATION);
  	ros::NodeHandle n;

    //Init
    cout<<"[ROSNODE] Starting Drone Obstacle Distance Calculator..."<<endl;

    //Init module
    DroneObstacleDistanceCalculatorROSModule MyDroneObstacleDistanceCalculatorROSModule;
    MyDroneObstacleDistanceCalculatorROSModule.open(n);

    //Loop
    ros::spin();

    return 1;
}
