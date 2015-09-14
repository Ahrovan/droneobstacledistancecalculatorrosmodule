//////////////////////////////////////////////////////
//  droneObstacleDistanceCalculatorROSModule.h
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef DRONE_OBSTACLE_DISTANCE_CALCULATOR_ROS_MODULE_H
#define DRONE_OBSTACLE_DISTANCE_CALCULATOR_ROS_MODULE_H





//Math
#include <cmath>


#include <cstdlib>
#include <ctime>



#include <iostream>

#include <fstream>

#include <vector>

#include <sstream>

#include <string>



////// ROS  ///////
#include "ros/ros.h"


//Drone module
#include "droneModuleROS.h"



////Msgs
//drone pose
#include "droneMsgsROS/dronePose.h"

// Obstacles
#include "droneMsgsROS/obstaclesTwoDim.h"
#include "droneMsgsROS/obstacleTwoDimWall.h"
#include "droneMsgsROS/obstacleTwoDimPole.h"

// Distances
#include "droneMsgsROS/distancesToObstacles.h"
#include "droneMsgsROS/distanceToObstacle.h"


// LEVMAR
#include <levmar.h>



//Freq
const double FREQ_OBSTACLEPROCESSOR = 30.0;





/////////////////////////////////////////
// Class DroneObstacleProcessorROSModule
//
//   Description
//
/////////////////////////////////////////
class DroneObstacleDistanceCalculatorROSModule : public DroneModule
{	
    // Algorithm
protected:

    static void distanciaPuntoElipse(double *p, double *hx, int dimP, int dimHx, void *adata);
    static void distanciaPuntoRectangulo(double *p, double *hx, int dimP, int dimHx, void *adata);



public:
    DroneObstacleDistanceCalculatorROSModule();
    ~DroneObstacleDistanceCalculatorROSModule();

public:
    void open(ros::NodeHandle & nIn);
	void close();


protected:
    bool init();
    void readParameters();

    //Reset
protected:
    bool resetValues();

    //Start
protected:
    bool startVal();

    //Stop
protected:
    bool stopVal();

    //Run
public:
    bool run();


protected:
    std::string obstacles_topic_name;
    ros::Subscriber obstaclesSub;
    void obstaclesCallback(const droneMsgsROS::obstaclesTwoDim::ConstPtr& msg);
    droneMsgsROS::obstaclesTwoDim obstaclesMsg;


protected:
    std::string drone_pose_topic_name;
    ros::Subscriber dronePoseSub;
    void dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);
    droneMsgsROS::dronePose dronePoseMsg;


protected:
    std::string distance_to_obstacles_topic_name;
    ros::Publisher distanceToObstaclesPub;
    droneMsgsROS::distancesToObstacles distanceToObstaclesMsg;
    void publishDistancesToObstacles();


};





#endif
