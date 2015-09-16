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




class DistancePointEllipseProblem
{
public:
    // Point of the drone
    double yp;
    double xp;
    // Size of the obstacles
    double a;
    double b;



};



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
    // Ellipse
    double distanciaPuntoEllipse(droneMsgsROS::dronePose drone_pose, droneMsgsROS::obstacleTwoDimPole obstacle);
    static void distanciaPuntoElipseEquations(double *p, double *hx, int dimP, int dimHx, void *adata);


protected:
    // Rectangle
    double distanciaPuntoRectangulo(droneMsgsROS::dronePose drone_pose, droneMsgsROS::obstacleTwoDimWall obstacle);
    double distanciaPuntoSegmento(std::vector<double> &punto, std::vector<std::vector<double> > &segmento);



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
