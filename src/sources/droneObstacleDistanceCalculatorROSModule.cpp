//////////////////////////////////////////////////////
//  droneObstacleDistanceCalculatorROSModule.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneObstacleDistanceCalculatorROSModule.h"



using namespace std;




void DroneObstacleDistanceCalculatorROSModule::distanciaPuntoElipse(double *p, double *hx, int dimP, int dimHx, void *adata)
{


    return;
}


void DroneObstacleDistanceCalculatorROSModule::distanciaPuntoRectangulo(double *p, double *hx, int dimP, int dimHx, void *adata)
{


    return;
}



DroneObstacleDistanceCalculatorROSModule::DroneObstacleDistanceCalculatorROSModule() : DroneModule(droneModule::active,FREQ_OBSTACLEPROCESSOR)
{
    // Init
    dronePoseMsg.time=-1;
    obstaclesMsg.time=-1;

    return;
}



DroneObstacleDistanceCalculatorROSModule::~DroneObstacleDistanceCalculatorROSModule()
{
	close();
	return;
}

bool DroneObstacleDistanceCalculatorROSModule::init()
{
    DroneModule::init();


    return true;
}


void DroneObstacleDistanceCalculatorROSModule::readParameters()
{
    // Topic names
    //
    ros::param::get("~obstacles_topic_name", obstacles_topic_name);
    if ( obstacles_topic_name.length() == 0)
    {
        obstacles_topic_name="obstacles";
    }
    std::cout<<"obstacles_topic_name="<<obstacles_topic_name<<std::endl;
    //
    ros::param::get("~drone_pose_topic_name", drone_pose_topic_name);
    if ( drone_pose_topic_name.length() == 0)
    {
        drone_pose_topic_name="ArucoSlam_EstimatedPose";
    }
    std::cout<<"drone_pose_topic_name="<<drone_pose_topic_name<<std::endl;
    //
    ros::param::get("~distance_to_obstacles_topic_name", distance_to_obstacles_topic_name);
    if ( distance_to_obstacles_topic_name.length() == 0)
    {
        distance_to_obstacles_topic_name="distanceToObstacles";
    }
    std::cout<<"distance_to_obstacles_topic_name="<<distance_to_obstacles_topic_name<<std::endl;

    return;
}


void DroneObstacleDistanceCalculatorROSModule::open(ros::NodeHandle & nIn)
{
	//Node
    DroneModule::open(nIn);


    readParameters();
    init();

    // Topics
    // Subs
    obstaclesSub = n.subscribe(obstacles_topic_name, 1, &DroneObstacleDistanceCalculatorROSModule::obstaclesCallback, this);
    dronePoseSub = n.subscribe(drone_pose_topic_name, 1, &DroneObstacleDistanceCalculatorROSModule::dronePoseCallback, this);
    // Pub
    distanceToObstaclesPub = n.advertise<droneMsgsROS::distancesToObstacles>(distance_to_obstacles_topic_name, 1, true);


    //Flag of module opened
    droneModuleOpened=true;
	
	//End
	return;
}



void DroneObstacleDistanceCalculatorROSModule::close()
{
    DroneModule::close();


	 return;
}



bool DroneObstacleDistanceCalculatorROSModule::resetValues()
{


    return true;

}



bool DroneObstacleDistanceCalculatorROSModule::startVal()
{


    //End
    return DroneModule::startVal();
}



bool DroneObstacleDistanceCalculatorROSModule::stopVal()
{

    return DroneModule::stopVal();
}



bool DroneObstacleDistanceCalculatorROSModule::run()
{
    if(!DroneModule::run())
        return false;


    if(droneModuleOpened==false)
        return false;


    // Check Pose
    if(dronePoseMsg.time<0)
        return false;

    // Check obstacles
    if(obstaclesMsg.time<0)
        return false;

    if(obstaclesMsg.poles.size()==0 && obstaclesMsg.walls.size()==0)
        return false;


    // Prepare message to publish
    distanceToObstaclesMsg.header.stamp=ros::Time::now();
    distanceToObstaclesMsg.distances_to_obstacles.resize(0);


    // Distancia a poles
    for(unsigned int i=0; i<obstaclesMsg.poles.size(); i++)
    {
        droneMsgsROS::distanceToObstacle theDistanceToObstacle;

        // id
        theDistanceToObstacle.id_obstacle=obstaclesMsg.poles[i].id;

        // Distance
        theDistanceToObstacle.distance_to_obstacle=0.0;


        // Push
        distanceToObstaclesMsg.distances_to_obstacles.push_back(theDistanceToObstacle);
    }


    // Distancia a walls
    for(unsigned int i=0; i<obstaclesMsg.walls.size(); i++)
    {
        droneMsgsROS::distanceToObstacle theDistanceToObstacle;

        // id
        theDistanceToObstacle.id_obstacle=obstaclesMsg.walls[i].id;

        // Distance
        theDistanceToObstacle.distance_to_obstacle=0.0;


        // Push
        distanceToObstaclesMsg.distances_to_obstacles.push_back(theDistanceToObstacle);
    }


    // Publicar
    publishDistancesToObstacles();


    // End
    return false;
}


void DroneObstacleDistanceCalculatorROSModule::obstaclesCallback(const droneMsgsROS::obstaclesTwoDim::ConstPtr& msg)
{
    obstaclesMsg=(*msg);


    run();

    return;
}


void DroneObstacleDistanceCalculatorROSModule::dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg)
{
    dronePoseMsg=(*msg);


    run();

    return;
}


void DroneObstacleDistanceCalculatorROSModule::publishDistancesToObstacles()
{
    distanceToObstaclesPub.publish(distanceToObstaclesMsg);

    return;
}
