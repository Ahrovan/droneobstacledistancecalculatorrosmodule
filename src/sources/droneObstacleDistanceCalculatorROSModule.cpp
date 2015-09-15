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
    // External parameters
    DistancePointEllipseProblem *TheDistancePointEllipseProblem=(DistancePointEllipseProblem *)adata;
    // Point of the drone
    double xp=TheDistancePointEllipseProblem->xp;
    double yp=TheDistancePointEllipseProblem->yp;
    // Size of the obstacles
    double a=TheDistancePointEllipseProblem->a;
    double b=TheDistancePointEllipseProblem->b;


    // Solution or guess
    double x=p[0];
    double y=p[1];


    // Equations
    hx[0]=x*(y-yp)-pow(a,2)/pow(b,2)*y*(x-xp);
    hx[1]=pow(x,2)/pow(a,2)+pow(y,2)/pow(b,2)-1.0;


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

        // Obstacle in world
        double xe=obstaclesMsg.poles[i].centerX;
        double ye=obstaclesMsg.poles[i].centerY;
        double alphae=obstaclesMsg.poles[i].yawAngle;
        double a=obstaclesMsg.poles[i].radiusX;
        double b=obstaclesMsg.poles[i].radiusY;
        // Drone in world
        double xd=dronePoseMsg.x;
        double yd=dronePoseMsg.y;
        // Drone in ellipse
        double x, y;
        x=xd*cos(alphae)+yd*sin(alphae)-(xe*cos(alphae)+ye*sin(alphae));
        y=-xd*sin(alphae)+yd*cos(alphae)+(xe*sin(alphae)-ye*cos(alphae));


        //
        DistancePointEllipseProblem TheDistancePointEllipseProblem;
        TheDistancePointEllipseProblem.a=a;
        TheDistancePointEllipseProblem.b=b;
        TheDistancePointEllipseProblem.xp=x;
        TheDistancePointEllipseProblem.yp=y;


        //Resolvemos
        double solution[2];
        solution[0]=x;
        solution[1]=y;

        //Options
        double opts[5];
        //tau
        opts[0]=1e-5;
        //epsilon-> 1-3
        opts[1]=1e-15;
        opts[2]=1e-15;
        opts[3]=1e-15;
        //delta
        opts[4]=1e-1;

        //Info
        double info[LM_INFO_SZ];

        //Calculation
       int iters = dlevmar_dif(&distanciaPuntoElipse, solution, 0, 2, 2, 10000, 0, info, 0, 0, &TheDistancePointEllipseProblem);
       if (iters == -1)
       {
           std::cout<<"unable to calculate solution"<<std::endl;
           return false;
       }



        // Distance
        double distance=sqrt(pow(x-solution[0],2)+pow(y-solution[1],2));

        double sign=pow(solution[0]/a,2)+pow(solution[1]/b,2)-1;
        double tol=1e-3;
        if(sign>=-tol)
            sign=1;
        else
            sign=-1;


        theDistanceToObstacle.distance_to_obstacle=sign*distance;


        // Push
        distanceToObstaclesMsg.distances_to_obstacles.push_back(theDistanceToObstacle);
    }


    // TODO JL: Distancia a walls
    for(unsigned int i=0; i<obstaclesMsg.walls.size(); i++)
    {
        droneMsgsROS::distanceToObstacle theDistanceToObstacle;

        // id
        theDistanceToObstacle.id_obstacle=obstaclesMsg.walls[i].id;

        // Distance
        theDistanceToObstacle.distance_to_obstacle=100000.0;


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
