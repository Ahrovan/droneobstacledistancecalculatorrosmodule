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



double DroneObstacleDistanceCalculatorROSModule::distanciaPuntoEllipse(droneMsgsROS::dronePose drone_pose, droneMsgsROS::obstacleTwoDimPole obstacle)
{


    // Obstacle in world
    double xe=obstacle.centerX;
    double ye=obstacle.centerY;
    double alphae=obstacle.yawAngle;
    double a=obstacle.radiusX;
    double b=obstacle.radiusY;
    // Drone in world
    double xd=drone_pose.x;
    double yd=drone_pose.y;
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
    // Punto inicial para iteracion
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
   int iters = dlevmar_dif(&distanciaPuntoElipseEquations, solution, 0, 2, 2, 10000, 0, info, 0, 0, &TheDistancePointEllipseProblem);
   if (iters == -1)
   {
       std::cout<<"unable to calculate solution"<<std::endl;
       return false;
   }



    // Distance
    double distance=sqrt(pow(x-solution[0],2)+pow(y-solution[1],2));

    // Sign
    double sign=pow(x/a,2)+pow(y/b,2)-1;
    if(sign>=0)
        sign=1;
    else
        sign=-1;


    double distanceOut=sign*distance;

    return distanceOut;
}

void DroneObstacleDistanceCalculatorROSModule::distanciaPuntoElipseEquations(double *p, double *hx, int dimP, int dimHx, void *adata)
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


double DroneObstacleDistanceCalculatorROSModule::distanciaPuntoRectangulo(droneMsgsROS::dronePose drone_pose, droneMsgsROS::obstacleTwoDimWall obstacle)
{

//    double cx_p = obstacle.centerX*cos(obstacle.yawAngle) + obstacle.centerY*sin(obstacle.yawAngle);
//    double cy_p = -obstacle.centerX*sin(obstacle.yawAngle) + obstacle.centerY*cos(obstacle.yawAngle);

    double alpha = obstacle.yawAngle;
    double obs_x = obstacle.centerX;
    double obs_y = obstacle.centerY;
    double dp_x = drone_pose.x;
    double dp_y = drone_pose.y;
    double Sx_2 = obstacle.sizeX/2.0;
    double Sy_2 = obstacle.sizeY/2.0;

    double drone_posex_p = dp_x*cos(alpha) + dp_y*sin(alpha)-obs_x*cos(alpha)-obs_y*sin(alpha);
    double drone_posey_p = -dp_x*sin(alpha) + dp_y*cos(alpha)+obs_x*sin(alpha)-obs_y*cos(alpha);

    bool pose_inside_obstacle = false;
    if(drone_posex_p > -Sx_2 && drone_posex_p < Sx_2)
    {
        if(drone_posey_p > -Sy_2 && drone_posey_p < Sy_2)
            pose_inside_obstacle = true;
    }


    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> p3;
    std::vector<double> p4;


    p1.push_back(-Sx_2*cos(alpha) - Sy_2*sin(alpha) + obs_x);
    p1.push_back(-Sx_2*sin(alpha) + Sy_2*cos(alpha) + obs_y);

    p2.push_back(Sx_2*cos(alpha) - Sy_2*sin(alpha) + obs_x);
    p2.push_back(Sx_2*sin(alpha) + Sy_2*cos(alpha) + obs_y);

    p3.push_back(Sx_2*cos(alpha) + Sy_2*sin(alpha) + obs_x);
    p3.push_back(Sx_2*sin(alpha) - Sy_2*cos(alpha) + obs_y);

    p4.push_back(-Sx_2*cos(alpha) + Sy_2*sin(alpha) + obs_x);
    p4.push_back(-Sx_2*sin(alpha) - Sy_2*cos(alpha) + obs_y);

    std::vector<double> punto;
    std::vector<std::vector<double> > segmento1, segmento2, segmento3, segmento4;


    punto.push_back(drone_pose.x);
    punto.push_back(drone_pose.y);

    segmento1.push_back(p1);
    segmento1.push_back(p2);

    segmento2.push_back(p2);
    segmento2.push_back(p3);

    segmento3.push_back(p3);
    segmento3.push_back(p4);

    segmento4.push_back(p4);
    segmento4.push_back(p1);

    double d1 = distanciaPuntoSegmento(punto,segmento1);
    double d2 = distanciaPuntoSegmento(punto,segmento2);
    double d3 = distanciaPuntoSegmento(punto,segmento3);
    double d4 = distanciaPuntoSegmento(punto,segmento4);


    std::vector<double> distances;
    distances.push_back(d1);
    distances.push_back(d2);
    distances.push_back(d3);
    distances.push_back(d4);

    droneMsgsROS::segmento seg;
    seg.id=obstacle.id;
    droneMsgsROS::seg so;
    so.p=p1;
    seg.seg1.push_back(so);
    droneMsgsROS::seg so2;
    so2.p=p2;
    seg.seg1.push_back(so2);
    so2.p=p2;
    seg.seg2.push_back(so2);
    droneMsgsROS::seg so3;
    so3.p=p3;
    seg.seg2.push_back(so3);
    so3.p=p3;
    seg.seg3.push_back(so3);
    droneMsgsROS::seg so4;
    so4.p=p4;
    seg.seg3.push_back(so4);
    so4.p=p4;
    seg.seg4.push_back(so4);
    so.p=p1;
    seg.seg4.push_back(so);
    segmentoPub.publish(seg);

    double min_distance = distances[0];
    for(int i=1;i<distances.size();i++)
    {
        if(distances[i] < min_distance)
            min_distance = distances[i];
    }

    if(pose_inside_obstacle)
        min_distance = -min_distance;
    return min_distance;
}

double DroneObstacleDistanceCalculatorROSModule::distanciaPuntoSegmento(std::vector<double> &punto, std::vector<std::vector<double> > &segmento)
{
    std::vector<double> limite1, limite2;
    limite1 = segmento[0];
    limite2 = segmento[1];


    double u[2];
    u[0] = (punto[0]-limite1[0]);
    u[1] = (punto[1]-limite1[1]);

    double v[2];
    v[0] = (limite2[0]-limite1[0]);
    v[1] = (limite2[1]-limite1[1]);

    double mod_v = sqrt(pow(v[0],2) + pow(v[1],2));
    if(mod_v > 0.00001)
    {
        v[0] = v[0]/mod_v;
        v[1] = v[1]/mod_v;
    }

    double dir[2];

    double valor = (u[0]*v[0] + u[1]*v[1]);
    double distancia = 0;

    if(valor<0)
    {
        dir[0] = u[0];
        dir[1] = u[1];
    }
    else if(valor > mod_v)
    {
        dir[0] = (punto[0]-limite2[0]);
        dir[1] = (punto[1]-limite2[1]);
    }
    else
    {
        dir[0] = u[0]- v[0]*valor;
        dir[1] = u[1]- v[1]*valor;
    }
    distancia = sqrt(pow(dir[0],2) + pow(dir[1],2));
    return distancia;
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

    ros::param::get("~segments_topic_name", segments_topic_name);
    if (segments_topic_name.length() == 0)
    {
        segments_topic_name="segments";
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
    segmentoPub= n.advertise<droneMsgsROS::segmento>(segments_topic_name,1,true);
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
    // Header
    distanceToObstaclesMsg.header.stamp=ros::Time::now();
    // List
    distanceToObstaclesMsg.distances_to_obstacles.resize(0);


    // Distancia a poles
    for(unsigned int i=0; i<obstaclesMsg.poles.size(); i++)
    {
        droneMsgsROS::distanceToObstacle theDistanceToObstacle;

        // id
        theDistanceToObstacle.id_obstacle=obstaclesMsg.poles[i].id;

        // Distance
        double distance=distanciaPuntoEllipse(dronePoseMsg,obstaclesMsg.poles[i]);
        theDistanceToObstacle.distance_to_obstacle=distance;


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
        double distance=distanciaPuntoRectangulo(dronePoseMsg,obstaclesMsg.walls[i]);
        theDistanceToObstacle.distance_to_obstacle=distance;


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
