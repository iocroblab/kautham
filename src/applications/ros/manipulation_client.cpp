/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Muhayyuddin, Aliakbar Akbari*/

#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <Inventor/SoDB.h>
#include "std_msgs/String.h"
#include "../console/kauthamshell.h"
#include "kautham/ManipulationAction.h"
#include "kautham/SetWorldState.h"

//#include"kautham/SetManipQuery.h"
#include"kautham/OpenManipProblem.h"
#include"kautham/GetPath.h"
#include"kautham/SetBodyState.h"
#include"kautham/GetBodyState.h"
#include"kautham/GetWorldState.h"
#include"kautham/SetWorldState.h"
#include"kautham/SolveManipQuery.h"
#include"kautham/CheckCollision.h"
#include"kautham/CheckCollisionObs.h"
#include"kautham/OpenProblem.h"
#include"kautham/SetObstacle.h"

using namespace std;
using namespace Kautham;


kauthamshell* kmanip;

//Compute distance to goal
double dist(std::vector<double>laststate, std::vector<double>goal)
{
    double distGoal;
    distGoal = sqrt((pow((laststate[0] - goal[0]), 2) + (pow((laststate[1] - goal[1]), 2))));
    return distGoal;
}

//Creat a potential goal for push/pull
std::vector<double> potGoal(std::vector<double>objPos,std::vector<double>robPos, std::string action, std::string know, double i)
{
    std::vector<double>goalPos;
    goalPos.resize(2);

    std::vector<double>goalPos1;
    goalPos1.resize(2);

    std::vector<double>goalPos2;
    goalPos2.resize(2);

    //Movement is free along x axis.
    if(know == "x")
    {
        double posx = objPos[0] + i;
        double negx = objPos[0] - i;

        goalPos1[0]=posx;
        goalPos1[1]=objPos[1];

        goalPos2[0]=negx;
        goalPos2[1]=objPos[1];

        double d1 = dist(goalPos1, robPos);
        double d2 = dist(goalPos2, robPos);

        if(d1<d2 && action=="pull")
        {
            goalPos=goalPos1;
        }

        if(d1>d2 && action=="pull")
        {
            goalPos=goalPos2;
        }

        if(d1<d2 && action=="push")
        {
            goalPos=goalPos2;
        }

        if(d1>d2 && action=="push")
        {
            goalPos=goalPos1;
        }
    }

    //Movement is free along y axis.
    if(know == "y")
    {
        double posy = objPos[1] + i; // 20 will be input for all.
        double negy = objPos[1] - i;

        goalPos1[0]=objPos[0];
        goalPos1[1]=posy;

        goalPos2[0]=objPos[0];
        goalPos2[1]=negy;

        double d1 = dist(goalPos1, robPos);
        double d2 = dist(goalPos2, robPos);

        if(d1<d2 && action=="pull")
        {
            goalPos=goalPos1;
        }

        if(d1>d2 && action=="pull")
        {
            goalPos=goalPos2;
        }

        if(d1<d2 && action=="push")
        {
            goalPos=goalPos2;
        }

        if(d1>d2 && action=="push")
        {
            goalPos=goalPos1;
        }
    }


    return goalPos;

}

void getWorldState()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/GetWorldState");
    ros::ServiceClient getWorldState_client = n.serviceClient<kautham::GetWorldState>("manipulation_node/GetWorldState");
    kautham::GetWorldState getWorldState_srv;
    getWorldState_client.call(getWorldState_srv);
    ROS_INFO("Get world state service performed sucessfully !");
    for (int i=0; i<getWorldState_srv.response.ObjectPose.size();i++)
    {
                for(int j=0; j<7;j++)
        {
            std::cout<<getWorldState_srv.response.ObjectPose.at(i).v[j]<<" ";
        }
                std::cout<<std::endl;
    }
}

void setObstacle()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/SetObs");
    ros::ServiceClient SetObs_client = n.serviceClient<kautham::SetObstacle>("manipulation_node/SetObs");
    kautham::SetObstacle SetObs_srv;

    SetObs_srv.request.targetObs=0;
    SetObs_srv.request.config.resize(3);
    SetObs_srv.request.config[0]=30.0;
    SetObs_srv.request.config[1]=30.0;
    SetObs_srv.request.config[2]=15.0;

    SetObs_client.call(SetObs_srv);
    ROS_INFO("SetObs service performed sucessfully!");
}

void setWorldState()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/SetWorldState");
    ros::ServiceClient setWorldState_client = n.serviceClient<kautham::SetWorldState>("manipulation_node/SetWorldState");
    kautham::SetWorldState setWorldState_srv;
    //dummy value to checek the functionality of the service
    setWorldState_srv.request.ObjectPose.resize(7);
    for(int i=0;i<7;i++)
    {
        setWorldState_srv.request.ObjectPose.at(i).v.resize(7);
        setWorldState_srv.request.ObjectPose.at(i).v[0]=20.0;
        setWorldState_srv.request.ObjectPose.at(i).v[1]=20.0;
        setWorldState_srv.request.ObjectPose.at(i).v[2]=20.0;
        setWorldState_srv.request.ObjectPose.at(i).v[3]=1;
        setWorldState_srv.request.ObjectPose.at(i).v[4]=0;
        setWorldState_srv.request.ObjectPose.at(i).v[5]=0;
        setWorldState_srv.request.ObjectPose.at(i).v[6]=0;
    }
    setWorldState_client.call(setWorldState_srv);
    ROS_INFO("Set world state service performed sucessfully !");
}

bool openProblem()
{
    ros::NodeHandle node;
    ros::service::waitForService("manipulation_node/OpenManipProblem");
    ros::ServiceClient open_problem_client = node.serviceClient<kautham::OpenManipProblem>("manipulation_node/OpenManipProblem");
    kautham::OpenManipProblem open_problem_srv;
    std::string model = "/home/users/aliakbar.akbari/catkin_ws/src/kautham/demos/models/obstacles/";
    open_problem_srv.request.problem = model+"../../OMPL_demos/KauthamOpenDE/STMP.xml";
    open_problem_srv.request.dir.resize(1);
    open_problem_srv.request.dir[0] = model+"../";
    open_problem_client.call(open_problem_srv);

    if (open_problem_srv.response.status == true) {
        std::cout << "Kautham Problem opened correctly" << std::endl;
        return true;
    } else {
        std::cout << "ERROR Opening Kautham Problem" << std::endl;
        return false;
    }
}

void setBodyState(std::vector<double> bodyPos, int bodyIndex)
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/SetBodyState");
    ros::ServiceClient SetBodyState_client = n.serviceClient<kautham::SetBodyState>("manipulation_node/SetBodyState");
    kautham::SetBodyState SetBodyState_srv;

    SetBodyState_srv.request.targetBody=bodyIndex;
    SetBodyState_srv.request.Pose.resize(7);
//    SetBodyState_srv.request.Pose[0]=-84;
//    SetBodyState_srv.request.Pose[1]=145;
    SetBodyState_srv.request.Pose[0]=bodyPos[0];
    SetBodyState_srv.request.Pose[1]=bodyPos[1];
    SetBodyState_srv.request.Pose[2]=15;
    SetBodyState_srv.request.Pose[3]=0.707107 ;
    SetBodyState_srv.request.Pose[4]=0;
    SetBodyState_srv.request.Pose[5]=0;
    SetBodyState_srv.request.Pose[6]=0.707107 ;

    SetBodyState_client.call(SetBodyState_srv);
    ROS_INFO("Set Body state service performed sucessfully !");
}

std::vector<double> getBodyState(int bodyIndex)
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/GetBodyState");
    ros::ServiceClient getBodyState_client = n.serviceClient<kautham::GetBodyState>("manipulation_node/GetBodyState");
    kautham::GetBodyState getBodyState_srv;
    getBodyState_srv.request.targetBody=bodyIndex;
    getBodyState_client.call(getBodyState_srv);

    std::cout<<"Body position is : "<<getBodyState_srv.response.targetBodyPose[0]<<" "<<getBodyState_srv.response.targetBodyPose[1]<<std::endl;
    std::vector<double> bodyPos;
    bodyPos=getBodyState_srv.response.targetBodyPose;
    ROS_INFO("Get Body state service performed sucessfully !");
    return bodyPos;
}


std::vector<double>contConversion(std::vector<double> pose)
{
    double minX, maxX, minY, maxY;
    minX = -285.0;
    maxX = 285.0;
    minY = -153.0;
    maxY = 153.0;


    std::vector<double>cont;
    cont.resize(2);
    cont[0] = (pose[0] - minX)/(maxX - minX);
    cont[1] = (pose[1] - minY)/(maxY - minY);
    // cont[2] = 0.5;
    return cont;

}

bool collisionCheck(std::vector<double>goalPos)
{
    ros::NodeHandle node;

    ros::service::waitForService("manipulation_node/OpenManipProblem");
    ros::ServiceClient open_problem_client = node.serviceClient<kautham::OpenManipProblem>("manipulation_node/OpenManipProblem");
    kautham::OpenManipProblem open_problem_srv;
    std::string model = "/home/muhayyuddin/catkin_ws/src/kautham/demos/models";
    //open_problem_srv.request.problem = "/home/users/aliakbar.akbari/catkin_ws/src/kautham/demos/OMPL_demos/KauthamOpenDE/ETFA2016.xml";
    open_problem_srv.request.problem = "/home/muhayyuddin/catkin_ws/src/kautham/demos/OMPL_demos/KauthamOpenDE/k-PMPCar.xml";
    open_problem_srv.request.dir.resize(1);
    open_problem_srv.request.dir[0] = model;
    open_problem_client.call(open_problem_srv);
    // activate the setquery service

    ros::service::waitForService("manipulation_node/CheckCollision");
    ros::ServiceClient check_collision_client = node.serviceClient<kautham::CheckCollision>("manipulation_node/CheckCollision");
    kautham::CheckCollision check_collision_srv;

    std::vector<double> conf;
    conf = contConversion(goalPos);

    check_collision_srv.request.config.resize(2);
    check_collision_srv.request.config[0] = goalPos[0];
    check_collision_srv.request.config[1] = goalPos[1];
  //  check_collision_srv.request.config[2] = goalPos[2];


    //    check_collision_srv.request.config[0] = 0.584;
    //    check_collision_srv.request.config[1] = 0.591;

    check_collision_client.call(check_collision_srv);

    if(check_collision_srv.response.collisionFree)
    {
        std::cout<<"free"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"collision"<<std::endl;
        return false;
    }

}

bool collisionCheckObs(std::vector<double>goalPos, int targetObs)
{
    ros::NodeHandle node;
    ros::service::waitForService("manipulation_node/CheckCollisionObs");
    ros::ServiceClient check_collision_obs_client = node.serviceClient<kautham::CheckCollisionObs>("manipulation_node/CheckCollisionObs");
    kautham::CheckCollisionObs check_collision_obs_srv;


    check_collision_obs_srv.request.config.resize(3);
    check_collision_obs_srv.request.config[0] = goalPos[0];
    check_collision_obs_srv.request.config[1] = goalPos[1];
    check_collision_obs_srv.request.config[2] = goalPos[2];
    check_collision_obs_srv.request.targetObs = targetObs;

    //    check_collision_srv.request.config[0] = 0.584;
    //    check_collision_srv.request.config[1] = 0.591;

    check_collision_obs_client.call(check_collision_obs_srv);

    if(check_collision_obs_srv.response.collisionFree)
    {
        std::cout<<"free"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"collision"<<std::endl;
        return false;
    }

}

bool setManipQuery()
{
    ros::NodeHandle node;

    ros::service::waitForService("manipulation_node/SolveManipQuery");
    ros::ServiceClient solve_manip_query_client = node.serviceClient<kautham::SolveManipQuery>("manipulation_node/SolveManipQuery");
    kautham::SolveManipQuery solve_manip_query_srv;
    solve_manip_query_srv.request.init.resize(2);
    solve_manip_query_srv.request.goal.resize(2);


    std::vector<double>init;
    std::vector<double>initC;

    std::vector<double>goal;
    std::vector<double>goalC;

    init.resize(2);
    init[0]=82;
    init[1]=-25;

    goal.resize(2);
    goal[0]=-58.0;
    goal[1]=83.0;

//        goal[0]=82;
//        goal[1]=67;

    initC = contConversion(init);
    goalC = contConversion(goal);

    std::cout << "initC " << initC[0] << " " << initC[1] << std::endl;
    std::cout << "goalC " << goalC[0] << " " << goalC[1] << std::endl;


    // Set the initial and goal vectors (kautham controls)
//    solve_manip_query_srv.request.init[0] = initC[0];
//    solve_manip_query_srv.request.init[1] = initC[1];
//    //solve_manip_query_srv.request.init[2] = initC[2];
//    solve_manip_query_srv.request.goal[0] = goalC[0];
//    solve_manip_query_srv.request.goal[1] = goalC[1];
    //solve_manip_query_srv.request.goal[2] = goalC[2];


    //    solve_manip_query_srv.request.init[0] = 0.64386;
    //    solve_manip_query_srv.request.init[1] = 0.418301;
    //    //solve_manip_query_srv.request.init[2] = initC[2];
    //    solve_manip_query_srv.request.goal[0] = 0.64386;
    //    solve_manip_query_srv.request.goal[1] = 0.718954;
    //    //solve_manip_query_srv.request.goal[2] = goalC[2];
    //    solve_manip_query_srv.request.actionType="move";

    //    // Just for push and pull.
    //    solve_manip_query_srv.request.force.resize(3);
    //    solve_manip_query_srv.request.force[0] = 0.0;
    //    solve_manip_query_srv.request.force[1] = -5.0;
    //    solve_manip_query_srv.request.force[2] = 0.0;
    //    solve_manip_query_srv.request.targetBody=5;


    //call kautham service to set the query
    solve_manip_query_client.call(solve_manip_query_srv);
    std::cout <<"Query solved: "<< std::boolalpha<< solve_manip_query_srv.response.status<<std::endl;
    std::cout<<"Power Consumed is " <<solve_manip_query_srv.response.powerconsumed<<std::endl;
    std::cout<<"Last State is "<<solve_manip_query_srv.response.laststate[0]<<" "<<solve_manip_query_srv.response.laststate[1]<<std::endl;

    std::vector<double>lstate;
    lstate = solve_manip_query_srv.response.laststate;

    double dis;
    dis = dist(lstate,goal);
    std::cout<<"Distance is "<<dis<<std::endl;

    ROS_INFO("Manipulation Query performed sucessfully !");

    if(dis < 3.5)
    {
        std::cout<<"Motion query has been solved"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"Motion query has not been solved"<<std::endl;
        return false;
    }

}

int main (int argc, char **argv)
{

    ros::init(argc, argv, "manipulation_client");
    ROS_INFO("Starting Manipulation_Client");



    std::vector<double>goalPos;

    std::vector<double>bodyPos;
    bodyPos.resize(3);
    bodyPos[0]=-117.420;
    bodyPos[1]=66.0;
    bodyPos[2]=15.0;

    std::vector<double>bodyPos2;
    bodyPos2.resize(3);

    bodyPos2[0]=235.0;
    bodyPos2[1]=83.0;
    bodyPos2[2]=15.0;


    std::vector<double>robPos;
    robPos.resize(2);
    robPos[0]=82.0;
    robPos[1]=-25.0;

    std::string action;
    action = "push";

    std::string know;
    know = "x";

    setManipQuery();

double d;
d = 50.0;
int bodyIndex;
bodyIndex=4;


openProblem();
setObstacle();


   //goalPos = potGoal(objPos, robPos, action, know);

   //std::cout<<" Goal Pose is "<<goalPos[0]<<" "<<goalPos[1]<<std::endl;


//while (true)
//{
//    openProblem();
//    bodyPos = getBodyState(bodyIndex);
//    // Set goal position for the push/pull action
//    goalPos = potGoal(bodyPos, robPos, action, know, d);
//    std::cout<<" Goal Object Pose is "<<goalPos[0]<<" "<<goalPos[1]<<std::endl;
//    if(!collisionCheck(goalPos))
//    {
//        std::cout<<" Action failed "<<std::endl;
//        break;
//    }
// //   getWorldState();
//    setBodyState(goalPos, bodyIndex);
// //   getWorldState();
//    if(setManipQuery())
//    {
//        std::cout<<" Query has been solved "<<std::endl;
//        break;
//    }
//    d += 10;
//}

    //    getBodyState();
    //    setBodyState();
    //    getWorldState();
    //    setWorldState();
    //    getWorldState();
    ros::spin();

    return 0;
}






