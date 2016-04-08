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
#include "kautham2/ManipulationAction.h"
#include "kautham2/SetWorldState.h"

//#include"kautham2/SetManipQuery.h"
#include"kautham2/OpenManipProblem.h"
#include"kautham2/GetPath.h"
#include"kautham2/SetBodyState.h"
#include"kautham2/GetBodyState.h"
#include"kautham2/GetWorldState.h"
#include"kautham2/SetWorldState.h"
#include"kautham2/SolveManipQuery.h"
#include"kautham2/CheckCollision.h"

using namespace std;
using namespace Kautham;

//std_msgs::String my_msg;
//ros::Publisher chatter_pub;
kauthamshell* kmanip;

//Compute distance to goal
double dist(std::vector<double>laststate, std::vector<double>goal)
{
    double distGoal;
    distGoal = sqrt((pow((laststate[0] - goal[0]), 2) + (pow((laststate[1] - goal[1]), 2))));
    return distGoal;
}

//Creat a potential goal for push/pull
std::vector<double> potGoal(std::vector<double>objPos,std::vector<double>robPos, std::string action, std::string know)
{
    std::vector<double>goalPos;
    goalPos.resize(2);

    std::vector<double>goalPos1;
    goalPos1.resize(2);

    std::vector<double>goalPos2;
    goalPos2.resize(2);

    if(know == "x")
    {
        double posx = objPos[0] + 20; // 20 will be input for all.
        double negx = objPos[0] - 20;

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


    if(know == "y")
    {
        double posy = objPos[0] + 20; // 20 will be input for all.
        double negy = objPos[0] - 20;

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
    ros::ServiceClient getWorldState_client = n.serviceClient<kautham2::GetWorldState>("manipulation_node/GetWorldState");
    kautham2::GetWorldState getWorldState_srv;
    getWorldState_client.call(getWorldState_srv);
    ROS_DEBUG("Get world state service performed sucessfully !");

}

void setWorldState()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/SetWorldState");
    ros::ServiceClient setWorldState_client = n.serviceClient<kautham2::SetWorldState>("manipulation_node/SetWorldState");
    kautham2::SetWorldState setWorldState_srv;
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
    ROS_DEBUG("Set world state service performed sucessfully !");
}
void setBodyState()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/SetBodyState");
    ros::ServiceClient SetBodyState_client = n.serviceClient<kautham2::SetBodyState>("manipulation_node/SetBodyState");
    kautham2::SetBodyState SetBodyState_srv;
    SetBodyState_srv.request.taretBody=4;
    //dummy value to checek the functionality of the service
    SetBodyState_srv.request.Pose.resize(7);
    SetBodyState_srv.request.Pose[0]=-105;
    SetBodyState_srv.request.Pose[1]=145;
    SetBodyState_srv.request.Pose[2]=15;
    SetBodyState_srv.request.Pose[3]=0.7;
    SetBodyState_srv.request.Pose[4]=0;
    SetBodyState_srv.request.Pose[5]=0;
    SetBodyState_srv.request.Pose[6]=0.7;

    SetBodyState_client.call(SetBodyState_srv);
    ROS_DEBUG("Set Body state service performed sucessfully !");
}

void getBodyState()
{
    ros::NodeHandle n;
    ros::service::waitForService("manipulation_node/GetBodyState");
    ros::ServiceClient getBodyState_client = n.serviceClient<kautham2::GetBodyState>("manipulation_node/GetBodyState");
    kautham2::GetBodyState getBodyState_srv;
    getBodyState_srv.request.taretBody=4;
    getBodyState_client.call(getBodyState_srv);

    std::cout<<"Body position is : "<<getBodyState_srv.response.targetBodyPose[0]<<" "<<getBodyState_srv.response.targetBodyPose[1]<<std::endl;

    ROS_DEBUG("Get Body state service performed sucessfully !");
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

void setManipQuery()
{
    ros::NodeHandle node;
    ros::service::waitForService("manipulation_node/OpenManipProblem");
    ros::ServiceClient open_problem_client = node.serviceClient<kautham2::OpenManipProblem>("manipulation_node/OpenManipProblem");
    kautham2::OpenManipProblem open_problem_srv;
    std::string model = "/home/muhayyuddin/catkin_ws/src/kautham/demos/models";
    open_problem_srv.request.problem = "/home/users/aliakbar.akbari/catkin_ws/src/kautham/demos/OMPL_demos/KauthamOpenDE/ETFA2016.xml";
    //open_problem_srv.request.problem = "/home/muhayyuddin/catkin_ws/src/kautham/demos/OMPL_demos/KauthamOpenDE/ETFA2016.xml";
    open_problem_srv.request.dir.resize(1);
    open_problem_srv.request.dir[0] = model;
    open_problem_client.call(open_problem_srv);
    // activate the setquery service

    //Set the new object pose
//    getBodyState();
//    setBodyState();
//    getBodyState();

    ros::service::waitForService("manipulation_node/SolveManipQuery");
    ros::ServiceClient solve_manip_query_client = node.serviceClient<kautham2::SolveManipQuery>("manipulation_node/SolveManipQuery");
    kautham2::SolveManipQuery solve_manip_query_srv;
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
    goal[0]=82;
    goal[1]=67;

    initC = contConversion(init);
    goalC = contConversion(goal);

    std::cout << "initC " << initC[0] << " " << initC[1] << std::endl;
    std::cout << "goalC " << goalC[0] << " " << goalC[1] << std::endl;


    // Set the initial and goal vectors (kautham controls)
    solve_manip_query_srv.request.init[0] = initC[0];
    solve_manip_query_srv.request.init[1] = initC[1];
    //solve_manip_query_srv.request.init[2] = initC[2];
    solve_manip_query_srv.request.goal[0] = goalC[0];
    solve_manip_query_srv.request.goal[1] = goalC[1];
    //solve_manip_query_srv.request.goal[2] = goalC[2];

    solve_manip_query_srv.request.actionType="move";

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

    if(dis < 1.1)
    {
        std::cout<<"Motion query has been solved"<<std::endl;
    }
    else
    {
        std::cout<<"Motion query has not been solved"<<std::endl;
    }


    ROS_INFO("Manipulation Query performed sucessfully !");


    //    //Set check collision and return the index of object collided if any.
    //    ros::service::waitForService("kautham_node/CheckCollision");
    //    ros::ServiceClient check_collision_client = node.serviceClient<kautham2::CheckCollision>("kautham_node/CheckCollision");
    //    kautham2::CheckCollision check_collision_srv;


}

int main (int argc, char **argv)
{


    ros::init(argc, argv, "manipulation_client");
    ROS_INFO("Starting Manipulation_Client");

    setManipQuery();

   std::vector<double>objPos;
   objPos.resize(2);
   objPos[0]=5;
   objPos[1]=2;

   std::vector<double>robPos;
   robPos.resize(2);
   robPos[0]=4;
   robPos[1]=2;

   std::string action;
   action = "pull";

   std::string know;
   know = "x";

   potGoal(objPos, robPos, action, know);

    //    getBodyState();
    //    setBodyState();
    //    getWorldState();
    //    setWorldState();
    //    getWorldState();
    ros::spin();

    return 0;
}







