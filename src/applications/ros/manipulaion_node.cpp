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


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <Inventor/SoDB.h>
#include "std_msgs/String.h"
#include "../console/kauthamshell.h"
#include "kautham2/ManipulationAction.h"
#include "kautham2/OpenManipProblem.h"
#include "kautham2/SetManipQuery.h"
#include "kautham2/SetBodyState.h"
#include "kautham2/GetBodyState.h"
#include "kautham2/SetWorldState.h"
#include "kautham2/GetWorldState.h"
#include "ode/ode.h"

using namespace std;
using namespace Kautham;

std_msgs::String manip_msg;
//ros::Publisher chatter_pub;
kauthamshell* kmanip;

bool srvOpenManipProblem(kautham2::OpenManipProblem::Request &req, kautham2::OpenManipProblem::Response &res)
{
    ROS_INFO("Opening problem:: %s", req.problem.c_str());
    string dir = req.problem;
    dir.erase(dir.find_last_of("/") + 1, dir.length());
    string absPath = dir;
    vector <string> def_path =req.dir;

    def_path.push_back(dir);
    def_path.push_back(dir+"/../../models/");
    dir = absPath.substr(0,absPath.find_last_of("/")+1);
    def_path.push_back(dir);
    def_path.push_back(dir+"/../../models/");

    if (kmanip->openProblem(req.problem, def_path)) {
        ROS_INFO("The problem file has been opened successfully.\n");
        manip_msg.data = "The problem file has been opened successfully.";
        res.status = true;
    } else {
        ROS_INFO("The problem file couldn't be opened.\n");
        manip_msg.data = "The problem file couldn't be opened.";
        res.status = false;
    }
    return true;    return true;
}
bool srvSetManipQuery(kautham2::SetManipQuery::Request &req,
                 kautham2::SetManipQuery::Response &res)
{
    res.status = kmanip->setQuery(req.init,req.goal);
    kmanip->setManipQueryPrams(req.actionType,req.targetBody,req.force);

    return true;
}


bool srvSetWorldState(kautham2::SetWorldState::Request &req,
             kautham2::SetWorldState::Response &res)
{
    std::vector<std::vector<double> > wstate;
   wstate.resize( req.ObjectPose.size());
    for(unsigned int i=0;i<wstate.size();i++)
    {
        wstate[i].resize(7);

        wstate[i][0]=req.ObjectPose[i].v[0];
        wstate[i][1]=req.ObjectPose[i].v[1];
        wstate[i][2]=req.ObjectPose[i].v[2];

        wstate[i][3]=req.ObjectPose[i].v[3];
        wstate[i][4]=req.ObjectPose[i].v[4];
        wstate[i][5]=req.ObjectPose[i].v[5];
        wstate[i][6]=req.ObjectPose[i].v[6];

    }
kmanip->setWorldState(wstate);
return true;
}
bool srvGetWorldState(kautham2::GetWorldState::Request &req,
             kautham2::GetWorldState::Response &res)
{
kmanip->getWorldState();
return true;
}

bool srvGetBodyState(kautham2::GetBodyState::Request &req,
             kautham2::GetBodyState::Response &res)
{

     std::vector<double> pose = kmanip->getBodyState(req.taretBody);
     std::cout<<"State of body " <<req.taretBody <<" is ["<< pose[0]<<" , "<< pose[1]<<" , "<< pose[2]
              <<" , "<< pose[3]<<" , "<< pose[4]<<" , "<< pose[5]<<" , "<< pose[6]<<" ]"<<std::endl;
     res.status=true;
    return true;
}

bool srvSetBodyState(kautham2::SetBodyState::Request &req,
             kautham2::SetBodyState::Response &res)
{
kmanip->setBodyState(req.taretBody,req.Pose);
res.status=true;
    return true;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle n;

    ROS_INFO("Starting Manipulation_Service");

    SoDB::init();
    kmanip = new kauthamshell();

    ros::ServiceServer service1 = n.advertiseService("manipulation_node/OpenManipProblem",srvOpenManipProblem);
    //ros::ServiceServer service2 = n.advertiseService("manipulation_node/ManipulationAction",srvManipulationAction);
    ros::ServiceServer service3 = n.advertiseService("manipulation_node/SetManipQuery",srvSetManipQuery);
    ros::ServiceServer service4 = n.advertiseService("manipulation_node/SetBodyState",srvSetBodyState);
    ros::ServiceServer service5 = n.advertiseService("manipulation_node/GetBodyState",srvGetBodyState);
    ros::ServiceServer service6 = n.advertiseService("manipulation_node/SetWorldState",srvSetWorldState);
    ros::ServiceServer service7 = n.advertiseService("manipulation_node/GetWorldState",srvGetWorldState);



    ros::spin();

    return 0;
}







