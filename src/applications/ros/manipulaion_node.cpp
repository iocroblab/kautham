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
#include "kautham/ManipulationAction.h"
#include "kautham/OpenManipProblem.h"
#include "kautham/SetBodyState.h"
#include "kautham/GetBodyState.h"
#include "kautham/SetWorldState.h"
#include "kautham/GetWorldState.h"
#include "kautham/SolveManipQuery.h"
#include "kautham/CheckCollision.h"
#include "kautham/CheckCollisionObs.h"
#include "kautham/SetObstacle.h"
#include "ode/ode.h"

using namespace std;
using namespace Kautham;

std_msgs::String manip_msg;
//ros::Publisher chatter_pub;
kauthamshell* kmanip;

bool srvOpenManipProblem(kautham::OpenManipProblem::Request &req, kautham::OpenManipProblem::Response &res)
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
bool srvSolveManipQuery(kautham::SolveManipQuery::Request &req,
                 kautham::SolveManipQuery::Response &res)
{
    //std::vector<State> worldstate;
    std::vector<std::pair<std::vector<float>, std::vector<float> > > worldstate;
    double power;
    std::vector<double> laststate;
     kmanip->setQuery(req.init,req.goal);
    res.status=kmanip->setManipPramsAndSolve(req.actionType,req.targetBody,req.force,&worldstate,&power,&laststate);
    res.powerconsumed=power;
    //res.laststate=laststate;
    std::cout<<"Computed Path states are : " <<worldstate.size()<<std::endl;;

    return true;
}


bool srvSetWorldState(kautham::SetWorldState::Request &req,
             kautham::SetWorldState::Response &res)
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
bool srvGetWorldState(kautham::GetWorldState::Request &req,
             kautham::GetWorldState::Response &res)
{
kmanip->getWorldState();
return true;
}

bool srvGetBodyState(kautham::GetBodyState::Request &req,
             kautham::GetBodyState::Response &res)
{

     std::vector<double> pose = kmanip->getBodyState(req.targetBody);
     std::cout<<"State of body " <<req.targetBody <<" is ["<< pose[0]<<" , "<< pose[1]<<" , "<< pose[2]
              <<" , "<< pose[3]<<" , "<< pose[4]<<" , "<< pose[5]<<" , "<< pose[6]<<" ]"<<std::endl;
     res.status=true;
     res.targetBodyPose=pose;
    return true;
}

bool srvSetBodyState(kautham::SetBodyState::Request &req,
             kautham::SetBodyState::Response &res)
{
kmanip->setBodyState(req.targetBody,req.Pose);
res.status=true;
    return true;
}
//bool srvSolveManipQuery(kautham::solveManipQuery::Request &req,
//                kautham::solveManipQuery::Response &res) {
//    ostringstream oss;
//    std::vector<State> worldstate;
//    if (kmanip->solveManipQuery(&worldstate)) {
////        vector < vector < KthReal > > path;
////        istringstream iss(oss.str());
////        for (string str; getline(iss,str); ) {
////            vector < KthReal > conf;
////            std::istringstream strs(str);
////            int chars_to_read = strs.str().size();
////            while (chars_to_read > 0) {
////                getline(strs,str,' ');
////                if (str.size() > 0) {
////                    conf.push_back(atof(str.c_str()));
////                }
////                chars_to_read -= str.size() + 1;
//        std::cout<<"Computed Path states are : " <<worldstate.size()<<std::endl;;

//    }
//    std::cout<<"Computed Path states are : " <<worldstate.size()<<std::endl;

////            std::cout<<"Computed Path is: ";
////            for(unsigned int i=0;i<conf.size();i++)
////            {
////                std::cout<<conf[i]<<" , ";
////            }

////            std::cout<<std::endl;
////            path.push_back(conf);

////        res.response.resize(path.size());
////        for (unsigned int i = 0; i < path.size(); ++i) {
////            res.response[i].v.resize(path.at(i).size());
////            for (unsigned int j = 0; j < path.at(i).size(); ++j) {
////                res.response[i].v[j] = path.at(i).at(j);
////            }
////        }
////    }

//    return true;
//}

bool srvCheckCollision(kautham::CheckCollision::Request &req,
                       kautham::CheckCollision::Response &res) {

    for (unsigned int i = 0; i < req.config.size(); ++i) {
        cout << req.config.at(i) << " ";
    }
    cout << endl;

    bool collisionFree;
    res.response = kmanip->checkCollision(req.config,&collisionFree);
    res.collisionFree = res.response&&collisionFree;

    return true;
}

bool srvSetObstacle(kautham::SetObstacle::Request &req,
                    kautham::SetObstacle::Response &res) {

    for (unsigned int i = 0; i < req.config.size(); ++i) {
        cout << req.config.at(i) << " ";
    }
    cout << endl;

    res.response = kmanip->setObstacle(req.config,req.targetObs);

    return true;
}

bool srvCheckCollisionObs(kautham::CheckCollisionObs::Request &req,
                          kautham::CheckCollisionObs::Response &res) {

    for (unsigned int i = 0; i < req.config.size(); ++i) {
        cout << req.config.at(i) << " ";
    }
    cout << endl;

    bool collisionFree;
    int collisionObs;
    res.response = kmanip->checkCollisionObs(req.config,req.targetObs,&collisionObs,&collisionFree);
    res.collisionFree = res.response&&collisionFree;
    res.collisionObject = collisionObs;

    std::cout<<"CollisionObject is " <<collisionObs<<std::endl;

    return true;
}
int main (int argc, char **argv) {
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle n;

    ROS_INFO("Starting Manipulation_Service");

    SoDB::init();
    kmanip = new kauthamshell();

    ros::ServiceServer service1 = n.advertiseService("manipulation_node/OpenManipProblem",srvOpenManipProblem);
    ros::ServiceServer service2 = n.advertiseService("manipulation_node/SolveManipQuery",srvSolveManipQuery);
    ros::ServiceServer service3 = n.advertiseService("manipulation_node/SetBodyState",srvSetBodyState);
    ros::ServiceServer service4 = n.advertiseService("manipulation_node/GetBodyState",srvGetBodyState);
    ros::ServiceServer service5 = n.advertiseService("manipulation_node/SetWorldState",srvSetWorldState);
    ros::ServiceServer service6 = n.advertiseService("manipulation_node/GetWorldState",srvGetWorldState);
    ros::ServiceServer service7 = n.advertiseService("manipulation_node/CheckCollision",srvCheckCollision);
    ros::ServiceServer service8 = n.advertiseService("manipulation_node/CheckCollisionObs",srvCheckCollisionObs);
    ros::ServiceServer service9 = n.advertiseService("manipulation_node/SetObs",srvSetObstacle);



    ros::spin();

    return 0;
}







