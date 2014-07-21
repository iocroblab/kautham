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

/* Author: Aliakbar Akbari */


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "kauthamshell.h"
#include "problem/problem.h"
#include "planner/ioc/kthquery.h"
#include "planner/ioc/iocplanner.h"
#include <Inventor/SoDB.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kautham2/Problem.h"
#include "kautham2/ProblemStream.h"
#include "kautham2/SetQuery.h"
#include "kautham2/MoveRobot.h"



using namespace std;
using namespace Kautham;

std_msgs::String my_msg;
ros::Publisher chatter_pub;



bool srvOpenProblem(kautham2::Problem::Request &req, kautham2::Problem::Response &res)

 {
    SoDB::init();

    kauthamshell* ksh = new kauthamshell();

    ROS_INFO("Opening problem:: %s", req.problem.c_str());

    ksh->openProblem(req.problem, req.dir);


    if (ksh->solve(std::cout))
    {
        ROS_INFO("The problem file has been solved successfully.\n");
        my_msg.data = "The problem file has been solved successfully.";
        res.response = true;

    }
    else
    {
        ROS_INFO("The problem file has not been solved successfully.\n");
        my_msg.data = "The problem file has been solved successfully.";
        res.response = false;
    }
     //   chatter_pub.publish(my_msg);

     return true;

 }

bool srvOpenProblemStream(kautham2::ProblemStream::Request &req, kautham2::ProblemStream::Response &res)
 {
    SoDB::init();

    kauthamshell* ksh = new kauthamshell();

    string dir = req.problem;
    dir.erase(dir.find_last_of("/") + 1, dir.length());
    string absPath = dir;


    //directory containing the models
    vector <string> def_path;

    def_path.push_back(dir);
    def_path.push_back(dir+"/../../models/");
    dir = absPath.substr(0,absPath.find_last_of("/")+1);
    def_path.push_back(dir);
    def_path.push_back(dir+"/../../models/");

    ifstream input;
    input.open(req.problem.c_str());

    ksh->openProblem(&input, def_path);

    if (ksh->solve(std::cout))
    {
        ROS_INFO("The problem file has been solved successfully.\n");
        res.response = true;
    }

    else
    {
        ROS_INFO("The problem file has not been solved successfully.\n");
        res.response = false;
    }

     return true;
 }

bool srvSetQuery(kautham2::SetQuery::Request &req, kautham2::SetQuery::Response &res)
{
    Problem *_problem;
    int d = _problem->wSpace()->getNumRobControls();

    kauthamshell* ksh = new kauthamshell();

    vector < KthReal > Init;
    Init.resize(d);

    for (unsigned i = 0; i < d; i++)
        // Init sample from service
            cout << "Initial sample: ";
            for (unsigned i = 0; i < d; i++)
            {
                Init[i] = req.init.positions[i];
                cout << Init[i] << " ";
            }
            cout << endl;


        // Goal sample from service
        vector < KthReal > Goal;
        Goal.resize(d);

        cout << "Goal sample: ";
        for (unsigned i = 0; i < d; i++)
        {
            Goal[i] = req.goal.positions[i];
            cout << Goal[i] << " ";
        }
        cout << endl;

    if(ksh->setQuery(Init, Goal))
    {
        res.response = true;
    }
    else
    {
        res.response = false;
    }
}

bool srvMoveRobot(kautham2::MoveRobot::Request &req, kautham2::MoveRobot::Response &res)
{
    Problem *_problem;
    int d = _problem->wSpace()->getNumRobControls();

    vector < KthReal > Move(d);



    for (unsigned i = 0; i < d; i++)
    {
        Move[i] = req.move.positions[i];
    }


    kauthamshell* ksh = new kauthamshell();
    ksh->setRobotsConfig(Move);

    res.response = true;

    ROS_INFO("Robot is moving");

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "KauthamService");
    ros::NodeHandle n;

    ROS_INFO("Kautham_Service");


    ros::ServiceServer service1 = n.advertiseService("OpenProblem", srvOpenProblem);

    ros::ServiceServer service2 = n.advertiseService("OpenProblemStream", srvOpenProblemStream);

    ros::ServiceServer service3 = n.advertiseService("SetQuery", srvSetQuery);

    ros::ServiceServer service4 = n.advertiseService("MoveRobot", srvMoveRobot);



    ros::spin();


    return 0;


}






