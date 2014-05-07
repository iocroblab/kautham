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
#include "kauthamshell.h"
#include "libproblem/problem.h"
#include "libsplanner/libioc/kthquery.h"
#include "libsplanner/libioc/iocplanner.h"
#include <Inventor/SoDB.h>
#include "ros/ros.h"
#include "task_planning/Location1.h"
#include "std_msgs/String.h"



using namespace std;
using namespace Kautham;

ros::Publisher chatter_pub;
std_msgs::String my_msg;

bool Query(task_planning::Location1::Request &req,
           task_planning::Location1::Response &res)
 {
    SoDB::init();
    kauthamshell* ksh = new kauthamshell();

    ROS_INFO("Opening problem:: %s", req.name.c_str());
    ksh->openProblem(req.name);

    if (ksh->solve(std::cout))
    {
        ROS_INFO("The problem file has been solved successfully.\n");
        my_msg.data = "The problem file has been solved successfully.";

    }
    else
    {
        ROS_INFO("The problem file has not been solved successfully.\n");
        my_msg.data = "The problem file has been solved successfully.";
    }

        chatter_pub.publish(my_msg);

     return true;

 }


int main (int argc, char **argv)
{
    ros::init(argc, argv, "KauthamService");
    ros::NodeHandle n;


    ros::ServiceServer service1 = n.advertiseService("pose11", Query);

    chatter_pub = n.advertise<std_msgs::String>("chatter5", 1000);



    ros::spin();


    return 0;


}






