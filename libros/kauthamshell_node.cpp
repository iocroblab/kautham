
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


    //ksh->openProblem("/home/aliakbari/stacks/devel/K_G/demos/OMPL_demos/Table_Rooms_R2/OMPL_EST_table_rooms_R2_easy.xml");


    ros::ServiceServer service1 = n.advertiseService("pose11", Query);

    chatter_pub = n.advertise<std_msgs::String>("chatter5", 1000);



    ros::spin();


    return 0;


}






