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

class kauthamshellnode {
    bool srvOpenProblem (kautham_ros::Problem::Request &req, kautham_ros::Problem::Response &res);

    bool srvOpenProblemStream (kautham_ros::Problem::Request &req, kautham_ros::Problem::Response &res);

    void srvSetQuery (kautham_ros::SetQuery::Request &req, kautham_ros::SetQuery::Response &res);
};
