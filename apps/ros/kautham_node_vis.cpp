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

/* Author: Jan Rosell */

#include <ros/ros.h>
#include <kautham/kauthamshell.h>
#include <kautham/LoadRobots.h>
#include <kautham/LoadObstacles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace Kautham;

std::vector<std::string> rfilename;
std::vector<std::string> ofilename;

int numrobots = 0;
int numobstacles = 0;
bool robotsloaded = false;
bool receivedrobotscall = false;
bool obstaclesloaded = false;
bool receivedobstaclesscall = false;
bool gui=false;
std::string rvizfile;


//This service is to be called only once to load the rots info: filenames and pose
bool srvLoadRobots(kautham::LoadRobots::Request &req,
                   kautham::LoadRobots::Response &res)
{
    ROS_INFO( "STARTING Kautham LoadRobots");
    ROS_INFO( "********************************");
    receivedrobotscall = true; //flag to control that the service has been called
    if(!robotsloaded) //load data if the data has not yet been loaded
    {
        gui = req.gui;
        numrobots = req.robotsfiles.size();
        ROS_INFO("---numrobots = %d",numrobots);
        rfilename.resize(numrobots);
        //rtransform.resize(numrobots);
        for(int i=0; i<numrobots;i++)
        {
            ROS_INFO("---robotfilenames[%d] = %s",i,req.robotsfiles[i].c_str());
            rfilename[i] = req.robotsfiles[i];
        }
        res.response = true;
        rvizfile = req.rvizfile;
        return true;
    }
    else
    {
        ROS_WARN("The robots have already been loaded. This service is only called once");
        res.response = false;
        return false;
    }
}

//This service is to be called only once to load the rots info: filenames and pose
bool srvLoadObstacles(kautham::LoadObstacles::Request &req,
                    kautham::LoadObstacles::Response &res)
{
    ROS_INFO( "STARTING Kautham LoadObstacles");
    ROS_INFO( "********************************");
    receivedobstaclesscall = true; //flag to control that the service has been called
    if(!obstaclesloaded) //load data if the data has not yet been loaded
    {
        numobstacles = req.obstaclesfiles.size();
        ROS_INFO("---numobstacles = %d",numobstacles);
        ofilename.resize(numobstacles);
        //otransform.resize(numobstacles);
        for(int i=0; i<numobstacles;i++)
        {
            //ROS_INFO("---!!!!!!!!obstaclesfilenames[%d] = %s",i,req.obstaclesfiles[i].c_str());
            ofilename[i] = req.obstaclesfiles[i];
        }
        res.response = true;
        return true;
    }
    else
    {
        ROS_WARN("The obstacles have already been loaded. This service is only called once");
        res.response = false;
        return false;
    }
}


bool loadRobots()
{
    if(!robotsloaded && receivedrobotscall)
    {
        ROS_INFO("loadRobots!!!!!!!!");
        //Start fillilng the launch command for the viewer.launch file
        std::string viewerlaunchstr = "roslaunch kautham viewer.launch";
        //Loop for all robots
        for(int i=0; i<numrobots; i++)
        {
            //(1) Launch the load_robot_description.launch that loads the robot_descrition within the robot namespace

            //Fill the string that will allow to obtain the complete robot filename
            //std::string rname = "\"\\$(find kautham)/demos/models/"+rfilename[i]+"\"";
            std::string rname = "\"" + rfilename[i] +"\"";
            //Fill the robot namespace. Robot namespaces are: robot0, robot1, etc...
            std::stringstream robot_namespace;
            robot_namespace << "robot"<<i;
            //Fill the launch command
            std::string launchstr = "roslaunch kautham load_robot_description.launch rname:="
                    + robot_namespace.str() + " rfilename:=" + rname;
            std::cout<<launchstr<<std::endl;
            //Launch
            system(launchstr.c_str());

            //(2) Modify the robot_description to include:
            // - names of links and joints that start with the robot namespace label, i.e. "robotX_"
            // - the correct paths for the geometries. Here it is assumed that the models are in the
            //   standard kautham directory, i.e. in: "package://kautham/demos/models/robots/"

            //Load the robot description string
            std::stringstream my_robot_description;
            my_robot_description << "robot"<<i<<"/robot_description";
            std::string str;
            ros::param::get(my_robot_description.str(), str);

            //Substitute the file paths
            //First extract from the rfilename[i] the path, i.e. from the begining till the last "/" character
            std::size_t found = rfilename[i].find_last_of("/");
            std::string substr1 = "filename=\"";
            std::string substr2 = std::string("filename=\"file://")+rfilename[i].substr(0,found+1);
            //std::cout<<"fname="<<ofilename[i]<<std::endl;
            //std::cout<<"found="<<found<<std::endl;
            //std::cout<<"fpath="<<substr2<<std::endl;
            //std::string substr2 = "filename=\"file:///home/jan/catkin/catkin_wsTAMP/src/kautham/demos/models/obstacles/chess/";
            //std::string substr2 = "filename=\"package://kautham/demos/models/obstacles/chess/";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the joint names
            substr1 = "<joint name=\"";
            substr2 = "<joint name=\""+robot_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the mimic joint names
            substr1 = "<mimic joint=\"";
            substr2 = "<mimic joint=\""+robot_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the link names
            substr1 = "<link name=\"";
            substr2 = "<link name=\""+robot_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the child link names
            substr1 = "<child link=\"";
            substr2 = "<child link=\""+robot_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the parent link names
            substr1 = "<parent link=\"";
            substr2 = "<parent link=\""+robot_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Set the corrected robot descrition
            ros::param::set(my_robot_description.str(), str);

            //Check the corrected robot description:
            std::string s;
            ros::param::get(my_robot_description.str(),s);
            std::cout<<"robot = "<<s<<std::endl;

            //(3) Complete the string to launch the viewer.launch file:
            std::stringstream rlabel;
            rlabel << "rname"<<i;
            std::stringstream rbool;
            rbool << "r"<<i;
            std::stringstream guibool;
            if(gui) guibool << "true";
            else guibool << "false";
            std:cout<<"***** RVIZ CONFIG FILE *** "<<rvizfile<<std::endl;
            viewerlaunchstr = viewerlaunchstr + " " + rbool.str() + ":=true " + rlabel.str() +
                              ":=" + robot_namespace.str() + " gui:=" + guibool.str() + " rvizconfig:=" + rvizfile;
        }
        //Launch the viewer.launch file
        std::cout<<viewerlaunchstr.c_str()<<std::endl;
        system(viewerlaunchstr.c_str());

        robotsloaded = true;
    }
    return robotsloaded;
}


bool loadObstacles()
{
    if(!obstaclesloaded && receivedobstaclesscall)
    {
        ROS_INFO("loadObstacles!!!!!!!!");
        //Loop for all obstacles
        for(int i=0; i<numobstacles; i++)
        {
            ROS_INFO("obstacle %d", i);
            //(1) Launch the load_robot_description.launch that loads the robot_descrition within the robot namespace
            // obstacles will be visualized as robot using robot_description within their own namespace
            // recall that obstacles are defines as robots in theire urdf files...

            //Fill the string that will allow to obtain the complete robot filename
            //std::string rname = "\"\\$(find kautham)/demos/models/"+rfilename[i]+"\"";
            std::string oname = "\"" + ofilename[i] +"\"";
            //Fill the obstacle namespace. Obstacles namespaces are: obstacle0, obstacle1, etc...
            std::stringstream obstacle_namespace;
            obstacle_namespace << "obstacle"<<i;
            //Fill the launch command
            std::string launchstr = "roslaunch kautham load_robot_description.launch rname:="
                    + obstacle_namespace.str() + " rfilename:=" + oname;
            std::cout<<launchstr<<std::endl;
            //Launch
            system(launchstr.c_str());

            //(2) Modify the robot_description to include:
            // - names of links and joints that start with the robot namespace label, i.e. "obstacleX_"
            // - the correct paths for the geometries. Here it is assumed that the models are in the
            //   standard kautham directory, i.e. in: "package://kautham/demos/models/obstacles/"

            //Load the robot description string
            std::stringstream my_obstacle_description;
            my_obstacle_description << "obstacle"<<i<<"/robot_description";
            std::string str;
            ros::param::get(my_obstacle_description.str(), str);

            //Substitute the file paths
            //First extract from the ofilename[i] the path, i.e. from the begining till the last "/" character
            std::size_t found = ofilename[i].find_last_of("/");
            std::string substr1 = "filename=\"";
            std::string substr2 = std::string("filename=\"file://")+ofilename[i].substr(0,found+1);
            //std::cout<<"fname="<<ofilename[i]<<std::endl;
            //std::cout<<"found="<<found<<std::endl;
            //std::cout<<"fpath="<<substr2<<std::endl;
            //std::string substr2 = "filename=\"file:///home/jan/catkin/catkin_wsTAMP/src/kautham/demos/models/obstacles/chess/";
            //std::string substr2 = "filename=\"package://kautham/demos/models/obstacles/chess/";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the joint names
            substr1 = "<joint name=\"";
            substr2 = "<joint name=\""+obstacle_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the mimic joint names
            substr1 = "<mimic joint=\"";
            substr2 = "<mimic joint=\""+obstacle_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the link names
            substr1 = "<link name=\"";
            substr2 = "<link name=\""+obstacle_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the child link names
            substr1 = "<child link=\"";
            substr2 = "<child link=\""+obstacle_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Substitute the parent link names
            substr1 = "<parent link=\"";
            substr2 = "<parent link=\""+obstacle_namespace.str()+"_";
            for (size_t index = str.find(substr1, 0);
                 index != std::string::npos && substr1.length();
                 index = str.find(substr1, index + substr2.length() ) )
                    str.replace(index, substr1.length(), substr2);

            //Set the corrected robot descrition
            ros::param::set(my_obstacle_description.str(), str);

            //Check the corrected robot description:
            std::string s;
            ros::param::get(my_obstacle_description.str(),s);
            std::cout<<"robot = "<<s<<std::endl;
        }

        obstaclesloaded = true;
    }
    return obstaclesloaded;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "kautham_node_vis");
    ros::NodeHandle nh;

    ROS_INFO("Starting Kautham ROS Viewer");

    //Define Load services
    ros::ServiceServer service00 = nh.advertiseService("kautham_node_vis/LoadRobots",srvLoadRobots);
    ros::ServiceServer service01 = nh.advertiseService("kautham_node_vis/LoadObstacles",srvLoadObstacles);

    ros::Rate rate(2.0);
    while (nh.ok())
    {
        //ROS_INFO("looping node_viz +++++++++++++++++++++");
        ros::spinOnce();
        //If the data has been received through a call to the load service and it has not yet been loaded then proceed
        loadRobots();
        loadObstacles();
        //Wait until it's time for another iteration
        rate.sleep();
    }
    return 0;
}







