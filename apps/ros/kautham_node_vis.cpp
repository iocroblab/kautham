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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <string>
using namespace Kautham;

std::vector<std::string> rfilename;
std::vector<geometry_msgs::TransformStamped> rtransform;
int numrobots = 0;
bool loaded = false;
bool receivedcall = false;

bool srvLoadRobots(kautham::LoadRobots::Request &req,
                        kautham::LoadRobots::Response &res)
{
    receivedcall = true;
    if(!loaded)
    {
        numrobots = req.robotsfiles.size();
        rfilename.resize(numrobots);
        for(int i=0; numrobots;i++)
        {
            rfilename[i] = req.robotsfiles[i];
            rtransform[i] = req.robottransforms[i];
        }
        return true;
    }
    else
    {
        ROS_WARN("The robots have already been loaded. This service is only called once");
        return false;
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "kautham_node_vis");
    ros::NodeHandle nh;

    ROS_INFO("Starting Kautham ROS Viewer");

    ros::ServiceServer service00 = nh.advertiseService("kautham_node_vis/LoadRobots",srvLoadRobots);

    //Test
    numrobots = 2;
    rfilename.resize(2);
    rfilename[0] = "robots/ur3_robotniq_A.urdf";
    rfilename[1] = "robots/ur3_robotniq_B.urdf";
    rtransform.resize(2);
    rtransform[0].transform.translation.x = 0.5;
    rtransform[0].transform.translation.y = 0.0;
    rtransform[0].transform.translation.z = 0.0;
    rtransform[0].transform.rotation.x = 0.0;
    rtransform[0].transform.rotation.y = 0.0;
    rtransform[0].transform.rotation.z = 0.0;
    rtransform[0].transform.rotation.w = 1.0;
    rtransform[1].transform.translation.x = -0.5;
    rtransform[1].transform.translation.y = 0.0;
    rtransform[1].transform.translation.z = 0.0;
    rtransform[1].transform.rotation.x = 0.0;
    rtransform[1].transform.rotation.y = 0.0;
    rtransform[1].transform.rotation.z = 0.0;
    rtransform[1].transform.rotation.w = 1.0;
    receivedcall = true;
    //end test data

    ros::Rate rate(2.0);
    while (nh.ok())
    {
        if(!loaded && receivedcall)
        {
            for(int i=0; i<numrobots; i++)
            {
                //Launch the load_robot_description.launch that loads the robot_descrition
                //std::string name = "\"\\$(find kautham)/demos/models/robots/ur3_robotniq_A.urdf\"";
                std::string rname = "\"\\$(find kautham)/demos/models/"+rfilename[i]+"\"";


                std::stringstream my_robot_name;
                my_robot_name << "/robot"<<i;
                std::stringstream my_robot_description;
                my_robot_description << "/robot"<<i<<"/robot_description";
                std::string launchstr = "roslaunch kautham load_robot_description.launch rname:="
                        + my_robot_name.str() + " rfilename:=" + rname;
                std::cout<<launchstr<<std::endl;
                system(launchstr.c_str());
    
                //Modify the robot_description to include the correct paths for the geometries
                //here it is assumed that the models are in the standard kautham directory
                std::string str;
                ros::param::get(my_robot_description.str(), str);
                std::string substr1 = "filename=\"";
                std::string substr2 = "filename=\"package://kautham/demos/models/robots/";
                for (size_t index = str.find(substr1, 0);
                     index != std::string::npos && substr1.length();
                     index = str.find(substr1, index + substr2.length() ) )
                        str.replace(index, substr1.length(), substr2);
                ros::param::set(my_robot_description.str(), str);
                //check:
                //std::string s;
                //ros::param::get(my_robot_description.str(),s);
                //std::cout<<"robot = "<<s<<std::endl;
    
//TO DO: pose of robot to be read from kautham
                //Fill a geometry_msgs::TransformStamped with the grasp info

                rtransform[i].header.stamp = ros::Time::now();
                rtransform[i].header.frame_id = "world";
                std::stringstream my_child_frame_id;
                my_child_frame_id  << "/robot"<<i<<"/base_link";
                //rtransform[i].child_frame_id = my_child_frame_id.str();
                rtransform[i].child_frame_id = "base_link";

                //Define a broadcaster and sendTransform
                static tf2_ros::StaticTransformBroadcaster br;
                br.sendTransform(rtransform[i]);
            }
            std::string viewerlaunchstr = "roslaunch kautham viewer.launch";
            std::cout<<viewerlaunchstr.c_str()<<std::endl;
            system(viewerlaunchstr.c_str());

            loaded = true;
        }
        //Wait until it's time for another iteration
        rate.sleep();
    }
    return 0;
}







