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
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace Kautham;

std::vector<std::string> rfilename;
std::vector<tf2::Transform> rtransform;
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
        rfiles.resize(numrobots);
        for(int i=0; numrobots;i++)
        {
            rfiles[i] = req.robotsfiles[i];
            rtransform[i] =
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
    ros::NodeHandle n;

    ROS_INFO("Starting Kautham ROS Viewer");

    ros::ServiceServer service00 = n.advertiseService("kautham_node_vis/LoadRobots",srvLoadRobots);

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

                std::stringstream my_robot_description;
                my_robot_description << "/robot"<<i<<"/robot_description";
                std::string launchstr = "roslaunch kautham load_robot_description.launch" +
                    " robot_description:=" + my_robot_description.str() +
                    " robot_filename:=" + rname;
                std::cout<<launchstr.c_str()<<std::endl;
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
                //ros::param::get("robot_description",s);
                //std::cout<<"robot = "<<s<<std::endl;
    
//TO DO: pose of robot to be read from kautham
                //Fill a geometry_msgs::TransformStamped with the grasp info
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = "/robot0/base_link";
                transformStamped.transform.translation.x = grasp01.getOrigin().getX();
                transformStamped.transform.translation.y = grasp01.getOrigin().getY();
                transformStamped.transform.translation.z = grasp01.getOrigin().getZ();
                transformStamped.transform.rotation.x = grasp01.getRotation().getX();
                transformStamped.transform.rotation.y = grasp01.getRotation().getY();
                transformStamped.transform.rotation.z = grasp01.getRotation().getZ();
                transformStamped.transform.rotation.w = grasp01.getRotation().getW();

                //Define a broadcaster and sendTransform
                static tf2_ros::StaticTransformBroadcaster br;
                br.sendTransform(transformStamped);

                //Modify the pose of the robot according to the kautham file
                std::string tf = "\"0.3 0.0 0.0  0.0 0.0 0.0 world base_link\"";
                std::string viewerlaunchstr = "roslaunch kautham viewer.launch basetf:="+tf;
                std::cout<<viewerlaunchstr.c_str()<<std::endl;
                system(viewerlaunchstr.c_str());

                loaded = true;
             }
        }
        //Wait until it's time for another iteration
        rate.sleep();
    }
    return 0;
}







