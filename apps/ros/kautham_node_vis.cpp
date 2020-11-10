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
std::vector<geometry_msgs::TransformStamped> rtransform;
visualization_msgs::MarkerArray kthobstacles;
int numrobots = 0;
bool robotsloaded = false;
bool receivedrobotscall = false;
bool obstaclesloaded = false;
bool receivedobstaclesscall = false;

//This service is to be called only once to load the rots info: filenames and pose
bool srvLoadRobots(kautham::LoadRobots::Request &req,
                   kautham::LoadRobots::Response &res)
{
    ROS_INFO( "STARTING Kautham LoadRobots");
    ROS_INFO( "********************************");
    receivedrobotscall = true; //flag to control that the service has been called
    if(!robotsloaded) //load data if the data has not yet been loaded
    {
        numrobots = req.robotsfiles.size();
        ROS_INFO("---numrobots = %d",numrobots);
        rfilename.resize(numrobots);
        rtransform.resize(numrobots);
        for(int i=0; i<numrobots;i++)
        {
            ROS_INFO("---robotfilenames[%d] = %s",i,req.robotsfiles[i].c_str());
            rfilename[i] = req.robotsfiles[i];
            rtransform[i] = req.robottransforms[i];
            ROS_INFO("---rtransform[%d] = (%f,%f,%f)",i,rtransform[i].transform.translation.x,
                     rtransform[i].transform.translation.y,
                     rtransform[i].transform.translation.z);
        }
        res.response = true;
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
    ROS_INFO( "STARTING Kautham LoadObjects");
    ROS_INFO( "********************************");

    unsigned int i;
    receivedobstaclesscall = true; //flag to control that the service has been called
    if(!obstaclesloaded) //load data if the data has not yet been loaded
    {
        i = 0;
        visualization_msgs::Marker marker;

        //PIECES SETUP
        ROS_DEBUG("LOADING OBJECTS");
        ROS_DEBUG("%d OBJECTs",(int)req.obstacles.markers.size());

        for(i=0; i<req.obstacles.markers.size(); i++) {
            marker.header.frame_id = req.obstacles.markers[i].header.frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = req.obstacles.markers[i].ns;
            marker.id = req.obstacles.markers[i].id;
            marker.mesh_resource = req.obstacles.markers[i].mesh_resource;

            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = req.obstacles.markers[i].pose;

            marker.scale.x = req.obstacles.markers[i].scale.x;
            marker.scale.y = req.obstacles.markers[i].scale.y;
            marker.scale.z = req.obstacles.markers[i].scale.z;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = req.obstacles.markers[i].color.r;
            marker.color.g = req.obstacles.markers[i].color.g;
            marker.color.b = req.obstacles.markers[i].color.b;
            marker.mesh_use_embedded_materials = req.obstacles.markers[i].mesh_use_embedded_materials;//if true the r,g,b peviously defined are overriden

            ROS_DEBUG("OBJECT: [%d]", marker.id);
            ROS_INFO("mesh_resource: %s", marker.mesh_resource.c_str());
            ROS_INFO("X value is: [%f]", marker.pose.position.x);
            ROS_INFO("Y value is: [%f]", marker.pose.position.y);
            ROS_INFO("Z value is: [%f]", marker.pose.position.z);
            ROS_INFO("ORI X value is: [%f]", marker.pose.orientation.x);
            ROS_INFO("ORI Y value is: [%f]", marker.pose.orientation.y);
            ROS_INFO("ORI Z value is: [%f]", marker.pose.orientation.z);
            ROS_INFO("ORI W value is: [%f]", marker.pose.orientation.w);

            kthobstacles.markers.push_back(marker);
        }
        res.response = true;
        obstaclesloaded = true;
        return true;
    }
    else
    {
        ROS_WARN("The objects have already been loaded. This service is only called once");
        res.response = false;
        return false;
    }
}


bool loadRobots()
{
    if(!robotsloaded && receivedrobotscall)
    {
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
            std::string substr1 = "filename=\"";
            std::string substr2 = "filename=\"package://kautham/demos/models/robots/";
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
            //std::string s;
            //ros::param::get(my_robot_description.str(),s);
            //std::cout<<"robot = "<<s<<std::endl;

            //(3) Broadcast transfporms where to place the robots.
            //It is assumed that the link of the robot base is called "base_link".
            rtransform[i].header.stamp = ros::Time::now();
            rtransform[i].header.frame_id = "world";
            std::stringstream my_child_frame_id;
            my_child_frame_id  << "robot"<<i<<"_base_link";
            rtransform[i].child_frame_id = my_child_frame_id.str();

            //Define a broadcaster and sendTransform
            static tf2_ros::StaticTransformBroadcaster br;
            br.sendTransform(rtransform[i]);

            //(4) Complete the string to launch the viewer.launch file:
            std::stringstream rlabel;
            rlabel << "rname"<<i;
            std::stringstream rbool;
            rbool << "r"<<i;
            viewerlaunchstr = viewerlaunchstr + " " + rbool.str() + ":=true " + rlabel.str() +":=" + robot_namespace.str();
        }
        //Launch the viewer.launch file
        std::cout<<viewerlaunchstr.c_str()<<std::endl;
        system(viewerlaunchstr.c_str());

        robotsloaded = true;
    }
    return robotsloaded;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "kautham_node_vis");
    ros::NodeHandle nh;

    ROS_INFO("Starting Kautham ROS Viewer");

    //Load services
    ros::ServiceServer service00 = nh.advertiseService("kautham_node_vis/LoadRobots",srvLoadRobots);
    ros::ServiceServer service01 = nh.advertiseService("kautham_node_vis/LoadObstacles",srvLoadObstacles);

    //The node advertises the marker poses
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );


    //Test the node without using info passed by the service
    /*
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
    receivedrobotscall = true;
    */
    //end test data

    ros::Rate rate(2.0);
    while (nh.ok())
    {

        if(obstaclesloaded) ROS_INFO("obstaclesloaded ....................");
        else ROS_INFO("NOT obstaclesloaded +++++++++++++++++++++");
        ros::spinOnce();
        //If the data has been received through a call to the load service and it has not yet been loaded then proceed
        loadRobots();

        //load environment
        if(obstaclesloaded)
        {
             vis_pub.publish( kthobstacles );
             ROS_INFO("publishing kthobstacles....................");
        }

        //Wait until it's time for another iteration
        rate.sleep();
    }
    return 0;
}







