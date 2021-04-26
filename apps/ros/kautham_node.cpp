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

/* Author: Aliakbar Akbari, Nestor Garcia Hidalgo */


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>

#include <Inventor/SoDB.h>

#include <kautham/kauthamshell.h>

#include <kautham/CloseProblem.h>
#include <kautham/ProblemOpened.h>
#include <kautham/OpenProblem.h>
#include <kautham/OpenProblemStream.h>
#include <kautham/CheckCollision.h>
#include <kautham/SetRobotsConfig.h>
#include <kautham/SetObstaclesConfig.h>
#include <kautham/SetQuery.h>
#include <kautham/SetInit.h>
#include <kautham/SetGoal.h>
#include <kautham/SetInitObs.h>
#include <kautham/ClearSampleSet.h>
#include <kautham/SetRobControls.h>
#include <kautham/SetRobControlsNoQuery.h>
#include <kautham/SetRobControlsStream.h>
#include <kautham/SetDefaultRobControls.h>
#include <kautham/SetObsControls.h>
#include <kautham/SetObsControlsStream.h>
#include <kautham/SetFixedObsControls.h>
#include <kautham/SetPlannerByName.h>
#include <kautham/SetPlanner.h>
#include <kautham/SetPlannerStream.h>
#include <kautham/SetPlannerParameter.h>
#include <kautham/Solve.h>
#include <kautham/GetPath.h>
#include <kautham/AddRobot.h>
#include <kautham/RemoveRobot.h>
#include <kautham/AddObstacle.h>
#include <kautham/RemoveObstacle.h>
#include <kautham/AttachObstacle2RobotLink.h>
#include <kautham/DetachObstacle.h>
#include <kautham/Connect.h>
#include <kautham/GetLastPlanComputationTime.h>
#include <kautham/GetNumEdges.h>
#include <kautham/GetNumVertices.h>
#include <kautham/ObsPos.h>
#include <kautham/RobPos.h>
#include <kautham/FindIK.h>
#include <kautham/LoadRobots.h>
#include <kautham/LoadObstacles.h>
#include <kautham/VisualizeScene.h>
#include <kautham/robconf.h>
#include <kautham/GetObstaclesNames.h>
#include <kautham/PathDofNames.h>


using namespace std;
using namespace Kautham;

std_msgs::String my_msg;
kauthamshell* ksh;
bool visualizescene = false;
bool guisliders = false;
std::vector<ros::Publisher*> joints_publishers;
std::vector<sensor_msgs::JointState> joint_state_robot;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;
std::vector<geometry_msgs::TransformStamped> rtransform;
std::map<std::string, geometry_msgs::TransformStamped> otransform;
int numrobots = 0;
int numobstacles = 0;

//Calls the srvLoadRobots service from the kautham_node_vis
//that will visualize the scene.
bool srvVisualizeScene(kautham::VisualizeScene::Request &req,
                       kautham::VisualizeScene::Response &resp) {

    ros::NodeHandle n;

    //Now call the LoadObjects to load the MarkerArray to visualize them in rviz
    ros::service::waitForService("/kautham_node_vis/LoadObstacles");
    kautham::LoadObstacles kthloadobstacles_srv;
    std::vector<std::string> obstaclesfilenames;
    numobstacles = ksh->getNumObstacles();
    ksh->getObstaclesFileNames(obstaclesfilenames);

    kthloadobstacles_srv.request.obstaclesfiles.resize(numobstacles);
    kthloadobstacles_srv.request.obstaclesnames.resize(numobstacles);
    kthloadobstacles_srv.request.obstacletransforms.resize(numobstacles);

    ROS_INFO("Kautham srvVisualizeScene");
    ROS_INFO("numobstacles = %d",numobstacles);
    std::vector< std::vector<float> > poses;
    poses.clear();
    poses.resize(numobstacles);

    uint i=0;
    for (std::pair<std::string, Robot*> element : ksh->getObstaclesMap() )
    {
        ROS_INFO("obstaclesfilenames[%d] = %s",i,obstaclesfilenames[i].c_str());
        kthloadobstacles_srv.request.obstaclesfiles[i] = obstaclesfilenames[i].c_str();
        kthloadobstacles_srv.request.obstaclesnames[i] = element.first;
        ksh->getObstaclePos(element.first, poses[i]);
        kthloadobstacles_srv.request.obstacletransforms[i].transform.translation.x = poses[i][0];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.translation.y = poses[i][1];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.translation.z = poses[i][2];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.rotation.x = poses[i][3];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.rotation.y = poses[i][4];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.rotation.z = poses[i][5];
        kthloadobstacles_srv.request.obstacletransforms[i].transform.rotation.w = poses[i][6];

        //It is assumed that the link of the obstacle base is called "base".
        geometry_msgs::TransformStamped ot;
        ot.header.stamp = ros::Time::now();
        ot.header.frame_id = "world";
        std::stringstream my_child_frame_id;
        my_child_frame_id  << "obstacle_"<<element.first<<"_base";
        ot.child_frame_id = my_child_frame_id.str();
        ot.transform.translation.x = poses[i][0];
        ot.transform.translation.y = poses[i][1];
        ot.transform.translation.z = poses[i][2];
        ot.transform.rotation.x = poses[i][3];
        ot.transform.rotation.y = poses[i][4];
        ot.transform.rotation.z = poses[i][5];
        ot.transform.rotation.w = poses[i][6];
        otransform.insert(std::pair<string,geometry_msgs::TransformStamped>(element.first, ot));

        /*
        ROS_INFO( "pose[0] = %f",poses[i][0]);
        ROS_INFO( "pose[1] = %f",poses[i][1]);
        ROS_INFO( "pose[2] = %f",poses[i][2]);
        ROS_INFO( "pose[3] = %f",poses[i][3]);
        ROS_INFO( "pose[4] = %f",poses[i][4]);
        ROS_INFO( "pose[5] = %f",poses[i][5]);
        ROS_INFO( "pose[6] = %f",poses[i][6]);
        */

        i++;
    }

    ros::ServiceClient kthloadobstacles_client = n.serviceClient<kautham::LoadObstacles>("/kautham_node_vis/LoadObstacles");
    ROS_INFO( "CALLING Kautham LoadObstacles");
    kthloadobstacles_client.call(kthloadobstacles_srv);

    if (kthloadobstacles_srv.response.response == true) {
        ROS_INFO( "Kautham LoadObstacles correctly loaded the obstacles" );
    } else {
        ROS_ERROR( "ERROR Kautham LoadObstacles could not loaded the obstacles" );
        return false;
    }

    //-----------------------------------------------------------------
    //Now call the LoadRobots to load the robots and visualize them in rviz
    ros::service::waitForService("/kautham_node_vis/LoadRobots");

    kautham::LoadRobots kthloadrobots_srv;
    std::vector<std::string> robotfilenames;
    numrobots = ksh->getNumRobots();
    ksh->getRobotFileNames(robotfilenames);

    kthloadrobots_srv.request.robotsfiles.resize(numrobots);
    kthloadrobots_srv.request.robottransforms.resize(numrobots);
    guisliders = req.guisliders;
    kthloadrobots_srv.request.gui = req.guisliders;

    ROS_INFO("Kautham srvVisualizeScene");
    ROS_INFO("numrobots = %d",numrobots);
    //std::vector< std::vector<float> > poses;
    poses.clear();
    poses.resize(numrobots);
    rtransform.resize(numobstacles);
    for(int i=0; i<numrobots; i++)
    {
        ROS_INFO("robotfilenames[%d] = %s",i,robotfilenames[i].c_str());
        kthloadrobots_srv.request.robotsfiles[i] = robotfilenames[i].c_str();
        ksh->getRobPos(i, poses[i]);
        kthloadrobots_srv.request.robottransforms[i].transform.translation.x = poses[i][0];
        kthloadrobots_srv.request.robottransforms[i].transform.translation.y = poses[i][1];
        kthloadrobots_srv.request.robottransforms[i].transform.translation.z = poses[i][2];
        kthloadrobots_srv.request.robottransforms[i].transform.rotation.x = poses[i][3];
        kthloadrobots_srv.request.robottransforms[i].transform.rotation.y = poses[i][4];
        kthloadrobots_srv.request.robottransforms[i].transform.rotation.z = poses[i][5];
        kthloadrobots_srv.request.robottransforms[i].transform.rotation.w = poses[i][6];

        //It is assumed that the link of the robot base is called "base_link".
        rtransform[i].header.stamp = ros::Time::now();
        rtransform[i].header.frame_id = "world";
        std::stringstream my_child_frame_id;
        my_child_frame_id  << "robot"<<i<<"_base_link";
        rtransform[i].child_frame_id = my_child_frame_id.str();
        rtransform[i].transform.translation.x = poses[i][0];
        rtransform[i].transform.translation.y = poses[i][1];
        rtransform[i].transform.translation.z = poses[i][2];
        rtransform[i].transform.rotation.x = poses[i][3];
        rtransform[i].transform.rotation.y = poses[i][4];
        rtransform[i].transform.rotation.z = poses[i][5];
        rtransform[i].transform.rotation.w = poses[i][6];
    }
    kthloadrobots_srv.request.rvizfile = req.rvizfile;

    ros::ServiceClient kthloadrobots_client = n.serviceClient<kautham::LoadRobots>("/kautham_node_vis/LoadRobots");
    ROS_INFO( "CALLING Kautham LoadRobots");
    kthloadrobots_client.call(kthloadrobots_srv);

    if (kthloadrobots_srv.response.response == true) {
        ROS_INFO( "Kautham LoadRobots correctly loaded the robots" );
    } else {
        ROS_ERROR( "ERROR Kautham LoadRobots could not loaded the robots" );
        return false;
    }

    //Create the publishers to publish the joints of each robot
    joints_publishers.resize(numrobots);
    joint_state_robot.resize(numrobots);
    for(int i=0; i<numrobots;i++)
    {
        std::stringstream topicname;
        topicname << "/robot"<<i<<"/joint_states";
        //std::cout<<topicname.str()<<std::endl;
        joints_publishers[i] = new ros::Publisher(n.advertise<sensor_msgs::JointState>(topicname.str().c_str(), 1));

        std::vector<std::string> jointnames;
        ksh->getRobotJointNames(i,jointnames);
        joint_state_robot[i].name.resize(jointnames.size());
        joint_state_robot[i].position.resize(jointnames.size());
        //change the names of the joints accoridng to the namespace (add prefix robotX_)
        for(unsigned int j=0; j<jointnames.size();j++)
        {
            std::stringstream jointnameprefix;
            jointnameprefix << "robot"<<i<<"_";
            //std::cout<<jointnameprefix.str()+jointnames[j]<<std::endl;
            joint_state_robot[i].name[j] = jointnameprefix.str()+jointnames[j];
            joint_state_robot[i].position[j] = 0;
        }
    }
    visualizescene = true;
    resp.response=true;
    return true;
}


bool srvCloseProblem(kautham::CloseProblem::Request &req,
                     kautham::CloseProblem::Response &res) {
    (void) req;//unused
    (void) res;//unused
    ksh->closeProblem();

    return true;
}


bool srvProblemOpened(kautham::ProblemOpened::Request &req,
                      kautham::ProblemOpened::Response &res) {
    (void) req;//unused
    res.response = ksh->problemOpened();

    return true;
}


bool srvGetObstaclesNames(kautham::GetObstaclesNames::Request &req,
                      kautham::GetObstaclesNames::Response &res) {
    (void) req;//unused
    std::vector<std::string> obsnames;
    ksh->getObstaclesNames(obsnames);
    res.obsnames = obsnames;
    return true;
}


bool srvOpenProblem(kautham::OpenProblem::Request &req,
                    kautham::OpenProblem::Response &res) {
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

    if (ksh->openProblem(req.problem, def_path)) {
        ROS_INFO("The problem file has been opened successfully.\n");
        my_msg.data = "The problem file has been opened successfully.";
        res.response = true;
    } else {
        ROS_INFO("The problem file couldn't be opened.\n");
        my_msg.data = "The problem file couldn't be opened.";
        res.response = false;
    }

    return true;
}


bool srvOpenProblemStream(kautham::OpenProblemStream::Request &req,
                          kautham::OpenProblemStream::Response &res) {
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

    filebuf fb;
    fb.open(req.problem.c_str(),ios::in);
    istream is(&fb);

    if (ksh->openProblem(&is,def_path)) {
        ROS_INFO("The problem file has been opened successfully.\n");
        my_msg.data = "The problem file has been opened successfully.";
        res.response = true;
    } else {
        ROS_INFO("The problem file couldn't be opened.\n");
        my_msg.data = "The problem file couldn't be opened.";
        res.response = false;
    }

    return true;
}


bool srvCheckCollision(kautham::CheckCollision::Request &req,
                       kautham::CheckCollision::Response &res) {


  for (unsigned int i = 0; i < req.config.size(); ++i) {
    cout << req.config.at(i) << " ";
  }
  cout << endl;
  std::pair< std::pair<int, string> , std::pair<int,int> > colliding_elements;
  bool collisionFree;
  string msg;
  res.response = ksh->checkCollision(req.config,&collisionFree, &msg, &colliding_elements);
  res.collisionFree = res.response&&collisionFree;
  res.collidingRob = colliding_elements.first.first;
  res.collidedObs = colliding_elements.first.second;
  res.msg = msg;
  return true;
}


bool srvSetRobotsConfig(kautham::SetRobotsConfig::Request &req,
                       kautham::SetRobotsConfig::Response &res) {

    std::vector<RobConf> config;
    res.response = ksh->setRobotsConfig(req.controls, config);
    res.config.resize(config.size());
    for(unsigned int i=0; i<config.size();i++)
    {
        //fill the response
        res.config[i].base.position.x = config[i].first.getPos().at(0);
        res.config[i].base.position.y = config[i].first.getPos().at(1);
        res.config[i].base.position.z = config[i].first.getPos().at(2);
        res.config[i].base.orientation.x = config[i].first.getOrient().at(0);
        res.config[i].base.orientation.y = config[i].first.getOrient().at(1);
        res.config[i].base.orientation.z = config[i].first.getOrient().at(2);
        res.config[i].base.orientation.w = config[i].first.getOrient().at(3);
        res.config[i].joints = config[i].second.getCoordinates();

        if(visualizescene)
        {
            //update the broadcasted transforms for the base
            rtransform[i].header.stamp = ros::Time::now();
            rtransform[i].header.frame_id = "world";
            std::stringstream my_child_frame_id;
            my_child_frame_id  << "robot"<<i<<"_base_link";
            rtransform[i].child_frame_id = my_child_frame_id.str();
            rtransform[i].transform.translation.x = config[i].first.getPos().at(0);
            rtransform[i].transform.translation.y = config[i].first.getPos().at(1);
            rtransform[i].transform.translation.z = config[i].first.getPos().at(2);
            rtransform[i].transform.rotation.x = config[i].first.getOrient().at(0);
            rtransform[i].transform.rotation.y = config[i].first.getOrient().at(1);
            rtransform[i].transform.rotation.z = config[i].first.getOrient().at(2);
            rtransform[i].transform.rotation.w = config[i].first.getOrient().at(3);

            //fill the joint values to be published
            if(!guisliders)
            {
                for(unsigned int j=0; j<joint_state_robot[i].position.size();j++)
                    joint_state_robot[i].position[j] = config[i].second.getCoordinates().at(j);
            }
        }
        /*
        std::cout<<"ROBOT "<<i<<std::endl;
        std::cout<<"x = "<<res.config[i].base.position.x<<std::endl;
        std::cout<<"y = "<<res.config[i].base.position.y<<std::endl;
        std::cout<<"z = "<<res.config[i].base.position.z<<std::endl;
        std::cout<<"qx = "<<res.config[i].base.orientation.x<<std::endl;
        std::cout<<"qy = "<<res.config[i].base.orientation.y<<std::endl;
        std::cout<<"qz = "<<res.config[i].base.orientation.y<<std::endl;
        std::cout<<"qw = "<<res.config[i].base.orientation.w<<std::endl;
        std::cout<<"q = (";
        */
        //std::cout<<"ROBOT "<<i<<std::endl;
        //for(unsigned int j=0; j<joint_state_robot[i].position.size();j++)
        //    std::cout<<res.config[i].joints[j]<<" ";
        //std::cout<<std::endl;
        //std::cout<<"ROBOT "<<i<<std::endl;
    }
}


bool srvFindIK(kautham::FindIK::Request &req,
                       kautham::FindIK::Response &res) {

    std::vector <float> solution;
    res.response = ksh->findIK(req.robIndx, req.armType, req.pos, req.conf, req.maintSameWrist, &solution);
    res.conf = solution;
    return true;
}


bool srvSetObstaclesConfig(kautham::SetObstaclesConfig::Request &req,
                           kautham::SetObstaclesConfig::Response &res) {

    res.response = ksh->setObstaclesConfig(req.config);

    return true;
}


bool srvSetQuery(kautham::SetQuery::Request &req,
                 kautham::SetQuery::Response &res) {
    res.response = ksh->setQuery(req.init,req.goal);

    return true;
}


bool srvSetInit(kautham::SetInit::Request &req,
                kautham::SetInit::Response &res) {
    res.response = ksh->setInit(req.init);

    return true;
}


bool srvSetGoal(kautham::SetGoal::Request &req,
                kautham::SetGoal::Response &res) {
    res.response = ksh->setGoal(req.goal);

    return true;
}


bool srvSetInitObs(kautham::SetInitObs::Request &req,
                   kautham::SetInitObs::Response &res) {
    res.response = ksh->setInitObs(req.initObs);

    return true;
}


bool srvClearSampleSet(kautham::ClearSampleSet::Request &req,
                       kautham::ClearSampleSet::Response &res) {
    (void) req;//unused
    res.response = ksh->clearSampleSet();

    return true;
}


bool srvSetRobControlsNoQuery(kautham::SetRobControlsNoQuery::Request &req,
                       kautham::SetRobControlsNoQuery::Response &res) {
    res.response = ksh->setRobControlsNoQuery(req.controls);

    return true;
}



bool srvSetRobControls(kautham::SetRobControls::Request &req,
                       kautham::SetRobControls::Response &res) {
    res.response = ksh->setRobControls(req.controls,req.init,req.goal);

    return true;
}


bool srvSetRobControlsStream(kautham::SetRobControlsStream::Request &req,
                             kautham::SetRobControlsStream::Response &res) {
    filebuf fb;
    fb.open(req.controls.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setRobControls(&is,req.init,req.goal);

    return true;
}


bool srvSetDefaultRobControls(kautham::SetDefaultRobControls::Request &req,
                              kautham::SetDefaultRobControls::Response &res) {
    res.response = ksh->setDefaultRobControls(req.init,req.goal);

    return true;
}


bool srvSetObsControls(kautham::SetObsControls::Request &req,
                       kautham::SetObsControls::Response &res) {
    res.response = ksh->setObsControls(req.controls,req.initObs);

    return true;
}


bool srvSetObsControlsStream(kautham::SetObsControlsStream::Request &req,
                             kautham::SetObsControlsStream::Response &res) {
    filebuf fb;
    fb.open(req.controls.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setObsControls(&is,req.initObs);

    return true;
}


bool srvSetFixedObsControls(kautham::SetFixedObsControls::Request &req,
                            kautham::SetFixedObsControls::Response &res) {
    (void) req;//unused
    res.response = ksh->setFixedObsControls();

    return true;
}


bool srvSetPlannerByName(kautham::SetPlannerByName::Request &req,
                         kautham::SetPlannerByName::Response &res) {
    res.response = ksh->setPlannerByName(req.name);

    return true;
}


bool srvSetPlanner(kautham::SetPlanner::Request &req,
                   kautham::SetPlanner::Response &res) {
    res.response = ksh->setPlanner(req.planner);

    return true;
}


bool srvSetPlannerStream(kautham::SetPlannerStream::Request &req,
                         kautham::SetPlannerStream::Response &res) {
    filebuf fb;
    fb.open(req.planner.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setPlanner(&is);

    return true;
}


bool srvSetPlannerParameter(kautham::SetPlannerParameter::Request &req,
                            kautham::SetPlannerParameter::Response &res) {
    res.response = ksh->setPlannerParameter(req.parameter,req.value);

    return true;
}


bool srvSolve(kautham::Solve::Request &req,
              kautham::Solve::Response &res) {
    (void) req;//unused
    res.response = ksh->solve(std::cout);

    return true;
}

//this service returns the names of the dof in the path returned by GetPath
//e.g. for the Tiago we have that each configuration of the path has 23 values according to:
//dofnames =  ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'wheel_right_joint', 'wheel_left_joint', 'torso_fixed_joint', 'torso_lift_joint', 'head_1_joint', 'head_2_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_joint', 'gripper_right_finger_joint', 'gripper_left_finger_joint']
bool srvPathDofNames(kautham::PathDofNames::Request &req,
              kautham::PathDofNames::Response &res) {
    (void) req;//unused

    std::vector< std::vector<std::string> > jnames;
    jnames.resize( ksh->getNumRobots() );
    unsigned int k=0;
    for(unsigned int i = 0; i < ksh->getNumRobots(); i++) {
      ksh->getRobotJointNames(i,jnames[i]);
      res.dofnames.push_back("x");
      res.dofnames.push_back("y");
      res.dofnames.push_back("z");
      res.dofnames.push_back("qx");
      res.dofnames.push_back("qy");
      res.dofnames.push_back("qz");
      res.dofnames.push_back("qw");
      for(unsigned int j=0;j<jnames[i].size();j++)
        res.dofnames.push_back(jnames[i][j]);
    }
    return true;
}

//See srvPathDofNames to know the contents of the configuration values of a path
bool srvGetPath(kautham::GetPath::Request &req,
                kautham::GetPath::Response &res) {
    (void) req;//unused
    ostringstream oss;
    if (ksh->getPath(oss)) {
        vector < vector < float > > path;
        istringstream iss(oss.str());
        for (string str; getline(iss,str); ) {
            vector < float > conf;
            std::istringstream strs(str);
            int chars_to_read = strs.str().size();
            while (chars_to_read > 0) {
                getline(strs,str,' ');
                if (str.size() > 0) {
                    conf.push_back(atof(str.c_str()));
                }
                chars_to_read -= str.size() + 1;
            }
            path.push_back(conf);
        }
        res.response.resize(path.size());
        for (unsigned int i = 0; i < path.size(); ++i) {
            res.response[i].v.resize(path.at(i).size());
            for (unsigned int j = 0; j < path.at(i).size(); ++j) {
                res.response[i].v[j] = path.at(i).at(j);
            }
        }
    }

    return true;
}


bool srvAddRobot(kautham::AddRobot::Request &req,
                            kautham::AddRobot::Response &res) {
    vector< vector<float> > limits, mapMatrix;
    limits.resize(req.limits.size());
    for (unsigned int i = 0; i < limits.size(); ++i) {
        limits[i].resize(req.limits[i].v.size());
        for (unsigned int j = 0; j < limits[i].size(); ++j) {
            limits[i][j] = req.limits[i].v[j];
        }
    }
    mapMatrix.resize(req.mapMatrix.size());
    for (unsigned int i = 0; i < mapMatrix.size(); ++i) {
        mapMatrix[i].resize(req.mapMatrix[i].v.size());
        for (unsigned int j = 0; j < mapMatrix[i].size(); ++j) {
            mapMatrix[i][j] = req.mapMatrix[i].v[j];
        }
    }
    res.response = ksh->addRobot(req.robot,req.scale,req.home,
                                 limits,mapMatrix,req.offMatrix);

    return true;
}


bool srvRemoveRobot(kautham::RemoveRobot::Request &req,
                            kautham::RemoveRobot::Response &res) {
    res.response = ksh->removeRobot(req.index);

    return true;
}


bool srvAddObstacle(kautham::AddObstacle::Request &req,
                            kautham::AddObstacle::Response &res) {
    res.response = ksh->addObstacle(req.obsname,req.scale,req.home);

    return true;
}


bool srvRemoveObstacle(kautham::RemoveObstacle::Request &req,
                            kautham::RemoveObstacle::Response &res) {
    res.response = ksh->removeObstacle(req.obsname);

    return true;
}


bool srvAttachObstacle2RobotLink(kautham::AttachObstacle2RobotLink::Request &req,
                            kautham::AttachObstacle2RobotLink::Response &res) {
    res.response = ksh->attachObstacle2RobotLink(req.robot,req.link,req.obs);

    return true;
}


bool srvDetachObstacle(kautham::DetachObstacle::Request &req,
                            kautham::DetachObstacle::Response &res) {
    res.response = ksh->detachObstacle(req.obsname);

    return true;
}


bool srvConnect(kautham::Connect::Request &req,
                            kautham::Connect::Response &res) {
    res.response = ksh->connect(req.sample1,req.sample2);

    return true;
}


bool srvGetLastPlanComputationTime(kautham::GetLastPlanComputationTime::Request &req,
                            kautham::GetLastPlanComputationTime::Response &res) {
    (void) req;//unused
    res.time = ksh->getLastPlanComputationTime();

    return true;
}


bool srvGetNumEdges(kautham::GetNumEdges::Request &req,
                            kautham::GetNumEdges::Response &res) {
    (void) req;//unused
    res.num = ksh->getNumEdges();

    return true;
}


bool srvGetNumVertices(kautham::GetNumVertices::Request &req,
                            kautham::GetNumVertices::Response &res) {
    (void) req;//unused
    res.num = ksh->getNumVertices();

    return true;
}

bool srvSetObstaclPos(kautham::ObsPos::Request &req,
                            kautham::ObsPos::Response &res) {
    res.response = ksh->setObstaclePos(req.obsname, req.setPos);

    if(visualizescene)
    {
        std::map<std::string, geometry_msgs::TransformStamped>::iterator it = otransform.find(req.obsname);
        it->second.header.stamp = ros::Time::now();
        it->second.header.frame_id = "world";
        std::stringstream my_child_frame_id;
        my_child_frame_id  << "obstacle_"<<req.obsname<<"_base";
        it->second.child_frame_id = my_child_frame_id.str();
        it->second.transform.translation.x = req.setPos.at(0);
        it->second.transform.translation.y = req.setPos.at(1);
        it->second.transform.translation.z = req.setPos.at(2);
        it->second.transform.rotation.x = req.setPos.at(3);
        it->second.transform.rotation.y = req.setPos.at(4);
        it->second.transform.rotation.z = req.setPos.at(5);
        it->second.transform.rotation.w = req.setPos.at(6);
    }
    return true;
}

bool srvGetObstaclPos(kautham::ObsPos::Request &req,
                            kautham::ObsPos::Response &res) {
    std::vector<float> obsPos;
    res.response = ksh->getObstaclePos(req.obsname, obsPos);
    if(res.response) {
        res.getPos = obsPos;
    }
    return true;
}

bool srvSetRobPos(kautham::RobPos::Request &req,
                            kautham::RobPos::Response &res) {
    res.response = ksh->setRobPos(req.index, req.setPos);

    return true;
}

bool srvGetRobPos(kautham::RobPos::Request &req,
                            kautham::RobPos::Response &res) {
    std::vector<float> robPos;
    res.response = ksh->getRobPos(req.index, robPos);
    if(res.response) {
        res.getPos = robPos;
    }

    return true;
}

bool srvGetRobHomePos(kautham::RobPos::Request &req,
                            kautham::RobPos::Response &res) {
    std::vector<float> robPos;
    res.response = ksh->getRobHomePos(req.index, robPos);
    if(res.response) {
        res.getPos = robPos;
    }

    return true;
}


int main (int argc, char **argv) {
    ros::init(argc, argv, "kautham_node");
    ros::NodeHandle n;

    ROS_INFO("Starting Kautham_Service");

    SoDB::init();
    ksh = new kauthamshell();

    ros::ServiceServer service00 = n.advertiseService("kautham_node/CloseProblem",srvCloseProblem);
    ros::ServiceServer service01 = n.advertiseService("kautham_node/ProblemOpened",srvProblemOpened);
    ros::ServiceServer service02 = n.advertiseService("kautham_node/OpenProblem",srvOpenProblem);
    ros::ServiceServer service03 = n.advertiseService("kautham_node/OpenProblemStream",srvOpenProblemStream);
    ros::ServiceServer service04 = n.advertiseService("kautham_node/CheckCollision",srvCheckCollision);
    ros::ServiceServer service05 = n.advertiseService("kautham_node/SetRobotsConfig",srvSetRobotsConfig);
    ros::ServiceServer service06 = n.advertiseService("kautham_node/SetObstaclesConfig",srvSetObstaclesConfig);
    ros::ServiceServer service07 = n.advertiseService("kautham_node/SetQuery",srvSetQuery);
    ros::ServiceServer service08 = n.advertiseService("kautham_node/SetInit",srvSetInit);
    ros::ServiceServer service09 = n.advertiseService("kautham_node/SetGoal",srvSetGoal);
    ros::ServiceServer service10 = n.advertiseService("kautham_node/SetInitObs",srvSetInitObs);
    ros::ServiceServer service11 = n.advertiseService("kautham_node/ClearSampleSet",srvClearSampleSet);
    ros::ServiceServer service12 = n.advertiseService("kautham_node/SetRobControls",srvSetRobControls);
    ros::ServiceServer service12b = n.advertiseService("kautham_node/SetRobControlsNoQuery",srvSetRobControlsNoQuery);
    ros::ServiceServer service13 = n.advertiseService("kautham_node/SetRobControlsStream",srvSetRobControlsStream);
    ros::ServiceServer service14 = n.advertiseService("kautham_node/SetDefaultRobControls",srvSetDefaultRobControls);
    ros::ServiceServer service15 = n.advertiseService("kautham_node/SetObsControls",srvSetObsControls);
    ros::ServiceServer service16 = n.advertiseService("kautham_node/SetObsControlsStream",srvSetObsControlsStream);
    ros::ServiceServer service17 = n.advertiseService("kautham_node/SetFixedObsControls",srvSetFixedObsControls);
    ros::ServiceServer service18 = n.advertiseService("kautham_node/SetPlannerByName",srvSetPlannerByName);
    ros::ServiceServer service19 = n.advertiseService("kautham_node/SetPlanner",srvSetPlanner);
    ros::ServiceServer service20 = n.advertiseService("kautham_node/SetPlannerStream",srvSetPlannerStream);
    ros::ServiceServer service21 = n.advertiseService("kautham_node/SetPlannerParameter",srvSetPlannerParameter);
    ros::ServiceServer service22 = n.advertiseService("kautham_node/Solve",srvSolve);
    ros::ServiceServer service23 = n.advertiseService("kautham_node/GetPath",srvGetPath);
    ros::ServiceServer service24 = n.advertiseService("kautham_node/AddRobot",srvAddRobot);
    ros::ServiceServer service25 = n.advertiseService("kautham_node/RemoveRobot",srvRemoveRobot);
    ros::ServiceServer service26 = n.advertiseService("kautham_node/AddObstacle",srvAddObstacle);
    ros::ServiceServer service27 = n.advertiseService("kautham_node/RemoveObstacle",srvRemoveObstacle);
    ros::ServiceServer service28 = n.advertiseService("kautham_node/AttachObstacle2RobotLink",srvAttachObstacle2RobotLink);
    ros::ServiceServer service29 = n.advertiseService("kautham_node/DetachObstacle",srvDetachObstacle);
    ros::ServiceServer service30 = n.advertiseService("kautham_node/Connect",srvConnect);
    ros::ServiceServer service31 = n.advertiseService("kautham_node/GetLastPlanComputationTime",srvGetLastPlanComputationTime);
    ros::ServiceServer service32 = n.advertiseService("kautham_node/GetNumEdges",srvGetNumEdges);
    ros::ServiceServer service33 = n.advertiseService("kautham_node/GetNumVertices",srvGetNumVertices);
    ros::ServiceServer service34 = n.advertiseService("kautham_node/SetObstaclePos",srvSetObstaclPos);
    ros::ServiceServer service35 = n.advertiseService("kautham_node/GetObstaclePos", srvGetObstaclPos);
    ros::ServiceServer service38 = n.advertiseService("kautham_node/FindIK",srvFindIK);
    ros::ServiceServer service39 = n.advertiseService("kautham_node/SetRobotPos",srvSetRobPos);
    ros::ServiceServer service40 = n.advertiseService("kautham_node/GetRobotPos",srvGetRobPos);
    ros::ServiceServer service41 = n.advertiseService("kautham_node/GetRobotHomePos",srvGetRobHomePos);
    ros::ServiceServer service42 = n.advertiseService("kautham_node/VisualizeScene",srvVisualizeScene);
    ros::ServiceServer service43 = n.advertiseService("kautham_node/GetObstaclesNames",srvGetObstaclesNames);
    ros::ServiceServer service44 = n.advertiseService("kautham_node/PathDofNames",srvPathDofNames);

    br.reset(new tf2_ros::StaticTransformBroadcaster);
    //ros::spin();

    ros::Rate rate(2.0);
    while (n.ok())
    {
        //ROS_INFO("looping kautham node -----------------------");
        ros::spinOnce();
        //publish joint values if rviz is shown and joint_state_publisher_gui is not running
        if(visualizescene)
        {
            if(!guisliders)
            {
                for(int i=0; i<numrobots; i++)
                {
                    joint_state_robot[i].header.stamp = ros::Time::now();
                    joints_publishers[i]->publish(joint_state_robot[i]);
                }
            }
            for(int i=0; i<numrobots; i++)
            {
                br->sendTransform(rtransform[i]);
            }
            //loop for obstacles
            for(std::pair<std::string, geometry_msgs::TransformStamped> element : otransform)
                br->sendTransform(element.second);
        }

        //Wait until it's time for another iteration
        rate.sleep();
    }

    return 0;
}
