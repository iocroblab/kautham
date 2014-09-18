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
#include <ros/ros.h>
#include <Inventor/SoDB.h>
#include "std_msgs/String.h"
#include "../console/kauthamshell.h"
#include "kautham2/CloseProblem.h"
#include "kautham2/ProblemOpened.h"
#include "kautham2/OpenProblem.h"
#include "kautham2/OpenProblemStream.h"
#include "kautham2/CheckCollision.h"
#include "kautham2/SetRobotsConfig.h"
#include "kautham2/SetObstaclesConfig.h"
#include "kautham2/SetQuery.h"
#include "kautham2/SetInit.h"
#include "kautham2/SetGoal.h"
#include "kautham2/SetInitObs.h"
#include "kautham2/ClearSampleSet.h"
#include "kautham2/SetRobControls.h"
#include "kautham2/SetRobControlsStream.h"
#include "kautham2/SetDefaultRobControls.h"
#include "kautham2/SetObsControls.h"
#include "kautham2/SetObsControlsStream.h"
#include "kautham2/SetFixedObsControls.h"
#include "kautham2/SetPlannerByName.h"
#include "kautham2/SetPlanner.h"
#include "kautham2/SetPlannerStream.h"
#include "kautham2/SetPlannerParameter.h"
#include "kautham2/Solve.h"
#include "kautham2/GetPath.h"
#include "kautham2/AddRobot.h"
#include "kautham2/RemoveRobot.h"
#include "kautham2/AddObstacle.h"
#include "kautham2/RemoveObstacle.h"
#include "kautham2/AttachObstacle2RobotLink.h"
#include "kautham2/DetachObstacle.h"
#include "kautham2/Connect.h"
#include "kautham2/GetLastPlanComputationTime.h"
#include "kautham2/GetNumEdges.h"
#include "kautham2/GetNumVertices.h"


using namespace std;
using namespace Kautham;

std_msgs::String my_msg;
ros::Publisher chatter_pub;
kauthamshell* ksh;


bool srvCloseProblem(kautham2::CloseProblem::Request &req,
                     kautham2::CloseProblem::Response &res) {
    ksh->closeProblem();

    return true;
}


bool srvProblemOpened(kautham2::ProblemOpened::Request &req,
                      kautham2::ProblemOpened::Response &res) {
    res.response = ksh->problemOpened();

    return true;
}


bool srvOpenProblem(kautham2::OpenProblem::Request &req,
                    kautham2::OpenProblem::Response &res) {
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


bool srvOpenProblemStream(kautham2::OpenProblemStream::Request &req,
                          kautham2::OpenProblemStream::Response &res) {
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


bool srvCheckCollision(kautham2::CheckCollision::Request &req,
                       kautham2::CheckCollision::Response &res) {

    bool collisionFree;
    res.response = ksh->checkCollision(req.config,&collisionFree);
    res.collisionFree = collisionFree;

    return true;
}


bool srvSetRobotsConfig(kautham2::SetRobotsConfig::Request &req,
                       kautham2::SetRobotsConfig::Response &res) {

    res.response = ksh->setRobotsConfig(req.config);

    return true;
}


bool srvSetObstaclesConfig(kautham2::SetObstaclesConfig::Request &req,
                           kautham2::SetObstaclesConfig::Response &res) {

    res.response = ksh->setObstaclesConfig(req.config);

    return true;
}


bool srvSetQuery(kautham2::SetQuery::Request &req,
                 kautham2::SetQuery::Response &res) {
    res.response = ksh->setQuery(req.init,req.goal);

    return true;
}


bool srvSetInit(kautham2::SetInit::Request &req,
                kautham2::SetInit::Response &res) {
    res.response = ksh->setInit(req.init);

    return true;
}


bool srvSetGoal(kautham2::SetGoal::Request &req,
                kautham2::SetGoal::Response &res) {
    res.response = ksh->setGoal(req.goal);

    return true;
}


bool srvSetInitObs(kautham2::SetInitObs::Request &req,
                   kautham2::SetInitObs::Response &res) {
    res.response = ksh->setInitObs(req.initObs);

    return true;
}


bool srvClearSampleSet(kautham2::ClearSampleSet::Request &req,
                       kautham2::ClearSampleSet::Response &res) {
    res.response = ksh->clearSampleSet();

    return true;
}


bool srvSetRobControls(kautham2::SetRobControls::Request &req,
                       kautham2::SetRobControls::Response &res) {
    res.response = ksh->setRobControls(req.controls,req.init,req.goal);

    return true;
}


bool srvSetRobControlsStream(kautham2::SetRobControlsStream::Request &req,
                             kautham2::SetRobControlsStream::Response &res) {
    filebuf fb;
    fb.open(req.controls.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setRobControls(&is,req.init,req.goal);

    return true;
}


bool srvSetDefaultRobControls(kautham2::SetDefaultRobControls::Request &req,
                              kautham2::SetDefaultRobControls::Response &res) {
    res.response = ksh->setDefaultRobControls(req.init,req.goal);

    return true;
}


bool srvSetObsControls(kautham2::SetObsControls::Request &req,
                       kautham2::SetObsControls::Response &res) {
    res.response = ksh->setObsControls(req.controls,req.initObs);

    return true;
}


bool srvSetObsControlsStream(kautham2::SetObsControlsStream::Request &req,
                             kautham2::SetObsControlsStream::Response &res) {
    filebuf fb;
    fb.open(req.controls.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setObsControls(&is,req.initObs);

    return true;
}


bool srvSetFixedObsControls(kautham2::SetFixedObsControls::Request &req,
                            kautham2::SetFixedObsControls::Response &res) {
    res.response = ksh->setFixedObsControls();

    return true;
}


bool srvSetPlannerByName(kautham2::SetPlannerByName::Request &req,
                         kautham2::SetPlannerByName::Response &res) {
    res.response = ksh->setPlannerByName(req.name);

    return true;
}


bool srvSetPlanner(kautham2::SetPlanner::Request &req,
                   kautham2::SetPlanner::Response &res) {
    res.response = ksh->setPlanner(req.planner);

    return true;
}


bool srvSetPlannerStream(kautham2::SetPlannerStream::Request &req,
                         kautham2::SetPlannerStream::Response &res) {
    filebuf fb;
    fb.open(req.planner.c_str(),ios::in);
    istream is(&fb);

    res.response = ksh->setPlanner(&is);

    return true;
}


bool srvSetPlannerParameter(kautham2::SetPlannerParameter::Request &req,
                            kautham2::SetPlannerParameter::Response &res) {
    res.response = ksh->setPlannerParameter(req.parameter,req.value);

    return true;
}


bool srvSolve(kautham2::Solve::Request &req,
              kautham2::Solve::Response &res) {
    res.response = ksh->solve(std::cout);

    return true;
}


bool srvGetPath(kautham2::GetPath::Request &req,
                kautham2::GetPath::Response &res) {
    ostringstream oss;
    if (ksh->getPath(oss)) {
        vector < vector < KthReal > > path;
        istringstream iss(oss.str());
        for (string str; getline(iss,str); ) {
            vector < KthReal > conf;
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
        for (int i = 0; i < path.size(); ++i) {
            res.response[i].v.resize(path.at(i).size());
            for (int j = 0; j < path.at(i).size(); ++j) {
                res.response[i].v[j] = path.at(i).at(j);
            }
        }
    }

    return true;
}


bool srvAddRobot(kautham2::AddRobot::Request &req,
                            kautham2::AddRobot::Response &res) {
    vector< vector<KthReal> > limits, mapMatrix;
    limits.resize(req.limits.size());
    for (int i = 0; i < limits.size(); ++i) {
        limits[i].resize(req.limits[i].v.size());
        for (int j = 0; j < limits[i].size(); ++j) {
            limits[i][j] = req.limits[i].v[j];
        }
    }
    mapMatrix.resize(req.mapMatrix.size());
    for (int i = 0; i < mapMatrix.size(); ++i) {
        mapMatrix[i].resize(req.mapMatrix[i].v.size());
        for (int j = 0; j < mapMatrix[i].size(); ++j) {
            mapMatrix[i][j] = req.mapMatrix[i].v[j];
        }
    }
    res.response = ksh->addRobot(req.robot,req.scale,req.home,
                                 limits,mapMatrix,req.offMatrix);

    return true;
}


bool srvRemoveRobot(kautham2::RemoveRobot::Request &req,
                            kautham2::RemoveRobot::Response &res) {
    res.response = ksh->removeRobot(req.index);

    return true;
}


bool srvAddObstacle(kautham2::AddObstacle::Request &req,
                            kautham2::AddObstacle::Response &res) {
    res.response = ksh->addObstacle(req.obstacle,req.scale,req.home);
    return true;
}


bool srvRemoveObstacle(kautham2::RemoveObstacle::Request &req,
                            kautham2::RemoveObstacle::Response &res) {
    res.response = ksh->removeObstacle(req.index);

    return true;
}


bool srvAttachObstacle2RobotLink(kautham2::AttachObstacle2RobotLink::Request &req,
                            kautham2::AttachObstacle2RobotLink::Response &res) {
    res.response = ksh->attachObstacle2RobotLink(req.robot,req.link,req.obs);

    return true;
}


bool srvDetachObstacle(kautham2::DetachObstacle::Request &req,
                            kautham2::DetachObstacle::Response &res) {
    res.response = ksh->detachObstacle(req.obs);

    return true;
}


bool srvConnect(kautham2::Connect::Request &req,
                            kautham2::Connect::Response &res) {
    res.response = ksh->connect(req.sample1,req.sample2);

    return true;
}


bool srvGetLastPlanComputationTime(kautham2::GetLastPlanComputationTime::Request &req,
                            kautham2::GetLastPlanComputationTime::Response &res) {
    res.time = ksh->getLastPlanComputationTime();

    return true;
}


bool srvGetNumEdges(kautham2::GetNumEdges::Request &req,
                            kautham2::GetNumEdges::Response &res) {
    res.num = ksh->getNumEdges();

    return true;
}


bool srvGetNumVertices(kautham2::GetNumVertices::Request &req,
                            kautham2::GetNumVertices::Response &res) {
    res.num = ksh->getNumVertices();

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

    ros::spin();

    return 0;
}






