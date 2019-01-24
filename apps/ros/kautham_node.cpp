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
#include <kautham/FindIK.h>


using namespace std;
using namespace Kautham;

std_msgs::String my_msg;
ros::Publisher chatter_pub;
kauthamshell* ksh;


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

    bool collisionFree;
    res.response = ksh->checkCollision(req.config,&collisionFree);
    res.collisionFree = res.response&&collisionFree;

    return true;
}

bool srvCheckCollisionRob(kautham::CheckCollision::Request &req,
                          kautham::CheckCollision::Response &res) {

    //    for (unsigned int i = 0; i < req.config.size(); ++i) {
    //        cout << req.config.at(i) << " ";
    //    }
    //    cout << endl;

    //    std::vector<unsigned> ObstColl;
    //    res.response = ksh->checkCollisionRob(req.config,&ObstColl);
    //    res.collObjs = ObstColl;

    //    return true;


    for (unsigned int i = 0; i < req.config.size(); ++i) {
        cout << req.config.at(i) << " ";
    }
    cout << endl;
    std::pair< std::pair<int, int> , std::pair<int,int> > colliding_elements;
    bool collisionFree;
    res.response = ksh->checkCollision(req.config,&collisionFree, &colliding_elements);
    res.collisionFree = res.response&&collisionFree;
    res.collObj = colliding_elements.first.second;
    return true;

}

bool srvCheckCollisionObs(kautham::CheckCollision::Request &req,
                                kautham::CheckCollision::Response &res) {

    std::vector<unsigned> ObstColl;
    std::string msg;
    res.response = ksh->checkCollisionObs(req.index, &ObstColl, &msg);
    res.collObjs = ObstColl;
    res.msg = msg;
//    res.collObj = colObj.first.second;

    return true;
}

bool srvSetRobotsConfig(kautham::SetRobotsConfig::Request &req,
                       kautham::SetRobotsConfig::Response &res) {

    res.response = ksh->setRobotsConfig(req.config);

    return true;
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
    res.response = ksh->addObstacle(req.obstacle,req.scale,req.home);

    return true;
}


bool srvRemoveObstacle(kautham::RemoveObstacle::Request &req,
                            kautham::RemoveObstacle::Response &res) {
    res.response = ksh->removeObstacle(req.index);

    return true;
}


bool srvAttachObstacle2RobotLink(kautham::AttachObstacle2RobotLink::Request &req,
                            kautham::AttachObstacle2RobotLink::Response &res) {
    res.response = ksh->attachObstacle2RobotLink(req.robot,req.link,req.obs);

    return true;
}


bool srvDetachObstacle(kautham::DetachObstacle::Request &req,
                            kautham::DetachObstacle::Response &res) {
    res.response = ksh->detachObstacle(req.obs);

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

    res.response = ksh->setObstaclePos(req.index, req.setPos);

    return true;
}

bool srvGetObstaclPos(kautham::ObsPos::Request &req,
                            kautham::ObsPos::Response &res) {
    (void) req;//unused
    res.getPos = ksh->getObstaclePos(req.index);

    return true;
}

bool srvSetRobPos(kautham::ObsPos::Request &req,
                            kautham::ObsPos::Response &res) {
    res.response = ksh->setRobPos(req.index, req.setPos);

    return true;
}

bool srvGetRobPos(kautham::ObsPos::Request &req,
                            kautham::ObsPos::Response &res) {
    std::vector<float> robPos;
    res.response = ksh->getRobPos(req.index, robPos);
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
    ros::ServiceServer service36 = n.advertiseService("kautham_node/CheckCollisionObs",srvCheckCollisionObs);
    ros::ServiceServer service37 = n.advertiseService("kautham_node/CheckCollisionRob",srvCheckCollisionRob);
    ros::ServiceServer service38 = n.advertiseService("kautham_node/FindIK",srvFindIK);
    ros::ServiceServer service39 = n.advertiseService("kautham_node/SetRobotPos",srvSetRobPos);
    ros::ServiceServer service40 = n.advertiseService("kautham_node/GetRobotPos",srvGetRobPos);




    ros::spin();

    return 0;
}







