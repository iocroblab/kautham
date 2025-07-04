/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Pol Ramon Canyameres, Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#include <iostream>

#include <kautham/kauthamshell.h>
#include <kautham/planner/omplg/omplplanner.h>
#include <kautham/planner/omplconstr/omplconstrplanner.hpp>
#include <kautham/util/kthutil/kauthamexception.h>
#include <kautham/problem/problem.h>
#include <kautham/util/libkin/ivkinyumi.h>
#include <kautham/util/libttg/Trajectory.hpp>
#include <kautham/sampling/robconf.h>
#include <kautham/problem/robot.h>

#include <Inventor/SoDB.h>
#include <Inventor/nodes/SoNode.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace Kautham {
    kauthamshell::kauthamshell() {
        memPtr_ = NULL;
        
        if (SoNode::getClassTypeId() == SoType::badType()) {
            // SoDB::init() has NOT been called
            SoDB::init();
        }
    }

    kauthamshell::~kauthamshell(){
        if (problemOpened()) closeProblem();
    }

    bool kauthamshell::problemOpened() {
        return memPtr_;
    }

    void kauthamshell::closeProblem() {
        delete (Problem*)memPtr_;
        memPtr_ = NULL;
    }


    bool kauthamshell::openProblem(istream *inputfile, vector <string> def_path) {
        try {
            if (problemOpened()) closeProblem();
            Problem *const problem = new Problem();
            memPtr_ = problem;
            if (problem->setupFromFile(inputfile,def_path)) {
                problem->getPlanner()->setInitSamp(problem->getSampleSet()->getSampleAt(0));
                problem->getPlanner()->setGoalSamp(problem->getSampleSet()->getSampleAt(1));
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        closeProblem();
        return false;
    }


    bool kauthamshell::openProblem(string problemfilename, vector <string> def_path) {
        try {
            if (problemOpened()) closeProblem();
            Problem *const problem = new Problem();
            memPtr_ = problem;
            if (problem->setupFromFile(problemfilename,def_path)) {
                problem->getPlanner()->setInitSamp(problem->getSampleSet()->getSampleAt(0));
                problem->getPlanner()->setGoalSamp(problem->getSampleSet()->getSampleAt(1));
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        closeProblem();
        return false;
    }




    bool kauthamshell::checkCollision(vector<double> smpcoords, bool *collisionFree, std::string *msg, std::pair< std::pair<int, string> , std::pair<int,int> > *colliding_elements) {
        Sample *smp = NULL;

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (smpcoords.size() != problem->wSpace()->getNumRobControls()) {
                cout << "Sample has dimension " << smpcoords.size() << " and should have dimension "
                     << problem->wSpace()->getNumRobControls() << endl;
                return false;
            }
            smp = new Sample(problem->wSpace()->getNumRobControls());

            if (smp->setCoords(smpcoords)) {
                *collisionFree = !problem->wSpace()->collisionCheck(smp,msg,colliding_elements);
                if(msg->empty()) {
                    std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
                } else {
                    std::cout<<"Response for collision checking service is: "<<*msg<<std::endl;
                }

                delete smp;
                return true;
            } else {
                cout << "Sample has dimension " << smpcoords.size() << " and should have dimension "
                     << problem->wSpace()->getNumRobControls() << endl;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        delete smp;
        return false;
    }



    double kauthamshell::cumDistCheck(std::vector<double> smpcoords) {
        Sample *smp = NULL;
        double value=0.;

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (smpcoords.size() != problem->wSpace()->getNumRobControls()) {
                cout << "Sample has dimension " << smpcoords.size() << " and should have dimension "
                     << problem->wSpace()->getNumRobControls() << endl;
                return false;
            }
            smp = new Sample(problem->wSpace()->getNumRobControls());

            value = problem->wSpace()->cumDistanceCheck(smp);
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        delete smp;

        return value;
    }

    //Sets the robot configuration according to the given sample coordinates (control)
    //The configuration is returned in the config parameter
    bool kauthamshell::setRobotsConfig(vector<double> smpcoords, std::vector<RobConf> &config) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            Sample *smp = new Sample(problem->wSpace()->getNumRobControls());
            if (smp->setCoords(smpcoords)) {
                problem->wSpace()->moveRobotsTo(smp);
                config = smp->getMappedConf();

                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::setRobotsConfig(vector<double> smpcoords) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            Sample *smp = new Sample(problem->wSpace()->getNumRobControls());
            if (smp->setCoords(smpcoords)) {
                problem->wSpace()->moveRobotsTo(smp);

                //EUROC
                /*
                std::cout<<"Robot moved to: (";
                for(unsigned i=0; i<smpcoords.size(); i++)
                {
                    std::cout<<smpcoords[i]<<" ";
                }
                std::cout<<std::endl;
                int s = problem->wSpace()->getRobot(0)->getAttachedObject()->size();
                std::cout<<"Number of attached objets = "<<s<<std::endl;
                list<attObj>::iterator it = problem->wSpace()->getRobot(0)->getAttachedObject()->begin();
                for( it = problem->wSpace()->getRobot(0)->getAttachedObject()->begin();
                     it != problem->wSpace()->getRobot(0)->getAttachedObject()->end();
                     ++it)
                {
                    float x,y,z;
                    string obsname = (*it).obs->getName();

                    x = (*it).obs->getLink(0)->getElement()->getPosition()[0];
                    y = (*it).obs->getLink(0)->getElement()->getPosition()[1];
                    z = (*it).obs->getLink(0)->getElement()->getPosition()[2];
                    std::cout<<"Object "<<obsname<<" is at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
                }
                */
                //EUROC

                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setRobotsConfigByIndexSample(const std::string& _sample_type, const unsigned int _sample_index, std::vector<RobConf>& sample_config_) {
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened." << std::endl;
                return false;
            }

            Problem* const problem = static_cast<Problem*>(memPtr_);
            Sample* requested_sample = nullptr;

            if (_sample_type == "path") {
                if (!problem->getPlanner()->isSolved()) {
                    std::cout << "Invalid path sample request, the problem is not solved." << std::endl;
                    return false;
                }
                std::vector<Sample*>* path_samples = problem->getPlanner()->getPath();
                if (!path_samples || path_samples->empty() || _sample_index >= path_samples->size()) {
                    std::cout << "Invalid path_samples or index." << std::endl;
                    return false;
                }
                requested_sample = path_samples->at(_sample_index);
            } else if (_sample_type == "init") {
                requested_sample = problem->getPlanner()->getInitSample();
            } else if (_sample_type == "goal") {
                requested_sample = problem->getPlanner()->getGoalSample();
            } else {
                std::cout << "Unknown sample_type: " << _sample_type << std::endl;
                return false;
            }

            if (!requested_sample) {
                std::cout << "Requested sample is null." << std::endl;
                return false;
            }
    
            problem->wSpace()->moveRobotsTo(requested_sample);
            sample_config_ = requested_sample->getMappedConf();
            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setObstaclesConfig(vector<double> smpcoords) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            Sample *smp = new Sample(problem->wSpace()->getNumObsControls());
            if (smp->setCoords(smpcoords)) {
                problem->wSpace()->moveObstaclesTo(smp);
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setQuery(vector<double> init, vector<double> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            int d = problem->wSpace()->getNumRobControls();
            SampleSet *samples = problem->getSampleSet();
            samples->clear();

            string msg_init, msg_goal;

            //init
            Sample *smp = new Sample(d);
            smp->setCoords(init);
            if(problem->wSpace()->collisionCheck(smp, &msg_init)) {
                std::cout<<"Init in collision: (";
                for(unsigned k=0;k<init.size();k++) std::cout<<init[k]<<" ";
                std::cout<<std::endl;
                std::cout<<msg_init<<std::endl;
                return false;
            }
            samples->add(smp);

            //goal
            smp = new Sample(d);
            smp->setCoords(goal);
            if(problem->wSpace()->collisionCheck(smp, &msg_goal)) {
                std::cout<<"Goal in collision: (";
                for(unsigned k=0;k<goal.size();k++) std::cout<<goal[k]<<" ";
                std::cout<<std::endl;
                std::cout<<msg_goal<<std::endl;
                return false;
            }
            samples->add(smp);

            problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

            return true;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::setQuery(const std::vector<std::string>& _control_names, const std::vector<double>& _init, const std::vector<double>& _goal) {
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            // Error check: Ensure control_names and the new init have the same size, iff new init is requested:
            if (!_init.empty() && (_control_names.size() != _init.size())) {
                std::cout << "Error: Requested control_names and init must have the same size." << std::endl;
                return false;
            }

            // Error check: Ensure control_names and the new goal have the same size, iff new goal is requested:
            if (!_goal.empty() && (_control_names.size() != _goal.size())) {
                std::cout << "Error: Requested control_names and goal must have the same size." << std::endl;
                return false;
            }
            
            // Get the information of the opened problem:
            Problem *const problem = (Problem*)memPtr_;
            std::vector<std::string> prob_control_names =  problem->wSpace()->getRobControlsNames();
            
            // Save the original samples (maybe only a single control will be set) and clear the stored samples:
            SampleSet *samples = problem->getSampleSet();
            Sample *init_sample = new Sample(*samples->getSampleAt(0));
            Sample *goal_sample = new Sample(*samples->getSampleAt(1));
            samples->clear();
            
            std::vector<double> updated_init_coord = init_sample->getCoords();
            std::vector<double> updated_goal_coord = goal_sample->getCoords();

            // Update original coordinates based on the control names and the new coordinates, iff they are requested:
            for (size_t i = 0; i < _control_names.size(); ++i) {
                const std::string& name = _control_names[i];
                auto it = std::find(prob_control_names.begin(), prob_control_names.end(), name);
                if (it != prob_control_names.end()) {
                    size_t index = std::distance(prob_control_names.begin(), it);
                    if (!_init.empty()) {
                        updated_init_coord[index] = _init[i];
                    }
                    if (!_goal.empty()) {
                        updated_goal_coord[index] = _goal[i];
                    }
                } else {
                    std::cout << "Error: Control name '" << name << "' not found in prob_control_names." << std::endl;
                    return false;
                }
            }

            std::string msg_init, msg_goal;

            // Set the new Init sample:
            if (!_init.empty()) {
                init_sample->setCoords(updated_init_coord);
                if(problem->wSpace()->collisionCheck(init_sample, &msg_init)) {
                    std::cout << "Init in collision: ";
                    for (unsigned k = 0; k < updated_init_coord.size(); k++) {
                        std::cout << updated_init_coord[k] << " ";
                    }
                    std::cout << std::endl;
                    std::cout << msg_init << std::endl;
                    return false;
                }
            }
            samples->add(init_sample);  // Add init_sample to the KauthamSampleSet.
            problem->getPlanner()->setInitSamp(init_sample);    // Set the init_sample as the init to the planner.

            // Goal sample:
            if (!_goal.empty()) {
                goal_sample->setCoords(updated_goal_coord);
                if(problem->wSpace()->collisionCheck(goal_sample, &msg_goal)) {
                    std::cout << "Goal in collision: ";
                    for (unsigned k = 0; k < updated_goal_coord.size(); k++) {
                        std::cout << updated_goal_coord[k] << " ";
                    }
                    std::cout << std::endl;
                    std::cout << msg_goal << std::endl;
                    return false;
                }
            }
            samples->add(goal_sample);  // Add goal_sample to the KauthamSampleSet.
            problem->getPlanner()->setGoalSamp(goal_sample);    // Set the goal_sample as the goal to the planner.

            return true;
        } catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } catch (const exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } catch(...) {
            std::cout << "Something is wrong when setting the query by name." << std::endl;
        }

        return false;
    }



    bool kauthamshell::setInit(vector<double> init) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            int d = problem->wSpace()->getNumRobControls();
            SampleSet *samples = problem->getSampleSet();
            Sample *goal = new Sample(samples->getSampleAt(1));
            samples->clear();

            //init
            Sample *smp = new Sample(d);
            smp->setCoords(init);
            if(problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            //goal
            samples->add(goal);

            problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

            return true;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setGoal(vector<double> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            int d = problem->wSpace()->getNumRobControls();
            SampleSet *samples = problem->getSampleSet();
            Sample *init = new Sample(samples->getSampleAt(0));
            samples->clear();

            //init
            samples->add(init);

            //goal
            Sample *smp = new Sample(d);
            smp->setCoords(goal);
            if (problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

            return true;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setInitObs(vector<double> initObs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            int d = problem->wSpace()->getNumObsControls();
            Sample *smp = new Sample(d);
            smp->setCoords(initObs);
            problem->wSpace()->setInitObsSample(smp);
            return (!problem->wSpace()->collisionCheck(problem->getSampleSet()->getSampleAt(0)) &&
                    !problem->wSpace()->collisionCheck(problem->getSampleSet()->getSampleAt(1)));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::clearSampleSet() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            SampleSet *samples = problem->getSampleSet();
            Sample *init = new Sample(samples->getSampleAt(0));
            Sample *goal = new Sample(samples->getSampleAt(1));
            samples->clear();

            samples->add(init);
            samples->add(goal);

            problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

            return true;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setRobControls(istream *inputfile, vector<double> init, vector<double> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setRobotControls(inputfile)) return false;
            return (setQuery(init,goal));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setRobControlsNoQuery(string controlsFile) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;
            //cout << "\nsetRobControls - control file "<<controlsFile<< endl;
            //cout << "defPath size"<<problem->defPath.size()<< endl;
            //for(uint i=0; i<problem->defPath.size();i++)
            //    cout << "problem->defPath("<<i<<") " << problem->defPath.at(i) << endl;
            if(problem->findFile(controlsFile,problem->defPath))
            {
                cout << "setRobControlsNoquery - Setting control file: "<<controlsFile<< endl;
                if (!problem->setRobotControls(controlsFile)) return false;
                return true;
            }
            else {
                cout << "setRobControlsNoquery - Failed to find control file: "<<controlsFile<< endl;
                return false;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "setRobControlsNoquery: Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }
        return false;
    }


    bool kauthamshell::setRobControls(string controlsFile, vector<double> init, vector<double> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;
            //cout << "\nsetRobControls - control file "<<controlsFile<< endl;
            //cout << "defPath size"<<problem->defPath.size()<< endl;
            //for(uint i=0; i<problem->defPath.size();i++)
            //    cout << "problem->defPath("<<i<<") " << problem->defPath.at(i) << endl;
            if(problem->findFile(controlsFile,problem->defPath))
            {
                cout << "setRobControls - Setting control file: "<<controlsFile<< endl;
                if (!problem->setRobotControls(controlsFile)) return false;
                return (setQuery(init,goal));
            }
            else {
                cout << "setRobControls - Failed to find control file: "<<controlsFile<< endl;
                return false;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "setRobControls: Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }
        return false;
    }


    bool kauthamshell::setDefaultRobControls(vector<double> init, vector<double> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setDefaultRobotControls()) return false;
            return (setQuery(init,goal));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setObsControls(istream *inputfile, vector<double> initObs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setObstacleControls(inputfile)) return false;
            return (setInitObs(initObs));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setObsControls(string controlsFile, vector<double> initObs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setObstacleControls(controlsFile)) return false;
            return (setInitObs(initObs));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setPlannerByName(string name) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (problem->createPlanner(name)) {
                problem->getPlanner()->setInitSamp(problem->getSampleSet()->getSampleAt(0));
                problem->getPlanner()->setGoalSamp(problem->getSampleSet()->getSampleAt(1));
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setPlanner(istream *inputfile) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            problem->resetPlanner();
            if (problem->createPlannerFromFile(inputfile)) {
                problem->getPlanner()->setInitSamp(problem->getSampleSet()->getSampleAt(0));
                problem->getPlanner()->setGoalSamp(problem->getSampleSet()->getSampleAt(1));
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setPlanner(string problemfilename) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (problem->createPlannerFromFile(problemfilename))  {
                problem->getPlanner()->setInitSamp(problem->getSampleSet()->getSampleAt(0));
                problem->getPlanner()->setGoalSamp(problem->getSampleSet()->getSampleAt(1));
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setPlannerParameter(string parameter, string value) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            return (problem->getPlanner()->setParametersFromString(parameter+"|"+value));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setFixedObsControls() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setFixedObstacleControls()) return false;
            vector<double> coords;
            coords.resize(0);
            return (setInitObs(coords));
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::solve() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            return problem->getPlanner()->solveAndInherit();
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    double kauthamshell::getLastPlanComputationTime() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1.0;
            }

            Problem *const problem = (Problem*)memPtr_;
            Planner *planner = problem->getPlanner();
            if (planner) {
                switch ((int)planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER:
                    return ((omplplanner::omplPlanner*)planner)->SimpleSetup()->getLastPlanComputationTime();
                    break;
                case OMPLCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#endif
                case NOFAMILY:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                default:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                }
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return -1.0;
    }


    int kauthamshell::getNumEdges() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Problem *const problem = (Problem*)memPtr_;
            Planner *planner = problem->getPlanner();
            if (planner) {
                switch ((int)planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)planner)->ss->getSpaceInformation()));
                    ((omplplanner::omplPlanner*)planner)->ss->getPlanner()->getPlannerData(*pdata);
                    return pdata->numEdges();
                    break;
                }
                case OMPLCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#endif
                case NOFAMILY:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                default:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                }
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return -1;
    }


    int kauthamshell::getNumVertices() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Problem *const problem = (Problem*)memPtr_;
            Planner *const planner = problem->getPlanner();
            if (planner) {
                switch ((int)planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)planner)->ss->getSpaceInformation()));
                    ((omplplanner::omplPlanner*)planner)->ss->getPlanner()->getPlannerData(*pdata);
                    return pdata->numVertices();
                    break;
                }
                case OMPLCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#endif
                case NOFAMILY:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                default:
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                }
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return -1;
    }


    bool kauthamshell::connect(vector<double> smpcoords1, vector<double> smpcoords2) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            const unsigned dim = problem->getDimension();
            if (smpcoords1.size() != dim || smpcoords2.size() != dim) {
                cout << "The samples should have dimension " << dim << endl;
                return false;
            }

            Planner *planner = problem->getPlanner();
            if (planner) {
                Sample *fromSample = new Sample(dim);
                fromSample->setCoords(smpcoords1);
                Sample *toSample = new Sample(dim);
                toSample->setCoords(smpcoords2);

                switch ((int)planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER: {
                    cout << "Not implemented for IOC planners" << endl;
                    break;
                }
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ((omplplanner::omplPlanner*)planner)->SimpleSetup()->setup();

                    ob::ScopedState<ob::CompoundStateSpace> fromState(((omplplanner::omplPlanner*)planner)->getSpace());
                    ((omplplanner::omplPlanner*)planner)->smp2omplScopedState(fromSample,&fromState);

                    ob::ScopedState<ob::CompoundStateSpace> toState(((omplplanner::omplPlanner*)planner)->getSpace());
                    ((omplplanner::omplPlanner*)planner)->smp2omplScopedState(toSample,&toState);

                    bool connected = ((ob::MotionValidator*)((ob::SpaceInformation*)((omplplanner::omplPlanner*)
                                                                                     planner)->
                                                             SimpleSetup()->getSpaceInformation().get())->
                                      getMotionValidator().get())->checkMotion(fromState.get(),toState.get());
                    if (connected) {
                        cout << "The samples can be connected." << endl;
                        return true;
                    } else {
                        cout << "The samples can not be connected." << endl;
                    }
                    break;
                }
                case OMPLCPLANNER: {
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
                }
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER: {
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
                }
#endif
#endif
                case NOFAMILY: {
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                }
                default: {
                    cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                    break;
                }
                }
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::solve(ostream &graphVizPlannerDataFile) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (problem->getPlanner()->getFamily()==OMPLPLANNER) {
                if (problem->getPlanner()->solveAndInherit()) {

                    omplplanner::omplPlanner *p = (omplplanner::omplPlanner *)problem->getPlanner();

                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(p->ss->getSpaceInformation()));

                    p->ss->getPlanner()->getPlannerData(*pdata);
                    pdata->printGraphviz(graphVizPlannerDataFile);

                    return true;
                }
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::getPath(ostream &path) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            //cout<<"getPath getPath getPath getPath getPath getPath"<<endl;
            //cout<<"problem->getPlanner()->getFamily() = "<<problem->getPlanner()->getFamily()<<endl;
            if (problem->getPlanner()->getFamily()==OMPLPLANNER) {
                cout<<"OMPLPLANNER"<<endl;
                if (problem->getPlanner()->solveAndInherit()) {
                    ((omplplanner::omplPlanner*)problem->getPlanner())->SimpleSetup()->
                            getSolutionPath().printAsMatrix(path);
                    cout<<((ostringstream&)path).str()<<endl;
                    return true;
                }
            }
            else {
              if (problem->getPlanner()->getFamily()==IOCPLANNER) {
                cout<<"IOCPLANNER"<<endl;
                if (problem->getPlanner()->solveAndInherit()) {
                  //in planner.h: vector<Sample*>* getPath();
                  //in sample.h: std::vector<double>& getCoords();
                  vector<Sample*>* p = problem->getPlanner()->getPath();
                  //cout<<"p->size() = "<<p->size()<<endl;
                  for(unsigned int i=0; i<p->size(); i++)
                  {
                    std::vector<RobConf> rc = p->at(i)->getMappedConf();
                    for(unsigned int j=0; j<rc.size();j++)
                    {
                      path<<rc[j].getSE3().getPos()[0]<<" ";
                      path<<rc[j].getSE3().getPos()[1]<<" ";
                      path<<rc[j].getSE3().getPos()[2]<<" ";
                      path<<rc[j].getSE3().getOrient()[0]<<" ";
                      path<<rc[j].getSE3().getOrient()[1]<<" ";
                      path<<rc[j].getSE3().getOrient()[2]<<" ";
                      path<<rc[j].getSE3().getOrient()[3]<<" ";
                      for(unsigned int k=0; k<rc[j].getRn().getCoordinates().size();k++)
                      {
                        path<<rc[j].getRn().getCoordinates()[k]<<" ";
                      }
                      path<<endl;
                     }
                  }
                  //cout<<((ostringstream&)path).str()<<endl;

                  return true;
                }
              }
              else{
                 cout<<"ERROR in getPath. Function only prepared for IOCPLANNER or OMPLPLANNER planners"<<endl;
              }
          }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::getPath(std::vector<std::vector<double>> &path, std::vector<std::string> &requested_joint_names, bool _only_controlled) {
        
        path.clear();   // Make sure that your return only the solved path!
        
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            Problem* const problem = static_cast<Problem*>(memPtr_);
            auto plannerFamily = problem->getPlanner()->getFamily();

            std::vector<bool> requested_joints = problem->getRequestedIndexJoints(_only_controlled);

            requested_joint_names = problem->getRequestedJointNames(requested_joints);

            if (plannerFamily == OMPLPLANNER || plannerFamily == OMPLCONSTRPLANNER) {
                std::cout << "OMPLPLANNER || OMPLCONSTRPLANNER" << std::endl;
                if (problem->getPlanner()->isSolved()) {
                    std::vector<Sample*>* p = problem->getPlanner()->getPath();
                    
                    std::vector<double> waypoint;
                    for(unsigned int i=0; i<p->size(); i++)
                    {
                        waypoint.clear();
                        std::vector<RobConf> rc = p->at(i)->getMappedConf();
                        for(unsigned int j=0; j<rc.size();j++)
                        {
                            if (!_only_controlled) {
                                waypoint.push_back(rc[j].getSE3().getPos()[0]);
                                waypoint.push_back(rc[j].getSE3().getPos()[1]);
                                waypoint.push_back(rc[j].getSE3().getPos()[2]);
                                waypoint.push_back(rc[j].getSE3().getOrient()[0]);
                                waypoint.push_back(rc[j].getSE3().getOrient()[1]);
                                waypoint.push_back(rc[j].getSE3().getOrient()[2]);
                                waypoint.push_back(rc[j].getSE3().getOrient()[3]);
                            }

                            for(unsigned int k=0; k<rc[j].getRn().getCoordinates().size();k++)
                            {
                                if (requested_joints[k]) {
                                    waypoint.push_back(rc[j].getRn().getCoordinates()[k]);
                                }
                            }
                        }
                        path.push_back(waypoint);
                    }
                    return true;
                }
            }

            else if (plannerFamily == IOCPLANNER) { // Not tested!
                std::cout << "IOCPLANNER" << std::endl;
                if (problem->getPlanner()->isSolved()) {
                    auto samples = problem->getPlanner()->getPath();
                    
                    // Clear and populate the path vector
                    path.clear();
                    for (auto& sample : *samples) {
                        auto mappedConfs = sample->getMappedConf();
                        for (auto& conf : mappedConfs) {
                            std::vector<double> stateData;

                            // Append SE3 position
                            auto pos = conf.getSE3().getPos();
                            stateData.insert(stateData.end(), pos.begin(), pos.end());

                            // Append SE3 orientation
                            auto orient = conf.getSE3().getOrient();
                            stateData.insert(stateData.end(), orient.begin(), orient.end());

                            // Append Rn coordinates
                            auto coordinates = conf.getRn().getCoordinates();
                            stateData.insert(stateData.end(), coordinates.begin(), coordinates.end());

                            // Add this state to the path
                            path.push_back(stateData);
                        }
                    }
                    return true;
                }
            } 
            else {
                std::cout << "ERROR in getPath: Unsupported planner family." << std::endl;
            }
        } 
        catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } 
        catch (const std::exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } 
        catch (...) {
            std::cout << "An unexpected error occurred. Verify the problem formulation with Kautham2." << std::endl;
        }

        return false;
    }


    bool kauthamshell::getSolvedPathNumSamples(unsigned int& num_samples_) {
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            Problem* const problem = static_cast<Problem*>(memPtr_);
            if (problem->getPlanner()->isSolved()) {
                std::vector<Sample*>* path_samples = problem->getPlanner()->getPath();
                num_samples_ = path_samples->size();
                return true;
            }
        } 
        catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } 
        catch (const std::exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } 
        catch (...) {
            std::cout << "An unexpected error occurred. Verify the problem formulation with Kautham2." << std::endl;
        }

        return false;
    }


    bool kauthamshell::computeTrajecotry(std::vector<std::vector<double>> &path, std::vector<double> &desired_max_velocity, std::vector<double> &desired_max_acceleration, double max_path_deviation, double freq, 
                                            std::vector<std::vector<double>> &traj_positions, std::vector<std::vector<double>> &traj_velocities, std::vector<double> & traj_time_from_start)
    {
        
        // 1) INITIAL CHECKS:
        
        // Ensure path is not empty
        if (path.empty()) {
            std::cerr << "Error: Path is empty." << std::endl;
            return false;
        }

        // Get degrees of freedom (dof) from the first waypoint
        size_t dof = path[0].size();

        // Ensure all waypoints in the path have the same size
        for (const auto& waypoint : path) {
            if (waypoint.size() != dof) {
                std::cerr << "Error: All waypoints must have the same number of dimensions." << std::endl;
                return false;
            }
        }

        // Check if desired_max_velocity and ratio_acceleration have the correct size
        if (desired_max_velocity.size() != dof) {
            std::cerr << "Error: desired_max_velocity size (" << desired_max_velocity.size()
                    << ") does not match the degrees of freedom (" << dof << ")." << std::endl;
            return false;
        }

        if (desired_max_acceleration.size() != dof) {
            std::cerr << "Error: desired_max_acceleration size (" << desired_max_acceleration.size()
                    << ") does not match the degrees of freedom (" << dof << ")." << std::endl;
            return false;
        }

        // 2) PREPARE THE INPUT:

        // List to store the waypoints (as Eigen::VectorXd)
        std::list<Eigen::VectorXd> waypoints;

        Eigen::VectorXd tmp_waypoint;
        for (const auto &waypoint : path) {
            // Convert each std::vector<double> to Eigen::VectorXd:
            tmp_waypoint = Eigen::VectorXd::Map(waypoint.data(), waypoint.size());
            waypoints.push_back(tmp_waypoint);
        }

        // Convert desired maximum X inputs to Eigen:
        Eigen::VectorXd maxVelocity = Eigen::VectorXd::Map(desired_max_velocity.data(), desired_max_velocity.size());
        Eigen::VectorXd maxAcceleration = Eigen::VectorXd::Map(desired_max_acceleration.data(), desired_max_acceleration.size());

        // 3) COMPUTE THE TRAJECTORY:
        Trajectory trajectory(Path(waypoints, max_path_deviation), maxVelocity, maxAcceleration);

        // 4) BUILD THE OUTPUT:
        if(trajectory.isValid()) {

            traj_positions.clear();
            traj_velocities.clear();
            traj_time_from_start.clear();

            double duration = trajectory.getDuration();
            std::vector<double> joint_positions(dof);
            std::vector<double> joint_velocities(dof);

            auto update_trajectory = [&](double time) {
                for (size_t q = 0; q < dof; q++) {
                    joint_positions[q] = trajectory.getPosition(time)[q];
                    joint_velocities[q] = trajectory.getVelocity(time)[q];
                }
                traj_positions.push_back(joint_positions);
                traj_velocities.push_back(joint_velocities);
                traj_time_from_start.push_back(time);
            };

            double last_time = 0;
            for (double t = 0.0; t < duration; t += (1.0 / freq)) {
                update_trajectory(t);
                last_time = t;
            }

            // Add the last point if not reached due to the FOR condition:
            if (std::abs(last_time - duration) > 1e-6) {
                update_trajectory(duration);
            }

            return true;
        } else {
            std::cerr << "Error: Cannot compute a valid trajectory." << std::endl;
            return false;
        }
    }

    bool kauthamshell::getProblemMaxJointVelocities(std::vector<double> &requested_max_joint_velocities, std::vector<std::string> &requested_joint_names, bool _only_controlled) {
        
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }
        
        Problem* const problem = static_cast<Problem*>(memPtr_);
        std::vector<bool> requested_joints = problem->getRequestedIndexJoints(_only_controlled);
        requested_max_joint_velocities = problem->getRequestedMaxJointVelocities(requested_joints);
        requested_joint_names = problem->getRequestedJointNames(requested_joints);
        return true;
    }


    bool kauthamshell::getTrajecotry(std::vector<double> &ratio_velocity, std::vector<double> &ratio_acceleration, double max_path_deviation, double freq,
                                        std::vector<std::string> &requested_joint_names, std::vector<std::vector<double>> &traj_positions, std::vector<std::vector<double>> &traj_velocities, std::vector<double> & traj_time_from_start,
                                        bool _only_controlled)
    {

        // Inside custom function:
        auto isInRange = [](const std::vector<double>& ratios) {
            for (double val : ratios) {
                if (val < 0.0 || val > 1.0) return false;
            }
            return true;
        };

        // Verify that ratio_velocity and ratio_acceleration are in [0, 1]
        if (!isInRange(ratio_velocity)) {
            std::cerr << "Error: ratio_velocity contains values outside the range [0, 1]." << std::endl;
            return false;
        }

        if (!isInRange(ratio_acceleration)) {
            std::cerr << "Error: ratio_acceleration contains values outside the range [0, 1]." << std::endl;
            return false;
        }

        try {

            std::vector<std::vector<double>> path;
            if (kauthamshell::getPath(path, requested_joint_names, _only_controlled)) {
                std::cout << "kauthamshell::getTrajecotry -> Path successfully computed." << std::endl;
                
                std::vector<double> requested_max_joint_velocities;
                bool max_vel_founded = getProblemMaxJointVelocities(requested_max_joint_velocities, requested_joint_names, _only_controlled);
                
                std::vector<double> desired_max_velocity(ratio_velocity.size());
                if (max_vel_founded) {
                    for (size_t i = 0; i < ratio_velocity.size(); ++i) {
                        desired_max_velocity[i] = ratio_velocity[i] * requested_max_joint_velocities[i];
                    }
                }

                std::vector<double> desired_max_acceleration(ratio_acceleration.size());
                for (size_t i = 0; i < ratio_acceleration.size(); ++i) {
                    desired_max_acceleration[i] = ratio_acceleration[i] * 5; // Hardcoded default: 5 m/s²
                }


                if (kauthamshell::computeTrajecotry(path, desired_max_velocity, desired_max_acceleration, max_path_deviation, freq, traj_positions, traj_velocities, traj_time_from_start)) {
                    std::cout << "kauthamshell::getTrajecotry -> Trajectory successfully computed." << std::endl;
                    return true;
                }
                std::cerr << "kauthamshell::getTrajecotry -> Failed to compute the trajectory." << std::endl;
                return false;
            }
            std::cerr << "kauthamshell::getTrajecotry -> Failed to compute the path." << std::endl;
            return false;
        } 
        catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } 
        catch (const std::exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } 
        catch (...) {
            std::cout << "An unexpected error occurred in getTrajectory()." << std::endl;
        }

        return false;
    }


    int kauthamshell::addRobot(string robFile, double scale, vector<double> home, vector<vector<double> > limits,
                               vector<vector<double> > mapMatrix, vector<double> offMatrix) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->addRobot2WSpace(robFile,scale,home,limits)) return (-1);
            int index = problem->wSpace()->getNumRobots()-1;
            Robot *rob = problem->wSpace()->getRobot(index);
            int numCntr = problem->wSpace()->getNumRobControls();
            int numDOF = 6+rob->getNumJoints();
            double **MapMatrix = new double*[numDOF];
            double *OffMatrix = new double[numDOF];
            for (int i = 0; i < numDOF; ++i) {
                OffMatrix[i] = offMatrix.at(i);
                MapMatrix[i] = new double[numCntr];
                for (int j = 0; j < numCntr; ++j) {
                    MapMatrix[i][j] = mapMatrix.at(i).at(j);
                }
            }
            rob->setMapMatrix(MapMatrix);
            rob->setOffMatrix(OffMatrix);

            return index;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return -1;
    }


    int kauthamshell::addObstacle(string obsFile, double scale, vector<double> home) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->addObstacle2WSpace(obsFile,scale,home)) return (-1);
            return problem->wSpace()->getNumObstacles()-1;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return -1;
    }

    bool kauthamshell::removeRobot(unsigned index) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (index < problem->wSpace()->getNumRobots()) {
                problem->wSpace()->removeRobot(index);
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::removeObstacle(string obsname) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;


            if(problem->getPlanner()->wkSpace()->getObstacle(obsname)==NULL)
            {
                cout << "Error in removeObstacle:" << obsname << " is not in the set of obstacles " << problem->getPlanner()->wkSpace()->getNumObstacles() << endl;
                return true;
            }
            else {
                problem->wSpace()->removeObstacle(obsname);
                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::attachObstacle2RobotLink(int robot, int link, std::string obs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            std::cout << "Attaching obstacle " << obs << " (" << problem->wSpace()->getObstacle(obs)->getName()
                      << ") to link " << link << " (" << problem->wSpace()->getRobot(robot)->getLink(link)->getName()
                      << ") of robot " << robot << " (" << problem->wSpace()->getRobot(robot)->getName()
                      << ")." << std::endl;

            float x,y,z;
            x = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[0];
            y = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[1];
            z = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<obs<<" at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;



            bool ret = problem->wSpace()->attachObstacle2RobotLink(robot,link,obs);
            x = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[0];
            y = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[1];
            z = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<obs<<" attached at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
            if (ret) {
                std::cout<<"attachfunction returned TRUE\n";
            } else {
                std::cout<<"attachfunction returned FALSE\n";
            }
            return (ret);
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    // New (most natural) implementation:
    bool kauthamshell::attachObstacle2RobotLink(const std::string& _robot_name, const std::string& _robot_link_name, const std::string& _obstacle_name) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            std::cout << "Attaching obstacle " << _obstacle_name << " (" << problem->wSpace()->getObstacle(_obstacle_name)->getName()
                      << ") to link " << _robot_link_name << " (" << problem->wSpace()->getRobot(_robot_name)->getLink(_robot_link_name)->getName()
                      << ") of robot " << _robot_name << " (" << problem->wSpace()->getRobot(_robot_name)->getName()
                      << ")." << std::endl;

            float x,y,z;
            // x = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[0];
            // y = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[1];
            // z = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[2];
            // std::cout<<"Object "<<_obstacle_name<<" at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;

            bool ret = problem->wSpace()->attachObstacle2RobotLink(_robot_name,_robot_link_name,_obstacle_name);
            x = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[0];
            y = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[1];
            z = problem->wSpace()->getObstacle(_obstacle_name)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<_obstacle_name<<" attached at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
            if (ret) {
                std::cout<<"attachfunction returned TRUE\n";
            } else {
                std::cout<<"attachfunction returned FALSE\n";
            }
            return (ret);
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::detachObstacle(const std::string& obs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }


            Problem *const problem = (Problem*)memPtr_;

            if(problem->wSpace()->getObstacle(obs)==NULL){
                cout << "Error: trying to detach a nonexisting object - " << obs << endl;
                return false;
            }

            bool ret = problem->wSpace()->detachObstacle(obs);
            float x,y,z;
            x = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[0];
            y = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[1];
            z = problem->wSpace()->getObstacle(obs)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<obs<<" detached at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;

            if (ret) {
                std::cout<<"detachfunction returned TRUE\n";
            } else {
                std::cout<<"detachfunction returned FALSE\n";
            }
            return (ret);
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }


    bool kauthamshell::setInterpolatePath(bool interpolate) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            if (problem->getPlanner()->getFamily() == OMPLPLANNER) {
                ((omplplanner::omplPlanner*)problem->getPlanner())->setInterpolate(interpolate);
                    return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::getInterpolatePath(bool &interpolate) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            if (problem->getPlanner()->getFamily() == OMPLPLANNER) {
                interpolate = ((omplplanner::omplPlanner*)problem->getPlanner())->getInterpolate();

                return true;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::motionPlanner(vector<double> init, vector<double> goal, string root){

        bool _solved = false;

        Problem *const problem = (Problem*)memPtr_;
        Planner* _planner;
        SampleSet* _samples;
        unsigned int _dim;
        //unsigned int _dim,_dimOMPL;

        _planner = problem->getPlanner();
        _samples = problem->getSampleSet();
        _dim = _samples->getSampleAt(0)->getDim();
        //_dimOMPL = 0;

        Sample* smp = new Sample(_dim);
        smp->setCoords(init);
        _samples->add(smp);
        _planner->setInitSamp(smp);

        smp = new Sample(_dim);
        smp->setCoords(goal);
        _samples->add(smp);
        _planner->setGoalSamp(smp);

        if (_planner->solveAndInherit()){
            _solved = true;

            og::SimpleSetupPtr ss = ((Kautham::omplplanner::omplPlanner*)_planner)->SimpleSetupPtr();
            std::vector< ob::State * >  pathstates = ss->getSolutionPath().getStates();
            ob::State *state;
            Sample *sample;

            int d = _planner->wkSpace()->getNumRobControls();

            vector < Sample* > setOfSamples;
            setOfSamples.clear();


            if(!pathstates.empty()){

                for (uint k = 0; k< pathstates.size();k++){
                    //create a sample
                    sample = new Sample(d);
                    //set a mapped configuration
                    sample->setMappedConf(_planner->initSamp()->getMappedConf());

                    //create a state from pathstates "k"
                    state = pathstates.at(k);
                    //convert the ompl state to kautham sample
                    ((Kautham::omplplanner::omplPlanner*)_planner)->omplState2smp(state,sample);
                    //adding the sample to the set of samples
                    setOfSamples.push_back(sample);


                }
            }

            //Taking from samples path
            ofstream out, out1, out2, out1S, out2S ;
            std::ostringstream sh, r1, r2;
            //out.open(root.c_str()+"Experiments/DatosGA2H/SolutionPlan.txt");
            out.open((root+"Experiments/DatosGA2H/SolutionPlan.txt").c_str());
            out1.open((root+"Experiments/DatosGA2H/R1SolutionPlan.txt").c_str());
            out2.open((root+"Experiments/DatosGA2H/R2SolutionPlan.txt").c_str());

            out1S.open((root+"Experiments/DatosGA2H/R1SolutionPlanS.txt").c_str());
            out2S.open((root+"Experiments/DatosGA2H/R2SolutionPlanS.txt").c_str());

            //std::cout<<"setOfSamples size = "<<setOfSamples.size()<<std::endl;

            for (uint i = 0; i < setOfSamples.size(); i++){

                vector < RobConf > configuration;
                configuration = setOfSamples.at(i)->getMappedConf();
                for (uint r = 0; r < configuration.size(); ++r) {
                    RobConf robotConfiguration = configuration.at(r);
                    uint SE3Dim = robotConfiguration.getSE3().getCoordinates().size();
                    uint RnDim = robotConfiguration.getRn().getCoordinates().size();


                    /*for (unsigned j = 0; j <  SE3Dim; j++) {
                        sh << robotConfiguration.getSE3().getCoordinates().at(j) << " ";
                    }*/

                    for (unsigned j = 0; j <  RnDim; j++) {
                        if((j != 6) && (j != 7) && (j != 12) && (j != 17) && (j != 22) && (j != 27))
                            sh << robotConfiguration.getRn().getCoordinates().at(j) << " ";
                    }


                    if (r == 1){

                        for (unsigned j = 0; j <  SE3Dim; j++) {
                            r1 << robotConfiguration.getSE3().getCoordinates().at(j) << " ";
                        }

                        for (unsigned j = 0; j <  RnDim; j++) {
                            r1 << robotConfiguration.getRn().getCoordinates().at(j) << " ";

                        }
                    }
                    else{
                        for (unsigned j = 0; j <  SE3Dim; j++) {
                            r2 << robotConfiguration.getSE3().getCoordinates().at(j) << " ";
                        }

                        for (unsigned j = 0; j <  RnDim; j++) {
                            r2 << robotConfiguration.getRn().getCoordinates().at(j) << " ";

                        }

                    }
                }
                sh<<std::endl;

                if(i == setOfSamples.size()-1){
                    r1<<std::endl;
                    r1<<1;
                    r1<<std::endl;
                    r1<<1;
                    r1<<std::endl;

                }
                else{
                    r1<<std::endl;
                    r1<<0;
                    r1<<std::endl;
                    r1<<0;
                    r1<<std::endl;
                }

                r2<<std::endl;
                r2<<0;
                r2<<std::endl;
                r2<<0;
                r2<<std::endl;
            }

            std::string s(sh.str());
            out << s;
            out.close();

            std::string s1(r1.str());
            out1 << s1;
            out1S<< s1;
            out1.close();
            out1S.close();

            std::string s2(r2.str());
            out2 << s2;
            out2S << s2;
            out2.close();
            out2S.close();


        }

        return _solved;

    }

    //Sets obstacle pose with orientation defined as axis-angle
    bool kauthamshell::setObstaclePos(string obsname, std::vector<double> pos){

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            if(problem->getPlanner()->wkSpace()->getObstacle(obsname)==NULL)
            {
                cout << "Error in setObstaclePos:" << obsname << " is not in the set of obstacles " << problem->getPlanner()->wkSpace()->getNumObstacles() << endl;
                return false;
            }

            // Convert from axis-angle to quaternion internal represtantation
            SE3Conf tmpC;
            SE3Conf::fromAxisToQuaternion(pos);
            tmpC.setCoordinates(pos);

            problem->getPlanner()->wkSpace()->getObstacle(obsname)->setHomePos(&tmpC);

            float x,y,z;
            x = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[0];
            y = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[1];
            z = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<obsname<<" at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
            float qx,qy,qz,qth;
            qx = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[0];
            qy = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[1];
            qz = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[2];
            qth = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[3];
            std::cout<<"Object "<<obsname<<" at quaternion ("<<qx<<","<<qy<<","<<qz<<","<<qth<<")"<<std::endl;

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    // Sets obstacle pose using Eigen:
    bool kauthamshell::setObstaclePos(const std::string& _obs_name, const Eigen::AffineCompact3d& _obs_pose) {
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            if (problem->getPlanner()->wkSpace()->getObstacle(_obs_name) == NULL) {
                std::cout << "Error in setObstaclePos:" << _obs_name << " is not in the set of obstacles " << problem->getPlanner()->wkSpace()->getNumObstacles() << std::endl;
                return false;
            }

            // Convert it to the internal represtantation, from Eigen to Kautham SE3 configuration:
            Kautham::SE3Conf tmp_config;
            tmp_config.setPose(_obs_pose);
            problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->setHomePos(&tmp_config);

            // Debug:
            // float x,y,z;
            // x = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getPosition()[0];
            // y = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getPosition()[1];
            // z = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getPosition()[2];
            // std::cout << "Object " << _obs_name << " at position (x,y,z) = (" << x << "," << y << "," << z << ")" << std::endl;
            // float qx,qy,qz,qw;
            // qx = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getOrientation()[0];
            // qy = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getOrientation()[1];
            // qz = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getOrientation()[2];
            // qw = problem->getPlanner()->wkSpace()->getObstacle(_obs_name)->getLink(0)->getElement()->getOrientation()[3];
            // std::cout << "Object " << _obs_name << " at quaternion (qx,qy,qz,qw) = (" << qx << "," << qy << "," << qz << "," << qw << ")" << std::endl;

            return true;

        } catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } catch (const exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } catch(...) {
            std::cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation." << std::endl;
        }

        return false;
    }

    //Gets obstacle pose with orientation defined as axisAn (format="axis-angle") or by default as quaternion (format="quaternion")
    bool kauthamshell::getObstaclePos(string obsname, std::vector<double> &pos){

        // std::cout << "************Getting Pose of obstacle " << obsname << std::endl;
        Problem *const problem = (Problem*)memPtr_;

        pos.clear();

        if(problem->getPlanner()->wkSpace()->getObstacle(obsname)==NULL)
        {
            cout << "Error in getObstaclePos:" << obsname << " is not in the set of obstacles " << problem->getPlanner()->wkSpace()->getNumObstacles() << endl;
            return true;
        }

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            double coord[3];
            vector<double> ort;
            ort.resize(4);

            coord[0] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[0];
            coord[1] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[1];
            coord[2] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getPosition()[2];

            ort[0] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[0];
            ort[1] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[1];
            ort[2] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[2];
            ort[3] = problem->getPlanner()->wkSpace()->getObstacle(obsname)->getLink(0)->getElement()->getOrientation()[3];

            SE3Conf se3;
            se3.setOrient(ort);
            vector<double> axisAn = se3.getAxisAngle();

            pos.push_back(coord[0]);
            pos.push_back(coord[1]);
            pos.push_back(coord[2]);
            pos.push_back(axisAn[0]);
            pos.push_back(axisAn[1]);
            pos.push_back(axisAn[2]);
            pos.push_back(axisAn[3]);

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::getObstaclePos(const std::string& _obs_name, Eigen::AffineCompact3d& _obs_pose) {

        // std::cout << "Getting pose of obstacle " << _obs_name << std::endl;
        Problem *const problem = (Problem*)memPtr_;

        Robot* obstacle = problem->getPlanner()->wkSpace()->getObstacle(_obs_name);
        if (obstacle == NULL) {
            std::cout << "Error in getObstaclePos:" << _obs_name << " is not in the set of obstacles " << problem->getPlanner()->wkSpace()->getNumObstacles() << std::endl;
            return true;
        }

        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            _obs_pose = Eigen::Translation3d(
                            obstacle->getLink(0)->getElement()->getPosition()[0],   // x
                            obstacle->getLink(0)->getElement()->getPosition()[1],   // y
                            obstacle->getLink(0)->getElement()->getPosition()[2]    // z
                        ) * Eigen::Quaterniond(
                            obstacle->getLink(0)->getElement()->getOrientation()[3],    // qw
                            obstacle->getLink(0)->getElement()->getOrientation()[0],    // qx
                            obstacle->getLink(0)->getElement()->getOrientation()[1],    // qy
                            obstacle->getLink(0)->getElement()->getOrientation()[2]     // qz
                        );
            return true;

        } catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } catch (const exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } catch(...) {
            std::cout << "Something is wrong with the problem. Please run the "
                        << "problem with the Kautham2 application at less once in order "
                        << "to verify the correctness of the problem formulation." << std::endl;
        }
        return false;
    }

    bool kauthamshell::findIK(int robIndx, bool armType, std::vector<double> pos, std::vector<double> conf, bool maintSameWrist, std::vector<double> *solution){
        bool ret = false;
        try {
            Problem *const problem = (Problem*)memPtr_;

            if(problem->getPlanner()->wkSpace()->getRobot(robIndx)->getName() == "yumi") {
                IvKinYumi *const IK = new IvKinYumi(problem->getPlanner()->wkSpace()->getRobot(robIndx), armType);
                IK->setTarget(pos, conf, maintSameWrist);
                ret = IK->solve();
                if(ret)
                    *solution = IK->getRn().getCoordinates();
            }
            else {
                std::cout<<"The inverse kinematic for the "<<problem->getPlanner()->wkSpace()->getRobot(robIndx)->getName()<<" has not been defined"<<std::endl;
            }
            return ret;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }
        return ret;
    }

    bool kauthamshell::setRobPos(unsigned int index, std::vector<double> pos) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;

            SE3Conf newconf;
            std::vector<double> newpos;
            newpos.push_back(pos.at(0));
            newpos.push_back(pos.at(1));
            newpos.push_back(pos.at(2));
            std::vector<double> newori;
            newori.push_back(pos.at(3));
            newori.push_back(pos.at(4));
            newori.push_back(pos.at(5));
            newori.push_back(pos.at(6));

            newconf.setPos(newpos);
            newconf.setOrient(newori);

            problem->getPlanner()->wkSpace()->getRobot(index)->setHomePos(&newconf);

            std::vector<double> coord = problem->getPlanner()->wkSpace()->getRobot(index)->getHomePos()->getSE3().getPos();
            std::cout<<"Robot "<<index<<" at position ("<<coord.at(0)<<", "
                    <<coord.at(1)<<", "<<coord.at(2)<<")"<<std::endl;
            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;
    }

    bool kauthamshell::getRobPos(unsigned int index, std::vector<double> &pos) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;
            SE3Conf conf = problem->getPlanner()->wkSpace()->getRobot(index)->getCurrentPos()->getSE3();
            std::cout<<"Robot "<<index<<" at position ("<<conf.getPos().at(0)<<", "
                    <<conf.getPos().at(1)<<", "<<conf.getPos().at(2)<<")"<<std::endl;

            pos.push_back(conf.getPos().at(0));
            pos.push_back(conf.getPos().at(1));
            pos.push_back(conf.getPos().at(2));
            pos.push_back(conf.getOrient().at(0));
            pos.push_back(conf.getOrient().at(1));
            pos.push_back(conf.getOrient().at(2));
            pos.push_back(conf.getOrient().at(3));
            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return false;

    }

    bool kauthamshell::getRobHomePos(unsigned int index, std::vector<double> &pos) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;
            SE3Conf conf = problem->getPlanner()->wkSpace()->getRobot(index)->getHomePos()->getSE3();
            std::cout<<"Robot "<<index<<" at position ("<<conf.getPos().at(0)<<", "
                    <<conf.getPos().at(1)<<", "<<conf.getPos().at(2)<<")"<<std::endl;

            pos.push_back(conf.getPos().at(0));
            pos.push_back(conf.getPos().at(1));
            pos.push_back(conf.getPos().at(2));
            pos.push_back(conf.getOrient().at(0));
            pos.push_back(conf.getOrient().at(1));
            pos.push_back(conf.getOrient().at(2));
            pos.push_back(conf.getOrient().at(3));
            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                << "problem with the Kautham2 application at less once in order "
                << "to verify the correctness of the problem formulation.\n";
        }

        return false;

    }

    bool kauthamshell::updateRobotDOFOffsetByName(const std::string& _robot_name, const std::vector<std::pair<std::string, double>>& _dof_updates) {
        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened!" << std::endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            
            // Get the requested robot:
            Robot* robot = problem->wSpace()->getRobot(_robot_name);
            if (!robot) {
                std::cout << "Error: Robot '" << _robot_name << "' not found!" << std::endl;
                return false;
            }

            // Update its DOF by name:
            if (!robot->Robot::updateOffMatrixDOFByName(_dof_updates)) {
                return false;
            }

            // Check collisions with the samples, also needed to updated the RobConf associated to each sample:
            SampleSet* samples = problem->getSampleSet();
            for (unsigned int i = 0; i < samples->getSize(); ++i) {

                // Get each sample of the sample set:
                Sample* sample = samples->getSampleAt(i);
                
                // Clear the previous RobConf associated to the sample coordinates:
                sample->clearMappedConf();

                // Set the new RobConf to the associated sample coordinates:
                if (problem->wSpace()->collisionCheck(sample)) {
                    std::cout << "The updated offMatrix is in collision with the current samples." << std::endl;
                    return false;
                }

            }

            return true;

        } catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } catch (const exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } catch(...) {
            std::cout << "Something is wrong with the method named kauthamshell::setRobotDOFByName()." << std::endl;
        }

        return false;
    }


    int kauthamshell::getNumRobots() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return 0;
            }

            Problem *const problem = (Problem*)memPtr_;
            return (problem->getPlanner()->wkSpace()->getNumRobots());

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }
        return 0;
    }


    int kauthamshell::getNumObstacles() {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return 0;
            }

            Problem *const problem = (Problem*)memPtr_;
            return (problem->getPlanner()->wkSpace()->getNumObstacles());

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }
        return 0;
    }

    map<string, Robot*> kauthamshell::getObstaclesMap()
    {
        Problem *const problem = (Problem*)memPtr_;
        return problem->wSpace()->getObstaclesMap();
    }

    bool kauthamshell::getRobotFileNames(std::vector<std::string> &rnames) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;

            problem->getRobotFileNames(rnames);
            /*
            std::cout<<"-------rnames: robot filename.size = "<<rnames.size()<<std::endl;
            for(int i=0; i<rnames.size(); i++)
            {
                std::cout<<"---------robot filename "<<i<<": "<<rnames[i]<<std::endl;
            }
            */

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return true;
    }


    bool kauthamshell::getObstaclesNames(std::vector<std::string> &obsnames) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            obsnames.clear();
            Problem *const problem = (Problem*)memPtr_;
            for (std::pair<std::string, Robot*> element : problem->getPlanner()->wkSpace()->getObstaclesMap())
            {
              obsnames.push_back(element.first);
            }

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return true;
    }


    bool kauthamshell::getRobotJointNames(int r, std::vector<std::string> &jnames) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            //std::cout<<"getRobotJointNames ************************"<<std::endl;
            problem->getPlanner()->wkSpace()->getRobot(r)->getJointNames(jnames);
            /*
            std::cout<<"-------jnames: jnames.size = "<<jnames.size()<<std::endl;
            for(unsigned int i=0; i<jnames.size(); i++)
            {
                std::cout<<"---------joint name "<<i<<": "<<jnames[i]<<std::endl;
            }
            */

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return true;
    }


    bool kauthamshell::getRobotIsSE3enabled(int r) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            return problem->getPlanner()->wkSpace()->getRobot(r)->isSE3Enabled();


        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return true;
    }

    bool kauthamshell::getSampleConfig(const unsigned int& _index, std::vector<double>& _sample_config) {

        _sample_config.clear();

        try {
            if (!problemOpened()) {
                std::cout << "The problem is not opened" << std::endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            // std::vector<std::string> rob_control_names =  problem->wSpace()->getRobControlsNames();
            
            Sample *req_sample = problem->getSampleSet()->getSampleAt(_index);
            if (req_sample == NULL) {
                std::cout << "Not possible to get the sample by index." << std::endl;
                return false;
            }

            // Get the robot configuration (joint values) of the requested sample, also the offsets are considered:
            std::vector<RobConf> rc = req_sample->getMappedConf();
            for(unsigned int n = 0; n < rc.size(); n++) {
                    _sample_config.push_back(rc[n].getSE3().getPos()[0]);
                    _sample_config.push_back(rc[n].getSE3().getPos()[1]);
                    _sample_config.push_back(rc[n].getSE3().getPos()[2]);
                    _sample_config.push_back(rc[n].getSE3().getOrient()[0]);
                    _sample_config.push_back(rc[n].getSE3().getOrient()[1]);
                    _sample_config.push_back(rc[n].getSE3().getOrient()[2]);
                    _sample_config.push_back(rc[n].getSE3().getOrient()[3]);

                for (unsigned int q = 0; q < rc[n].getRn().getCoordinates().size(); q++) {
                    _sample_config.push_back(rc[n].getRn().getCoordinates()[q]);
                }
            }

        } catch (const KthExcp& excp) {
            std::cout << "Error: " << excp.what() << std::endl << excp.more() << std::endl;
        } catch (const exception& excp) {
            std::cout << "Error: " << excp.what() << std::endl;
        } catch(...) {
            std::cout << "Something is wrong when getting the sample in the index " << _index << "." << std::endl;
        }

        return true;
    }

    // Kautham Public Main Constraints Methods:
    bool kauthamshell::getConstraintIdsPerRobot(std::map<std::string,std::vector<std::string>>& _constraints_by_robot) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }
        Problem *const problem = (Problem*)memPtr_;

        std::vector<std::shared_ptr<RobotProblemConstraint>> prob_robot_constraints;
        for (size_t r = 0; r < problem->wSpace()->getNumRobots(); r++) {
            Robot* robot = problem->wSpace()->getRobot(r);
            prob_robot_constraints = robot->getConstraints();
            std::vector<std::string> constraint_ids;
            for (const auto& constraint : prob_robot_constraints) {
                if (constraint) {
                    constraint_ids.push_back(constraint->getConstraintId());
                }
            }
            _constraints_by_robot[robot->getName()] = constraint_ids;
        }
        return true;
    }

    bool kauthamshell::updateConstraintTargetLink(const std::string& _robot_name, const std::string& _id, const std::string& _target_link) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setTargetLink(_target_link);
        return true;
    }

    bool kauthamshell::updateConstraintEnabled(const std::string& _robot_name, const std::string& _id, const bool& _enabled) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setEnabledStatus(_enabled);
        return true;
    }


    // Kautham Public Orientation Constraints Methods:
    bool kauthamshell::updateConstraintTargetOrientation(const std::string& _robot_name, const std::string& _id, const Eigen::Quaterniond& _target_orientation) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setTargetOrientation(_target_orientation);
        return true;
    }

    bool kauthamshell::updateConstraintTargetOrientation(const std::string& _robot_name, const std::string& _id, const double& qx, const double& qy, const double& qz, const double& qw) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setTargetOrientation(qx,qy,qz,qw);
        return true;
    }

    bool kauthamshell::updateConstraintTolerance(const std::string& _robot_name, const std::string& _id, const double& _tolerance_value, const bool& _tolerance_variable, const double& _tolerance_grandient) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setToleranceInfo(_tolerance_value, _tolerance_variable, _tolerance_grandient);
        return true;
    }

    bool kauthamshell::updateConstraintFreeMovementAxes(const std::string& _robot_name, const std::string& _id, const bool& _free_x, const bool& _free_y, const bool& _free_z) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        prob_robot_constr->setFreeMovementAxes(_free_x, _free_y, _free_z);
        return true;
    }


    // Kautham Public Geometric Constraints Methods:
    bool kauthamshell::getConstraintGeoProperties(const std::string& _robot_name, const std::string& _id, std::string& _type, std::map<std::string, double>& _geo_params_map, Eigen::AffineCompact3d& _constraint_pose, std::string& _reference_frame_entity, std::string& _reference_frame_link) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        
        _type = prob_robot_constr->getConstraintType();
        
        _geo_params_map.clear();
        _geo_params_map["length"] = prob_robot_constr->getGeometricParamLength();
        _geo_params_map["width"] = prob_robot_constr->getGeometricParamWidth();
        _geo_params_map["height"] = prob_robot_constr->getGeometricParamHeight();
        _geo_params_map["radius"] = prob_robot_constr->getGeometricParamRadius();

        _constraint_pose = prob_robot_constr->getReferencedFrameOrigin();
        _reference_frame_entity = prob_robot_constr->getReferenceFrameEntity();
        _reference_frame_link = prob_robot_constr->getReferenceFrameLink();

        return true;
    }

    bool kauthamshell::updateConstraintGeoPose(const std::string& _robot_name, const std::string& _id, const Eigen::AffineCompact3d& _constraint_origin) {
        if (!problemOpened()) {
            std::cout << "The problem is not opened" << std::endl;
            return false;
        }

        Problem *const problem = (Problem*)memPtr_;
        Robot* robot = problem->wSpace()->getRobot(_robot_name);
        std::shared_ptr<RobotProblemConstraint> prob_robot_constr = robot->getConstraintById(_id);
        prob_robot_constr->setOrigin(_constraint_origin);
        return true;
    }

}   // Namespace: Kautham
