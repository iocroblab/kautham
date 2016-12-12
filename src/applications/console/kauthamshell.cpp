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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo, Muhayyuddin */



#include <planner/omplc/omplcplanner.h>
#include <planner/omplg/omplplanner.h>
#include "kauthamshell.h"
#include "util/kthutil/kauthamexception.h"
#include <iostream>
#include <problem/problem.h>
#include <planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
#include <ode/ode.h>
#include <planner/omplOpenDE/environment/KauthamOpenDEEnvironment.h>

namespace ob = ompl::base;

namespace Kautham {
    kauthamshell::kauthamshell() {
        memPtr_ = NULL;
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
            std::cout << "Kautham is opening a problem file: " << problemfilename << endl;
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
    bool kauthamshell::setObstacle(vector<KthReal> smpcoords, int targetObs)
    {
        try {
            Problem *const problem = (Problem*)memPtr_;
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Sample* smp = new Sample(3);
            smp->setCoords(smpcoords);

             problem->wSpace()->setObstacle(smp, targetObs);

            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return false;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return false;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return false;
        }
    }

    bool kauthamshell::checkCollisionObs(vector<KthReal> smpcoords, int targetObs, int *collisionObs, bool *collisionFree) {
        try {
            Problem *const problem = (Problem*)memPtr_;
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            string msg;
            *collisionFree = !problem->wSpace()->collisionCheckObs(smpcoords,targetObs,collisionObs,&msg);
            if(msg.empty()) {
                std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
            } else {
                std::cout<<"Response for collision checking service is: "<<msg<<std::endl;
            }
            return true;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return false;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return false;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return false;
        }
    }

    bool kauthamshell::checkCollision(vector<float> smpcoords, bool *collisionFree) {
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
            string msg;
            if (smp->setCoords(smpcoords)) {
                *collisionFree = !problem->wSpace()->collisionCheck(smp,&msg);
                if(msg.empty()) {
                    std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
                } else {
                    std::cout<<"Response for collision checking service is: "<<msg<<std::endl;
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


    bool kauthamshell::setRobotsConfig(vector<float> smpcoords) {
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

                    x = (*it).obs->getCurrentPos()->getSE3().getPos()[0];
                    y = (*it).obs->getCurrentPos()->getSE3().getPos()[1];
                    z = (*it).obs->getCurrentPos()->getSE3().getPos()[2];
                    std::cout<<"Object "<<obsname<<" is at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
                }
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


    bool kauthamshell::setObstaclesConfig(vector<float> smpcoords) {
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


    bool kauthamshell::setQuery(vector<float> init, vector<float> goal) {
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


    bool kauthamshell::setInit(vector<float> init) {
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


    bool kauthamshell::setGoal(vector<float> goal) {
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


    bool kauthamshell::setInitObs(vector<float> initObs) {
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


    bool kauthamshell::setRobControls(istream *inputfile, vector<float> init, vector<float> goal) {
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


    bool kauthamshell::setRobControls(string controlsFile, vector<float> init, vector<float> goal) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (!problem->setRobotControls(controlsFile)) return false;
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


    bool kauthamshell::setDefaultRobControls(vector<float> init, vector<float> goal) {
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


    bool kauthamshell::setObsControls(istream *inputfile, vector<float> initObs) {
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


    bool kauthamshell::setObsControls(string controlsFile, vector<float> initObs) {
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
            vector<KthReal> coords;
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


    bool kauthamshell::connect(vector<float> smpcoords1, vector<float> smpcoords2) {
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

                        ((IOC::iocPlanner*)planner)->getLocalPlanner()->setInitSamp(fromSample);
                        ((IOC::iocPlanner*)planner)->getLocalPlanner()->setGoalSamp(toSample);

                        if (((IOC::iocPlanner*)planner)->getLocalPlanner()->canConect()) {
                            cout << "The samples can be connected." << endl;
                            return true;
                        } else {
                            cout << "The samples can not be connected." << endl;
                        }
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
            if (problem->getPlanner()->getFamily()==OMPLPLANNER || problem->getPlanner()->getFamily()==ODEPLANNER) {
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
            if (problem->getPlanner()->getFamily()==OMPLPLANNER || problem->getPlanner()->getFamily()==ODEPLANNER) {
                if (problem->getPlanner()->solveAndInherit()) {
                    ((omplplanner::omplPlanner*)problem->getPlanner())->SimpleSetup()->
                            getSolutionPath().printAsMatrix(path);

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


    int kauthamshell::addRobot(string robFile, double scale, vector<float> home, vector<vector<float> > limits,
                               vector<vector<float> > mapMatrix, vector<float> offMatrix) {
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
            KthReal **MapMatrix = new KthReal*[numDOF];
            KthReal *OffMatrix = new KthReal[numDOF];
            for (int i = 0; i < numDOF; ++i) {
                OffMatrix[i] = offMatrix.at(i);
                MapMatrix[i] = new KthReal[numCntr];
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


    int kauthamshell::addObstacle(string obsFile, double scale, vector<float> home) {
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


    bool kauthamshell::removeObstacle(unsigned index) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            if (index < problem->wSpace()->getNumObstacles()) {
                problem->wSpace()->removeObstacle(index);
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


    bool kauthamshell::attachObstacle2RobotLink(int robot, int link, int obs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            bool ret = problem->wSpace()->attachObstacle2RobotLink(robot,link,obs);
            float x,y,z;
            x = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[0];
            y = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[1];
            z = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[2];
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


    bool kauthamshell::detachObstacle(unsigned int obs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            bool ret = problem->wSpace()->detachObstacle(obs);
            float x,y,z;
            x = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[0];
            y = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[1];
            z = problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[2];
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

    bool kauthamshell::motionPlanner(vector <float> init, vector <float> goal, string root){

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
            //out.open(root.c_str()+"Aplicaciones/Experiments/DatosGA2H/SolutionPlan.txt");
            out.open((root+"Aplicaciones/Experiments/DatosGA2H/SolutionPlan.txt").c_str());
            out1.open((root+"Aplicaciones/Experiments/DatosGA2H/R1SolutionPlan.txt").c_str());
            out2.open((root+"Aplicaciones/Experiments/DatosGA2H/R2SolutionPlan.txt").c_str());

            out1S.open((root+"Aplicaciones/Experiments/DatosGA2H/R1SolutionPlanS.txt").c_str());
            out2S.open((root+"Aplicaciones/Experiments/DatosGA2H/R2SolutionPlanS.txt").c_str());

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

    bool kauthamshell::setManipPramsAndSolve(string actiontype, int targetbody, std::vector<double> force,
                                            std::vector<std::pair<std::vector<float>, std::vector<float> > > *ws,
                                             double *power,std::vector<double> *laststate)
    {
         // std::vector<std::pair<std::vector<double>,std::vector<double> > >
        try {
            if (!problemOpened())
            {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;
            dJointID joint;
            ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                    stateSpace->getEnvironment().get())->manipulationQuery->setActionType(actiontype);
            string action = (((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                        stateSpace->getEnvironment().get())->manipulationQuery->getActionType());
            if(action=="push" || action== "pull")
             {

            ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                    stateSpace->getEnvironment().get())->stateBodies_[targetbody];

            ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                    stateSpace->getEnvironment().get())->manipulationQuery->setforce(force);

            std::vector<double> f = ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                     stateSpace->getEnvironment().get())->manipulationQuery->getforce();
            }
            if(actiontype=="pull"||actiontype=="Pull")
            {
                unsigned int robBodyIndex = problem->wSpace()->getRobot(0)->getNumLinks()-1;
                ((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->stateSpace->getEnvironment().get()->stateBodies_[robBodyIndex];
                joint=dJointCreateHinge(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->stateSpace->getEnvironment().get()->world_ , 0);
                dJointAttach (joint,((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->stateSpace->getEnvironment().get()->stateBodies_[robBodyIndex],
                              ((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->stateSpace->getEnvironment().get()->stateBodies_[targetbody]);

            }





//            std::cout<<"Action is : "<<  ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
//                                          stateSpace->getEnvironment().get())->manipulationQuery->getActionType()<<" :: force is :: [" <<f[0]<<" , "<<f[1]<<" , "<<f[2]<<"] Target body is: "<<targetbody<<std::endl;
            bool solve = false;
            solve = problem->getPlanner()->solveAndInherit();
            if (solve)
            {
                //*ws = ((omplcplanner::KauthamDEPlanner*)problem->getPlanner())->worldState;
                std::vector<State> worldState = ((omplcplanner::KauthamDEPlanner*)problem->getPlanner())->worldState;

                for (std::vector<State>::iterator state = worldState.begin();
                     state != worldState.end(); ++state) {
                    std::pair<std::vector<float>,std::vector<float> > st;
                    st = std::make_pair(state->getRob()->getCoords(),state->getObs()->getCoords());
                    ws->push_back(st);
                }

                *power=((omplcplanner::KauthamDEPlanner*)problem->getPlanner())->PowerConsumed;
                *laststate=((omplcplanner::KauthamDEPlanner*)problem->getPlanner())->lastState;

            }
            std::string actiontype = ((omplcplanner::KauthamDEEnvironment*)((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                      stateSpace->getEnvironment().get())->manipulationQuery->getActionType();

            if(actiontype=="pull"||actiontype=="Pull")
            {
                dJointDestroy(joint);
            }

            return solve;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return false;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return false;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return false;
        }
    }


    bool kauthamshell::setBodyState(int targetBody, std::vector<double> pose)
    {
        Problem *const problem = (Problem*)memPtr_;
        dBodySetPosition(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                         stateSpace->getEnvironment().get()->stateBodies_[targetBody],pose[0],pose[1],pose[2]);
        dQuaternion q;

        q[0]=pose[3];
        q[1]=pose[4];
        q[2]=pose[5];
        q[3]=pose[6];
        dBodySetQuaternion(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                           stateSpace->getEnvironment().get()->stateBodies_[targetBody],q);
        return true;
    }

    std::vector<double> kauthamshell::getBodyState(int targetBody)
    {
        Problem *const problem = (Problem*)memPtr_;
        std::vector<double> pose(7);
        const dReal *p = dBodyGetPosition(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                          stateSpace->getEnvironment().get()->stateBodies_[targetBody]);
        const dReal *q = dBodyGetQuaternion(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                            stateSpace->getEnvironment().get()->stateBodies_[targetBody]);
        pose[0]=p[0];
        pose[1]=p[1];
        pose[2]=p[2];

        pose[3]=q[0];
        pose[4]=q[1];
        pose[5]=q[2];
        pose[6]=q[3];
        return pose;
    }

    bool kauthamshell::setWorldState(std::vector< std::vector<double> > worldstate)
    {
        Problem *const problem = (Problem*)memPtr_;
        dQuaternion q;
        for(unsigned int i=0;i<worldstate.size();i++)
        {
            worldstate[i].resize(7);
            dBodySetPosition(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                             stateSpace->getEnvironment().get()->stateBodies_[i],worldstate[i][0],worldstate[i][1],worldstate[i][2]);
            q[0] = worldstate[i][3];
            q[1] = worldstate[i][4];
            q[2] = worldstate[i][5];
            q[3] = worldstate[i][6];

            dBodySetQuaternion(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                               stateSpace->getEnvironment().get()->stateBodies_[i],q);
        }
        return true;
    }

    std::vector<std::vector<double> > kauthamshell::getWorldState()
    {
        Problem *const problem = (Problem*)memPtr_;
        std::vector< std::vector<double> > worldstate;
        unsigned int bodyCount=((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->stateSpace->getEnvironment().get()->stateBodies_.size();
        worldstate.resize(bodyCount);
        for(unsigned int i=0;i<bodyCount;i++)
        {
            worldstate[i].resize(7);
            const dReal *p = dBodyGetPosition(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                              stateSpace->getEnvironment().get()->stateBodies_[i]);
            const dReal *q = dBodyGetQuaternion(((omplcplanner::KauthamDEPlanner*)(problem->getPlanner()))->
                                                stateSpace->getEnvironment().get()->stateBodies_[i]);
            worldstate[i][0]=p[0];
            worldstate[i][1]=p[1];
            worldstate[i][2]=p[2];

            worldstate[i][3]=q[0];
            worldstate[i][4]=q[1];
            worldstate[i][5]=q[2];
            worldstate[i][6]=q[3];

        }
        for(unsigned int i=0;i<bodyCount;i++)
        {

            std::cout<<"state of body "<< i << " is ["<< worldstate[i][0]<<" , "
                    << worldstate[i][1]<<" , "
                    << worldstate[i][2]<<" , "
                    << worldstate[i][3]<<" , "
                    << worldstate[i][4]<<" , "
                    << worldstate[i][5]<<" , "
                    << worldstate[i][6]<<" ] "<<std::endl;

        }

        return worldstate;
    }




}
