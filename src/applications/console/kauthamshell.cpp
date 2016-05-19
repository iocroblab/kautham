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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */



#include <planner/omplg/omplplanner.h>
#include "kauthamshell.h"
#include "util/kthutil/kauthamexception.h"
#include <iostream>
#include <problem/problem.h>

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
            if (problem->getPlanner()->getFamily()==OMPLPLANNER) {
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
}
