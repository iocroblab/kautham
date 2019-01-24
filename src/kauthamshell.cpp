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


#include <iostream>

#include <kautham/kauthamshell.h>
#include <kautham/planner/omplg/omplplanner.h>
#include <kautham/util/kthutil/kauthamexception.h>
#include <kautham/problem/problem.h>
#include <kautham/util/libkin/ivkinyumi.h>


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


    bool kauthamshell::checkCollision(vector<float> smpcoords, bool *collisionFree, std::pair< std::pair<int, int> , std::pair<int,int> > *colliding_elements) {
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
                *collisionFree = !problem->wSpace()->collisionCheck(smp,&msg,colliding_elements);
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

    double kauthamshell::cumDistCheck(std::vector<float> smpcoords) {
        Sample *smp = NULL;
        double value;

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


    bool kauthamshell::checkCollisionRob(std::vector<float> smpcoords, std::vector<unsigned> *ObstColl) {
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
            bool collisionFree;
            if (smp->setCoords(smpcoords)) {
                collisionFree = !problem->wSpace()->collisionCheckObstacles(smp,*ObstColl);
                if(!collisionFree) {
                    std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
                    delete smp;
                    return collisionFree;
                } else {
                    std::cout<<"Response for collision checking service is: "<<msg<<std::endl;
                }

                delete smp;
                return collisionFree;
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

    bool kauthamshell::checkCollisionObs(int index, std::vector<unsigned> *collObs, std::string *msg) {
        Sample *smp = NULL;

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;


            bool collisionFree;
            collisionFree = !problem->wSpace()->collisionCheckObs(index, collObs, msg);
            if(!collisionFree) {
                std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
                return collisionFree;
            } else {
                std::cout<<"Response for collision checking service is: "<<msg<<std::endl;
            }
            return collisionFree;

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

                    x = (*it).obs->getLink(0)->getElement()->getPosition()[0];
                    y = (*it).obs->getLink(0)->getElement()->getPosition()[1];
                    z = (*it).obs->getLink(0)->getElement()->getPosition()[2];
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


    bool kauthamshell::detachObstacle(unsigned int obs) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
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

    bool kauthamshell::setObstaclePos(int index, std::vector<float> pos){

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Problem *const problem = (Problem*)memPtr_;
            float  coord[3];
            float ort[4];

            coord[0] = pos[0];
            coord[1] = pos[1];
            coord[2] = pos[2];
            ort[0] = pos[3];
            ort[1] = pos[4];
            ort[2] = pos[5];
            ort[3] = pos[6];

            problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->setPosition(coord);
            problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->setOrientation(ort);

            float x,y,z;
            x = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[0];
            y = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[1];
            z = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[2];
            std::cout<<"Object "<<index<<" at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;

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

    std::vector<float> kauthamshell::getObstaclePos(int index){

        std::vector<float> pos;
        pos.clear();

        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return pos;
            }

            Problem *const problem = (Problem*)memPtr_;

            float coord[3];
            float ort[4];

            coord[0] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[0];
            coord[1] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[1];
            coord[2] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getPosition()[2];

            ort[0] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getOrientation()[0];
            ort[1] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getOrientation()[1];
            ort[2] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getOrientation()[2];
            ort[3] = problem->getPlanner()->wkSpace()->getObstacle(index)->getLink(0)->getElement()->getOrientation()[3];

            pos.push_back(coord[0]);
            pos.push_back(coord[1]);
            pos.push_back(coord[2]);
            pos.push_back(ort[0]);
            pos.push_back(ort[1]);
            pos.push_back(ort[2]);
            pos.push_back(ort[3]);

            return pos;

        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
        }

        return pos;
    }

    bool kauthamshell::findIK(int robIndx, bool armType, std::vector<float> pos, std::vector<float> conf, bool maintSameWrist, std::vector<float> *solution){
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

    bool kauthamshell::setRobPos(unsigned int index, std::vector<float> pos) {
        try {
            if (!problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }
            Problem *const problem = (Problem*)memPtr_;

            SE3Conf newconf;
            std::vector<float> newpos;
            newpos.push_back(pos.at(0));
            newpos.push_back(pos.at(1));
            newpos.push_back(pos.at(2));
            std::vector<float> newori;
            newori.push_back(pos.at(3));
            newori.push_back(pos.at(4));
            newori.push_back(pos.at(5));
            newori.push_back(pos.at(6));

            newconf.setPos(newpos);
            newconf.setOrient(newori);

            problem->getPlanner()->wkSpace()->getRobot(index)->setHomePos(&newconf);

            std::vector<float> coord = problem->getPlanner()->wkSpace()->getRobot(index)->getHomePos()->getSE3().getPos();
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

    bool kauthamshell::getRobPos(unsigned int index, std::vector<float> &pos) {
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

}
