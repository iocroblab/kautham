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


namespace ob = ompl::base;

namespace Kautham {
    bool kauthamshell::openProblem(istream* inputfile, vector <string> def_path) {
        try {
            delete _problem;
            _problem = new Problem();
            if (_problem->setupFromFile(inputfile,def_path)) {
                _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
                _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
                return true;
            } else  {
                return false;
            }
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


    bool kauthamshell::openProblem(string problemfilename, vector <string> def_path) {
        try {
            std::cout << "Kautham is opening a problem file: " << problemfilename << endl;
            delete _problem;
            _problem = new Problem();
            if (_problem->setupFromFile(problemfilename,def_path)) {
                _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
                _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::checkCollision(vector<KthReal> smpcoords, bool *collisionFree) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Sample* smp = new Sample(_problem->wSpace()->getNumRobControls());
            if (smp->setCoords(smpcoords)) {
                *collisionFree = !_problem->wSpace()->collisionCheck(smp);
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setRobotsConfig(vector<KthReal> smpcoords) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Sample* smp = new Sample(_problem->wSpace()->getNumRobControls());
            if (smp->setCoords(smpcoords)) {
                _problem->wSpace()->moveRobotsTo(smp);
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setObstaclesConfig(vector<KthReal> smpcoords) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            Sample* smp = new Sample(_problem->wSpace()->getNumObsControls());
            if (smp->setCoords(smpcoords)) {
                _problem->wSpace()->moveObstaclesTo(smp);
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setQuery(vector<KthReal> init, vector<KthReal> goal) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            int d = _problem->wSpace()->getNumRobControls();
            SampleSet* samples = _problem->getSampleSet();
            samples->clear();

            //init
            Sample* smp = new Sample(d);
            smp->setCoords(init);
            if(_problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            //goal
            smp = new Sample(d);
            smp->setCoords(goal);
            if(_problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

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


    bool kauthamshell::setInit(vector<KthReal> init) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            int d = _problem->wSpace()->getNumRobControls();
            SampleSet* samples = _problem->getSampleSet();
            Sample *goal = new Sample(samples->getSampleAt(1));
            samples->clear();

            //init
            Sample* smp = new Sample(d);
            smp->setCoords( init );
            if(_problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            //goal
            samples->add(goal);

            _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

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


    bool kauthamshell::setGoal(vector<KthReal> goal) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            int d = _problem->wSpace()->getNumRobControls();
            SampleSet* samples = _problem->getSampleSet();
            Sample *init = new Sample(samples->getSampleAt(0));
            samples->clear();

            //init
            samples->add(init);

            //goal
            Sample* smp = new Sample(d);
            smp->setCoords(goal);
            if (_problem->wSpace()->collisionCheck(smp)) return false;
            samples->add(smp);

            _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

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


    bool kauthamshell::setInitObs(vector<KthReal> initObs) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            int d = _problem->wSpace()->getNumObsControls();
            Sample* smp = new Sample(d);
            smp->setCoords(initObs);
            _problem->wSpace()->setInitObsSample(smp);
            return (!_problem->wSpace()->collisionCheck(_problem->getSampleSet()->getSampleAt(0)) &&
                    !_problem->wSpace()->collisionCheck(_problem->getSampleSet()->getSampleAt(1)));
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


    bool kauthamshell::clearSampleSet() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            SampleSet* samples = _problem->getSampleSet();
            Sample *init = new Sample(samples->getSampleAt(0));
            Sample *goal = new Sample(samples->getSampleAt(1));
            samples->clear();

            samples->add(init);
            samples->add(goal);

            _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

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


    bool kauthamshell::setRobControls(istream* inputfile, vector<KthReal> init, vector<KthReal> goal) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setRobotControls(inputfile)) return false;
            return (setQuery(init,goal));
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


    bool kauthamshell::setRobControls(string controlsFile, vector<KthReal> init, vector<KthReal> goal) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setRobotControls(controlsFile)) return false;
            return (setQuery(init,goal));
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


    bool kauthamshell::setDefaultRobControls(vector<KthReal> init, vector<KthReal> goal) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setDefaultRobotControls()) return false;
            return (setQuery(init,goal));
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


    bool kauthamshell::setObsControls(istream* inputfile, vector<KthReal> initObs) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setObstacleControls(inputfile)) return false;
            return (setInitObs(initObs));
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


    bool kauthamshell::setObsControls(string controlsFile, vector<KthReal> initObs) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setObstacleControls(controlsFile)) return false;
            return (setInitObs(initObs));
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


    bool kauthamshell::setPlannerByName(string name) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (_problem->createPlanner(name)) {
                _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
                _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setPlanner(istream* inputfile) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            _problem->resetPlanner();
            if (_problem->createPlannerFromFile(inputfile)) {
                _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
                _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setPlanner(string problemfilename) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (_problem->createPlannerFromFile(problemfilename))  {
                _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
                _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
                return true;
            } else {
                return false;
            }
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


    bool kauthamshell::setPlannerParameter(string parameter, string value) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            return (_problem->getPlanner()->setParametersFromString(parameter+"|"+value));
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


    bool kauthamshell::setFixedObsControls() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (!_problem->setFixedObstacleControls()) return false;
            vector<KthReal> coords;
            coords.resize(0);
            return (setInitObs(coords));
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


    bool kauthamshell::solve() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            return _problem->getPlanner()->solveAndInherit();
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


    double kauthamshell::getLastPlanComputationTime() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1.0;
            }

            Planner *_planner = _problem->getPlanner();
            if (_planner != NULL) {
                switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER:
                    return ((omplplanner::omplPlanner*)_planner)->SimpleSetup()->getLastPlanComputationTime();
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
                return -1.0;
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                return -1.0;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return -1.0;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return -1.0;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return -1.0;
        }
    }


    int kauthamshell::getNumEdges() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Planner *_planner = _problem->getPlanner();
            if (_planner != NULL) {
                switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)_planner)->ss->getSpaceInformation()));
                    ((omplplanner::omplPlanner*)_planner)->ss->getPlanner()->getPlannerData(*pdata);
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
                return -1;
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                return -1;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return -1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return -1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return -1;
        }
    }


    int kauthamshell::getNumVertices() {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            Planner *_planner = _problem->getPlanner();
            if (_planner != NULL) {
                switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    cout << "This function is not implemeted yet for this planner family" << endl;
                    break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)_planner)->ss->getSpaceInformation()));
                    ((omplplanner::omplPlanner*)_planner)->ss->getPlanner()->getPlannerData(*pdata);
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
                return -1;
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                return -1;
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return -1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return -1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return -1;
        }
    }


    bool kauthamshell::connect(vector<float> smpcoords1, vector<float> smpcoords2) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            int dim = _problem->getDimension();
            if (smpcoords1.size() != dim || smpcoords2.size() != dim) {
                cout << "The samples should have dimension " << dim << endl;
            }

            Planner *_planner = _problem->getPlanner();
            if (_planner != NULL) {
                Sample *fromSample = new Sample(dim);
                fromSample->setCoords(smpcoords1);
                Sample *toSample = new Sample(dim);
                toSample->setCoords(smpcoords2);

                switch ((int)_planner->getFamily()) {
    #if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER: {

                    ((IOC::iocPlanner*)_planner)->getLocalPlanner()->setInitSamp(fromSample);
                    ((IOC::iocPlanner*)_planner)->getLocalPlanner()->setGoalSamp(toSample);

                    if (((IOC::iocPlanner*)_planner)->getLocalPlanner()->canConect()) {
                        cout << "The samples can be connected." << endl;
                        return true;
                    } else {
                        cout << "The samples can not be connected." << endl;
                        return false;
                    }
                    break;
                }
    #endif
    #if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER: {
                    ((omplplanner::omplPlanner*)_planner)->SimpleSetup()->setup();

                    ob::ScopedState<ob::CompoundStateSpace> fromState(((omplplanner::omplPlanner*)_planner)->getSpace());
                    ((omplplanner::omplPlanner*)_planner)->smp2omplScopedState(fromSample,&fromState);

                    ob::ScopedState<ob::CompoundStateSpace> toState(((omplplanner::omplPlanner*)_planner)->getSpace());
                    ((omplplanner::omplPlanner*)_planner)->smp2omplScopedState(toSample,&toState);

                    bool connected = ((ob::MotionValidator*)((ob::SpaceInformation*)((omplplanner::omplPlanner*)
                                                                                       _planner)->
                                                              SimpleSetup()->getSpaceInformation().get())->
                                      getMotionValidator().get())->checkMotion(fromState.get(),toState.get());
                    if (connected) {
                        cout << "The samples can be connected." << endl;
                        return true;
                    } else {
                        cout << "The samples can not be connected." << endl;
                        return false;
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
                return false;
            } else {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                return false;
            }
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


    bool kauthamshell::solve(ostream &graphVizPlannerDataFile) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            bool ret = false;
            if (_problem->getPlanner()->getFamily()==OMPLPLANNER) {
                ret = _problem->getPlanner()->solveAndInherit();
                if (ret) {

                    omplplanner::omplPlanner *p = (omplplanner::omplPlanner *)_problem->getPlanner();

                    ob::PlannerDataPtr pdata;
                    pdata = ((ob::PlannerDataPtr) new ob::PlannerData(p->ss->getSpaceInformation()));

                    p->ss->getPlanner()->getPlannerData(*pdata);
                    pdata->printGraphviz(graphVizPlannerDataFile);
                }
            }

            return ret;
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

    bool kauthamshell::getPath(ostream &path) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            bool ret = false;

            if (_problem->getPlanner()->getFamily()==OMPLPLANNER) {
                ret = _problem->getPlanner()->solveAndInherit();
                if (ret) {
                    ((omplplanner::omplPlanner*)_problem->getPlanner())->SimpleSetup()->
                            getSolutionPath().printAsMatrix(path);
                }
            }

            return ret;
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


    int kauthamshell::addRobot(string robFile, KthReal scale, vector<KthReal> home, vector< vector<KthReal> > limits,
                                  vector< vector<KthReal> > mapMatrix, vector<KthReal> offMatrix) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            if (!_problem->addRobot2WSpace(robFile,scale,home,limits)) return (-1);
            int index = _problem->wSpace()->getNumRobots()-1;
            Robot *rob = _problem->wSpace()->getRobot(index);
            int numCntr = _problem->wSpace()->getNumRobControls();
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
            return -1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return -1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return -1;
        }
    }


    int kauthamshell::addObstacle(string obsFile, KthReal scale, vector<KthReal> home) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return -1;
            }

            if (!_problem->addObstacle2WSpace(obsFile,scale,home)) return (-1);
            int index = _problem->wSpace()->getNumObstacles()-1;
            return index;
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return -1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return -1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return -1;
        }
    }


    bool kauthamshell::removeRobot(int index) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (index < 0 && index >= _problem->wSpace()->getNumRobots()) {
                return false;
            } else {
                _problem->wSpace()->removeRobot(index);
                return true;
            }
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


    bool kauthamshell::removeObstacle(int index) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            if (index < 0 && index >= _problem->wSpace()->getNumObstacles()) {
                return false;
            } else {
                _problem->wSpace()->removeObstacle(index);
                return true;
            }
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


    bool kauthamshell::attachObstacle2RobotLink(int robot, int link, int obs) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            return (_problem->wSpace()->attachObstacle2RobotLink(robot,link,obs));
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


    bool kauthamshell::detachObstacle(uint obs) {
        try {
            if (_problem == NULL || !problemOpened()) {
                cout << "The problem is not opened" << endl;
                return false;
            }

            return (_problem->wSpace()->detachObstacle(obs));
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
}
