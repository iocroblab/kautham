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

/* Author: Nestor Garcia Hidalgo */


#if defined(KAUTHAM_USE_OMPL)
#include <boost/bind/mem_fn.hpp>
#include <iostream>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <kautham/planner/omplg/omplFOSRRTStarplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/omplg/FOSUpstreamCriterionOptimizationObjective.h>
#include <kautham/planner/omplg/vectorField.h>

namespace Kautham {
    namespace omplplanner{
        //! Constructor
        omplFOSRRTStarPlanner::omplFOSRRTStarPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                                     SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr,
                                                     const std::string &synergyTreeFilename):
            omplPlanner(stype, init, goal, samples, ws, ssptr)
        {
            _guiName = "ompl FOSRRT Star Planner";
            _idName = "omplFOSRRTStar";

            //create planner
            og::RRTstar *planner(new og::RRTstar(si));
            if (synergyTreeFilename.empty()) {
                st_ = new VectorField();
            } else {
                st_ =  new SynergyTree(synergyTreeFilename);
            }
            ob::FOSUpstreamCriterionOptimizationObjective *opt =
                    new ob::FOSUpstreamCriterionOptimizationObjective(si,this,st_);

            ob::ProblemDefinitionPtr pdefPtr = ss->getProblemDefinition();
            pdefPtr->setOptimizationObjective(ob::OptimizationObjectivePtr(opt));
            planner->setProblemDefinition(pdefPtr);
            planner->setup();

            //set planner parameters:
            addParameter("Range",planner->getRange());
            addParameter("Goal Bias",planner->getGoalBias());
            addParameter("Use K-Nearest (0/1)",planner->getKNearest());
            addParameter("K-Neigh Factor",planner->getRewireFactor());
            addParameter("Delay CC (0/1)",planner->getDelayCC());
            addParameter("Pruning (0/1)",planner->getTreePruning());
            addParameter("Prune Threshold",planner->getPruneThreshold());
            addParameter("Pruned Measure (0/1)",planner->getPrunedMeasure());
            addParameter("Informed Sampling (0/1)",planner->getInformedSampling());
            addParameter("Sample Rejection (0/1)",planner->getSampleRejection());
            addParameter("New State Rejection (0/1)",planner->getNewStateRejection());
            addParameter("Admissible Cost2Come (0/1)",planner->getAdmissibleCostToCome());
            addParameter("Focus Search (0/1)",planner->getFocusSearch());
            addParameter("Num Sampling Attempts",planner->getNumSamplingAttempts());
            if (synergyTreeFilename.empty()) {
                addParameter("Problem",((VectorField*)st_)->getProblem());
            }

            //set the planner
            ss->setPlanner(ob::PlannerPtr(planner));
        }


        //! void destructor
        omplFOSRRTStarPlanner::~omplFOSRRTStarPlanner(){

        }


        //! function to find a solution path
        bool omplFOSRRTStarPlanner::trySolve()
        {
            if (omplPlanner::trySolve()) {
                //evaluate path
                cout << "path with " << ((og::PathGeometric)ss->getSolutionPath()).getStateCount() << " states" << endl;
                ob::Cost pathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(ss->getProblemDefinition()->getOptimizationObjective());
                cout << "Path cost = " << pathcost.value() << endl;

                return true;
            } else {
                cout<<"No solution found"<<endl;

                return true;
            }
        }


        //! setParameters sets the parameters of the planner
        bool omplFOSRRTStarPlanner::setParameters() {
            if (!omplPlanner::setParameters()) return false;

            try {
                HASH_S_K::iterator it;
                og::RRTstar *planner = ss->getPlanner()->as<og::RRTstar>();

                it = _parameters.find("Range");
                if (it == _parameters.end()) return false;
                if (it->second <= 0.) {
                    it->second = planner->getRange();
                } else {
                    planner->setRange(it->second);
                }

                it = _parameters.find("Goal Bias");
                if (it == _parameters.end()) return false;
                if (it->second < 0. || it->second > 1.) {
                    it->second = planner->getGoalBias();
                } else {
                    planner->setGoalBias(it->second);
                }

                it = _parameters.find("Use K-Nearest (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getKNearest();
                } else {
                    planner->setKNearest(it->second);
                }

                it = _parameters.find("K-Neigh Factor");
                if (it == _parameters.end()) return false;
                if (it->second <= 0.) {
                    it->second = planner->getRewireFactor();
                } else {
                    planner->setRewireFactor(it->second);
                }

                it = _parameters.find("Delay CC (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getDelayCC();
                } else {
                    planner->setDelayCC(it->second);
                }

                it = _parameters.find("Pruning (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getTreePruning();
                } else {
                    planner->setTreePruning(it->second);
                }

                it = _parameters.find("Prune Threshold");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = planner->getPruneThreshold();
                } else {
                    planner->setPruneThreshold(it->second);
                }

                it = _parameters.find("Pruned Measure (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getPrunedMeasure();
                } else {
                    planner->setPrunedMeasure(it->second);
                }

                it = _parameters.find("Informed Sampling (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getInformedSampling();
                } else {
                    planner->setInformedSampling(it->second);
                }

                it = _parameters.find("Sample Rejection (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getSampleRejection();
                } else {
                    planner->setSampleRejection(it->second);
                }

                it = _parameters.find("New State Rejection (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getNewStateRejection();
                } else {
                    planner->setNewStateRejection(it->second);
                }

                it = _parameters.find("Admissible Cost2Come (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getAdmissibleCostToCome();
                } else {
                    planner->setAdmissibleCostToCome(it->second);
                }

                it = _parameters.find("Focus Search (0/1)");
                if (it == _parameters.end()) return false;
                if (it->second != 0. && it->second != 1.) {
                    it->second = planner->getFocusSearch();
                } else {
                    planner->setFocusSearch(it->second);
                }

                it = _parameters.find("Num Sampling Attempts");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = planner->getNumSamplingAttempts();
                } else {
                    planner->setNumSamplingAttempts(it->second);
                }

                if (dynamic_cast<VectorField*>(st_)) {
                    it = _parameters.find("Problem");
                    if (it == _parameters.end()) return false;
                    if (it->second < 0.) {
                        it->second = ((VectorField*)st_)->getProblem();
                    } else {
                        ((VectorField*)st_)->setProblem(it->second);
                    }
                }
            } catch(...) {
                return false;
            }
            return true;
        }
    }
}


#endif // KAUTHAM_USE_OMPL
