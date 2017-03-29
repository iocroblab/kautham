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
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplLazyTRRTplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/omplg/omplMyOptimizationObjective.h>
#include <kautham/planner/omplg/lazyTRRT.h>


namespace Kautham {
    namespace omplplanner {
        //! Constructor
        omplLazyTRRTPlanner::omplLazyTRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                                 SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr) :
            omplPlanner(stype, init, goal, samples, ws, ssptr) {
            _guiName = "ompl Lazy TRRT Planner";
            _idName = "omplLazyTRRT";

            //create planner
            ob::PlannerPtr planner(new og::LazyTRRT(si));

            _opti = ob::OptimizationObjectivePtr(new myMWOptimizationObjective(ss->getSpaceInformation(), this, false));

            ob::ProblemDefinitionPtr pdefPtr = ss->getProblemDefinition();
            pdefPtr->setOptimizationObjective(_opti);

            planner->setProblemDefinition(pdefPtr);

            //set the planner
            ss->setPlanner(planner);

            //set planner parameters: range and goalbias
            _Range=(planner->as<og::LazyTRRT>())->getRange();
            _GoalBias=(planner->as<og::LazyTRRT>())->getGoalBias();
            _maxStatesFailed = (planner->as<og::LazyTRRT>())->getMaxStatesFailed();
            _tempChangeFactor = (planner->as<og::LazyTRRT>())->getTempChangeFactor();
            _frontierThreshold = (planner->as<og::LazyTRRT>())->getFrontierThreshold();
            _frontierNodesRatio = (planner->as<og::LazyTRRT>())->getFrontierNodeRatio();

            addParameter("Goal Bias", _GoalBias);
            addParameter("Max States Failed", _maxStatesFailed);
            addParameter("T Change Factor", _tempChangeFactor);
            addParameter("Frontier Nodes Ratio", _frontierNodesRatio);
            addParameter("Path Length Weight",0.00001);
            addParameter("Min Temp",planner->as<og::LazyTRRT>()->getMinTemperature());
            addParameter("Init Temp",planner->as<og::LazyTRRT>()->getInitTemperature());
            addParameter("Min Collision Threshold",planner->as<og::LazyTRRT>()->getMinCollisionThreshold());
            addParameter("Max Collision Threshold",planner->as<og::LazyTRRT>()->getMaxCollisionThreshold());
        }

        //! void destructor
        omplLazyTRRTPlanner::~omplLazyTRRTPlanner() {
        }


        bool omplLazyTRRTPlanner::setPotentialCost(string filename) {
            bool result = ((myMWOptimizationObjective*) _opti.get())->setPotentialCost(filename);

            ss->getPlanner()->as<og::LazyTRRT>()->setup();
            addParameter("Range", ss->getPlanner()->as<og::LazyTRRT>()->getRange());
            addParameter("K Constant", ss->getPlanner()->as<og::LazyTRRT>()->getKConstant());
            addParameter("Frontier Threshold", ss->getPlanner()->as<og::LazyTRRT>()->getFrontierThreshold());

            return result;
        }


        //! setParameters sets the parameters of the planner
        bool omplLazyTRRTPlanner::setParameters() {
            if (!omplPlanner::setParameters()) return false;
            try {
                HASH_S_K::iterator it;

                it = _parameters.find("Range");
                if (it != _parameters.end()) {
                    _Range = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setRange(_Range);
                    if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                        space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
                        space->setup();
                    }
                } else {
                    return false;
                }

                it = _parameters.find("Goal Bias");
                if (it != _parameters.end()) {
                    _GoalBias = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setGoalBias(_GoalBias);
                } else {
                    return false;
                }

                it = _parameters.find("Max States Failed");
                if (it != _parameters.end()) {
                    _maxStatesFailed = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setMaxStatesFailed(_maxStatesFailed);
                } else {
                    return false;
                }

                it = _parameters.find("T Change Factor");
                if (it != _parameters.end()) {
                    _tempChangeFactor = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setTempChangeFactor(_tempChangeFactor);
                } else {
                    return false;
                }

                it = _parameters.find("Frontier Threshold");
                if (it != _parameters.end()) {
                    _frontierThreshold = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setFrontierThreshold(_frontierThreshold);
                } else {
                    return false;
                }


                it = _parameters.find("Init Temp");
                if (it != _parameters.end()) {
                    ss->getPlanner()->as<og::LazyTRRT>()->setInitTemperature(it->second);
                } else {
                    return false;
                }

                it = _parameters.find("Min Temp");
                if (it != _parameters.end()) {
                    ss->getPlanner()->as<og::LazyTRRT>()->setMinTemperature(it->second);
                } else {
                    return false;
                }

                it = _parameters.find("K Constant");
                if (it != _parameters.end()) {
                    ss->getPlanner()->as<og::LazyTRRT>()->setKConstant(it->second);
                } else {
                    return false;
                }

                it = _parameters.find("Path Length Weight");
                if (it == _parameters.end()) return false;
                ((myMWOptimizationObjective*) _opti.get())->setPathLengthWeight(it->second);


                it = _parameters.find("Frontier Nodes Ratio");
                if (it != _parameters.end()) {
                    _frontierNodesRatio = it->second;
                    ss->getPlanner()->as<og::LazyTRRT>()->setFrontierNodeRatio(_frontierNodesRatio);
                } else {
                    return false;
                }

                it = _parameters.find("Min Collision Threshold");
                if (it != _parameters.end()) {
                    ss->getPlanner()->as<og::LazyTRRT>()->setMinCollisionThreshold(it->second);
                } else {
                    return false;
                }

                it = _parameters.find("Max Collision Threshold");
                if (it != _parameters.end()) {
                    ss->getPlanner()->as<og::LazyTRRT>()->setMaxCollisionThreshold(it->second);
                } else {
                    return false;
                }

            } catch (...) {
                return false;
            }

            return true;
        }

        bool omplLazyTRRTPlanner::trySolve() {
            if (omplPlanner::trySolve()) {
                cout<<"Path cost = " << ss->getProblemDefinition()->
                      getSolutionPath()->cost(_opti).value() << endl;

                return true;
            } else {
                return false;
            }
        }
    }
}

#endif // KAUTHAM_USE_OMPL
