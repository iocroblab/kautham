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

#include <problem/workspace.h>
#include <sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "omplLazyTRRTplanner.h"
#include "omplValidityChecker.h"
#include "omplMyOptimizationObjective.h"
#include "lazyTRRT.h"


namespace Kautham {
    namespace omplplanner {
        //! Constructor
        omplLazyTRRTPlanner::omplLazyTRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                                 SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr) :
            omplPlanner(stype, init, goal, samples, ws, ssptr) {
            _guiName = "ompl Lazy TRRT Planner";
            _idName = "omplLazyTRRT";


            //alloc valid state sampler
            si->setValidStateSamplerAllocator(boost::bind(&omplplanner::allocValidStateSampler, _1, (Planner*)this));
            //alloc state sampler
            space->setStateSamplerAllocator(boost::bind(&omplplanner::allocStateSampler, _1, (Planner*)this));

            //create planner
            ob::PlannerPtr planner(new og::LazyTRRT(si));

            _opti = ob::OptimizationObjectivePtr(new myMWOptimizationObjective(ss->getSpaceInformation(), this, false));

            ob::ProblemDefinitionPtr pdefPtr = ss->getProblemDefinition();
            pdefPtr->setOptimizationObjective(_opti);

            planner->setProblemDefinition(pdefPtr);
            planner->setup();


            //set the planner
            ss->setPlanner(planner);

            //set planner parameters: range and goalbias
            _Range=(planner->as<og::LazyTRRT>())->getRange();
            _GoalBias=(planner->as<og::LazyTRRT>())->getGoalBias();
            _maxStatesFailed = (planner->as<og::LazyTRRT>())->getMaxStatesFailed();
            _tempChangeFactor = (planner->as<og::LazyTRRT>())->getTempChangeFactor();
            _frontierThreshold = (planner->as<og::LazyTRRT>())->getFrontierThreshold();
            _frontierNodesRatio = (planner->as<og::LazyTRRT>())->getFrontierNodeRatio();

            addParameter("Range", _Range);
            addParameter("Goal Bias", _GoalBias);
            addParameter("Max States Failed", _maxStatesFailed);
            addParameter("T Change Factor", _tempChangeFactor);
            addParameter("Frontier Threshold", _frontierThreshold);
            addParameter("Frontier Nodes Ratio", _frontierNodesRatio);
            addParameter("Path Length Weight",0.00001);
            addParameter("Min Temp",planner->as<og::LazyTRRT>()->getMinTemperature());
            addParameter("Init Temp",planner->as<og::LazyTRRT>()->getInitTemperature());
            addParameter("K Constant",planner->as<og::LazyTRRT>()->getKConstant());
        }

        //! void destructor
        omplLazyTRRTPlanner::~omplLazyTRRTPlanner() {

        }


        bool omplLazyTRRTPlanner::setPotentialCost(string filename) {
            return ((myMWOptimizationObjective*) _opti.get())->setPotentialCost(filename);
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
            } catch (...) {
                return false;
            }

            return true;
        }

        bool omplLazyTRRTPlanner::trySolve()
        {
            if (omplPlanner::trySolve())
            {
                cout<<"Path cost = " << ss->getProblemDefinition()->getSolutionPath()->cost(_opti).v << endl;

                return true;
            }
            else
            {
                return false;
            }
        }
        }
    }


#endif // KAUTHAM_USE_OMPL
