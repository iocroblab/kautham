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


#include <kautham/planner/omplg/omplTRRTConnectplanner.h>
#include <kautham/planner/omplg/TRRTConnect.h>
#include <kautham/planner/omplg/omplMyOptimizationObjective.h>


using namespace Kautham::omplplanner;


omplTRRTConnectPlanner::omplTRRTConnectPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                               SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr) :
    omplPlanner(stype,init,goal,samples,ws,ssptr) {
    _guiName = "ompl TRRTConnect Planner";
    _idName = "omplTRRTConnect";

    //set the planner
    //planner setup done after setting the potential cost
    og::TRRTConnect *planner(new og::TRRTConnect(si));
    planner->setProblemDefinition(ss->getProblemDefinition());
    ss->setPlanner(ob::PlannerPtr(planner));

    //set the optimization objective
    opt_ = ob::OptimizationObjectivePtr(new myMWOptimizationObjective(ss->getSpaceInformation(),this));
    ss->getProblemDefinition()->setOptimizationObjective(opt_);

    //set planner parameters
    //range, kConstant and frontierThreshold set after setting the potential cost
    addParameter("Max States Succeed",planner->getMaxStatesSucceed());
    addParameter("Max States Failed",planner->getMaxStatesFailed());
    addParameter("Temp Change Factor",planner->getTempChangeFactor());
    addParameter("Frontier Node Ratio",planner->getFrontierNodeRatio());
    addParameter("Min Temp",planner->getMinTemperature());
    addParameter("Init Temp",planner->getInitTemperature());
    addParameter("Path Length Weight",((myMWOptimizationObjective*)opt_.get())->getPathLengthWeight());
}


omplTRRTConnectPlanner::~omplTRRTConnectPlanner() {
}


bool omplTRRTConnectPlanner::setPotentialCost(string filename) {
    bool result = ((myMWOptimizationObjective*)opt_.get())->setPotentialCost(filename);

    og::TRRTConnect *planner(ss->getPlanner()->as<og::TRRTConnect>());
    planner->setup();
    addParameter("Range",planner->getRange());
    addParameter("K Constant",planner->getKConstant());
    addParameter("Frontier Threshold",planner->getFrontierThreshold());

    return result;
}


bool omplTRRTConnectPlanner::setParameters() {
    if (!omplPlanner::setParameters()) return false;

    try {
        HASH_S_K::iterator it;
        og::TRRTConnect *planner(ss->getPlanner()->as<og::TRRTConnect>());

        it = _parameters.find("Range");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setRange(it->second);
            if (it->second <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                space->setLongestValidSegmentFraction(it->second/_validSegmentCount/space->getMaximumExtent());
                space->setup();
            }
        } else {
            it->second = planner->getRange();
        }

        it = _parameters.find("K Constant");
        if (it == _parameters.end()) return false;
        planner->setKConstant(it->second);

        it = _parameters.find("Frontier Threshold");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setFrontierThreshold(it->second);
        } else {
            it->second = planner->getFrontierThreshold();
        }

        it = _parameters.find("Max States Succeed");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setMaxStatesSucceed(it->second);
        } else {
            it->second = planner->getMaxStatesSucceed();
        }

        it = _parameters.find("Max States Failed");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setMaxStatesFailed(it->second);
        } else {
            it->second = planner->getMaxStatesFailed();
        }

        it = _parameters.find("Temp Change Factor");
        if (it == _parameters.end()) return false;
        if (it->second >= 1.0) {
            planner->setTempChangeFactor(it->second);
        } else {
            it->second = planner->getTempChangeFactor();
        }

        it = _parameters.find("Frontier Node Ratio");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setFrontierNodeRatio(it->second);
        } else {
            it->second = planner->getFrontierNodeRatio();
        }

        it = _parameters.find("Min Temp");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            planner->setMinTemperature(it->second);
        } else {
            it->second = planner->getMinTemperature();
        }

        it = _parameters.find("Init Temp");
        if (it == _parameters.end()) return false;
        if (it->second > planner->getMinTemperature()) {
            planner->setInitTemperature(it->second);
        } else {
            it->second = planner->getInitTemperature();
        }

        it = _parameters.find("Path Length Weight");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            ((myMWOptimizationObjective*)opt_.get())->setPathLengthWeight(it->second);
        } else {
            it->second = ((myMWOptimizationObjective*)opt_.get())->getPathLengthWeight();
        }
    } catch (...) {
        return false;
    }

    return true;
}

bool omplTRRTConnectPlanner::trySolve() {
    if (omplPlanner::trySolve()) {
        ob::Cost pathcost = ss->getProblemDefinition()->getSolutionPath()->cost(opt_);
        cout << "Path cost = " << pathcost.value() << endl;

        return true;
    } else {
        return false;
    }
}
