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


#include <kautham/planner/omplg/omplFOSRRTConnectplanner.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <kautham/planner/omplg/FOSOptimizationObjective.h>


using namespace Kautham::omplplanner;


//! Constructor
omplFOSRRTConnectPlanner::omplFOSRRTConnectPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                                   SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr) :
    omplPlanner(stype, init, goal, samples, ws, ssptr) {
    _guiName = "ompl FOSRRTConnect Planner";
    _idName = "omplFOSRRTConnect";

    //set planner
    ob::PlannerPtr planner(new og::RRTConnect(si));
    ss->setPlanner(planner);
    planner->setup();

    //set the optimization objective
    opt_ = ob::OptimizationObjectivePtr(new ob::FOSOptimizationObjective(this,ss->getSpaceInformation()));
    ss->getProblemDefinition()->setOptimizationObjective(opt_);

    //set planner parameters:
    addParameter("Range", planner->as<og::RRTConnect>()->getRange());
}


//! void destructor
omplFOSRRTConnectPlanner::~omplFOSRRTConnectPlanner(){
}


bool omplFOSRRTConnectPlanner::setSynergyTree(string filename) {
    return ((ob::FOSOptimizationObjective*)opt_.get())->setSynergyTree(filename);
}


//! setParameters sets the parameters of the planner
bool omplFOSRRTConnectPlanner::setParameters(){

    omplPlanner::setParameters();
    try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if (it == _parameters.end()) return false;
        if (it->second >= 0.0) {
            ss->getPlanner()->as<og::RRTConnect>()->setRange(it->second);
        } else {
            it->second = ss->getPlanner()->as<og::RRTConnect>()->getRange();
        }
    } catch(...) {
        return false;
    }
    return true;
}


bool omplFOSRRTConnectPlanner::trySolve() {
    if (omplPlanner::trySolve()) {
        ob::Cost pathcost = ss->getProblemDefinition()->getSolutionPath()->cost(opt_);
        cout << "Path cost = " << pathcost.value() << endl;

        return true;
    } else {
        return false;
    }
}
