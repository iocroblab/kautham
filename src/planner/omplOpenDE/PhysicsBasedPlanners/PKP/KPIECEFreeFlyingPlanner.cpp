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

/* Author: Muhayyuddin */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include "KPIECEFreeFlyingPlanner.h"
//#include <sampling/state.h>
namespace Kautham {

namespace omplcplanner{


/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
KPIECE3DPlanner::KPIECE3DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    //set intial values from parent class data
    _wkSpace->moveRobotsTo(init);

    _guiName = "KPIECE 3D Planner";
    _idName = "KPIECE3DPlanner";
    dInitODE2(0);


    //       oc::OpenDEEnvironmentPtr envPtr(new KauthamDEtableEnvironment(ws,_maxspeed));
    //       stateSpace = new KauthamDEStateSpace(envPtr);
    //       stateSpacePtr = ob::StateSpacePtr(stateSpace);

     envPtr = oc::OpenDEEnvironmentPtr(new FreeFlyingRobotEnvironment (ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
    stateSpace = new freeFlyingRobotStateSpace(envPtr);
    stateSpacePtr = ob::StateSpacePtr(stateSpace);

    ss = new oc::OpenDESimpleSetup(stateSpacePtr);
    oc::SpaceInformationPtr si=ss->getSpaceInformation();
    ob::PlannerPtr planner(new oc::KPIECE1(si));
    //set planner parameters: range and goalbias
    addParameter("Goal Bias", _GoalBias);
    planner->as<oc::KPIECE1>()->setGoalBias(_GoalBias);
    //set the planner
    ss->setPlanner(planner);

}
//! void destructor
KPIECE3DPlanner::~KPIECE3DPlanner(){

}
//! this function set the necessary parameters for KAPIECE Planner.
bool KPIECE3DPlanner::setParameters()
{
    KauthamDEPlanner::setParameters();
    try{
        HASH_S_K::iterator it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<oc::KPIECE1>()->setGoalBias(_GoalBias);
        }
        else
            return false;

    }catch(...){
        return false;
    }
    return true;

}

}

}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

