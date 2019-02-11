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

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedKPIECE2DPlanner.h>

namespace Kautham {

namespace omplcplanner{


/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
KnowledgeOrientedKPIECE2DPlanner::KnowledgeOrientedKPIECE2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    _wkSpace->moveRobotsTo(init);

    _guiName = "Knowledge Oriented KPIECE 2D Planner";
    _idName = "KnowledgeOrientedKPIECE2DPlanner";
    dInitODE2(0);


    //       oc::OpenDEEnvironmentPtr envPtr(new KauthamDEtableEnvironment(ws,_maxspeed));
    //       stateSpace = new KauthamDEStateSpace(envPtr);
    //       stateSpacePtr = ob::StateSpacePtr(stateSpace);

     envPtr = oc::OpenDEEnvironmentPtr(new ConstraintAware2DRobotEnvironment(ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
    stateSpace = new ConstraintAwaretwoDRobotStateSpace(envPtr);
    stateSpacePtr = ob::StateSpacePtr(stateSpace);
    //oc::ControlSpacePtr csp(new ConstraintAwaretwoDControlSpace(stateSpacePtr));
    //ss = new oc::OpenDESimpleSetup(csp);
     ss = new oc::OpenDESimpleSetup(stateSpacePtr);
    oc::SpaceInformationPtr si=ss->getSpaceInformation();

    ob::PlannerPtr planner(new oc::KPIECE1(si));
    //set planner parameters: range and goalbias
    addParameter("Goal Bias", _GoalBias);
    planner->as<oc::KPIECE1>()->setGoalBias(_GoalBias);
    //planner->as<oc::KPIECE1>()->setProjectionEvaluator(stateSpacePtr->getDefaultProjection());

    //set the planner
    ss->setPlanner(planner);

}
//! void destructor
KnowledgeOrientedKPIECE2DPlanner::~KnowledgeOrientedKPIECE2DPlanner(){

}
//! this function set the necessary parameters for KAPIECE Planner.
bool KnowledgeOrientedKPIECE2DPlanner::setParameters()
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


