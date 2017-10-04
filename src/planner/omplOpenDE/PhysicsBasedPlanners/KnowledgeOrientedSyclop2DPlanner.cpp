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

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedSyclop2DPlanner.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace Kautham {

namespace omplcplanner{
enum MotionModel { Motion_2D, Motion_3D };

class MyDecomposition : public oc::GridDecomposition
{
public:
    MyDecomposition(const int length, const ob::RealVectorBounds& bounds)
        : GridDecomposition(length, 2, bounds)
    {
    }
    virtual void project(const ob::State* s, std::vector<double>& coord) const
    {
        coord.resize(2);
        coord[0]=s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        coord[1]=s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        //const double *pos = s->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        //coord[0] =  pos[0];
        //coord[1] =  pos[1];
        //std::cout<<"Cord are : "<<coord[0]<<" "<<coord[1]<<std::endl;

    }
    virtual void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
    {
        sampler->sampleUniform(s);
        s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = coord[0];
        s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = coord[1];

    }

};

/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
KnowledgeOrientedSyclop2DPlanner::KnowledgeOrientedSyclop2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    //set intial values from parent class data
    _wkSpace->moveRobotsTo(init);
    ob::RealVectorBounds bounds(2);

    _guiName = "Knowledge Oriented Syclop 2D Planner";
    _idName = "KnowledgeOrientedSyclop2DPlanner";
    bounds.resize(2);
    dInitODE2(0);


    //       oc::OpenDEEnvironmentPtr envPtr(new KauthamDEtableEnvironment(ws,_maxspeed));
    //       stateSpace = new KauthamDEStateSpace(envPtr);
    //       stateSpacePtr = ob::StateSpacePtr(stateSpace);

    envPtr = oc::OpenDEEnvironmentPtr(new ConstraintAware2DRobotEnvironment(ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
   stateSpace = new ConstraintAwaretwoDRobotStateSpace(envPtr);
   stateSpacePtr = ob::StateSpacePtr(stateSpace);
   oc::ControlSpacePtr csp(new ConstraintAwaretwoDControlSpace(stateSpacePtr));
   ss = new oc::OpenDESimpleSetup(csp);

//    oc::ControlSpacePtr csp(new KauthamControlSpace(stateSpacePtr));
//    ss = new oc::OpenDESimpleSetup(csp);
    //ss = new oc::OpenDESimpleSetup(stateSpacePtr);

    oc::SpaceInformationPtr si=ss->getSpaceInformation();


    bounds.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
//    // Create a 10x10 grid decomposition for Syclop
//    control::DecompositionPtr decomp(new SyclopDecomposition (10, bounds));

//    control::SyclopRRT *srrt = new control::SyclopRRT(si, decomp);
    oc::DecompositionPtr decomp(new MyDecomposition(10,bounds));

    //oc::DecompositionPtr allocDecomposition(stateSpacePtr, Motion_2D, stateSpacePtr);
    ob::PlannerPtr planner(new oc::SyclopRRT(si,decomp));
    //set planner parameters: range and goalbias

//    _NumFreeVolumeSamples=(planner->as<oc::SyclopRRT>())->getNumFreeVolumeSamples();
//    _getProbShortestPathLead=(planner->as<oc::SyclopRRT>())->getProbShortestPathLead();
//    _getProbAddingToAvailableRegions=(planner->as<oc::SyclopRRT>())->getProbAddingToAvailableRegions();
//    _getNumRegionExpansions=(planner->as<oc::SyclopRRT>())->getNumRegionExpansions ();
//    _getNumTreeExpansions=(planner->as<oc::SyclopRRT>())->getNumTreeExpansions();
//    _getProbAbandonLeadEarly= (planner->as<oc::SyclopRRT>())->getProbAbandonLeadEarly();

//   planner->as<oc::SyclopRRT>()->setNumFreeVolumeSamples(_NumFreeVolumeSamples);
//   planner->as<oc::SyclopRRT>()->setProbShortestPathLead(_getProbShortestPathLead);
//   planner->as<oc::SyclopRRT>()->setProbAddingToAvailableRegions(_getProbAddingToAvailableRegions);
//   planner->as<oc::SyclopRRT>()->setNumRegionExpansions(_getNumRegionExpansions);
//   planner->as<oc::SyclopRRT>()->setNumTreeExpansions(_getNumTreeExpansions);
//   planner->as<oc::SyclopRRT>()->setProbAbandonLeadEarly(_getProbAbandonLeadEarly);
    //set the planner

    // set the problem we are trying to solve for the planner
    ss->setPlanner(planner);


}
//! void destructor
KnowledgeOrientedSyclop2DPlanner::~KnowledgeOrientedSyclop2DPlanner(){

}
//! this function set the necessary parameters for SyCLoP Planner.
bool KnowledgeOrientedSyclop2DPlanner::setParameters()
{
    KauthamDEPlanner::setParameters();
    try{
        HASH_S_K::iterator it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            //ss->getPlanner()->as<oc::SyclopRRT>()->setGoalBias(_GoalBias);
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


