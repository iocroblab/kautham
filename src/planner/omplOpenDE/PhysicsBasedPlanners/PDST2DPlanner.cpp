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

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/PDST2DPlanner.h>

namespace Kautham {

namespace omplcplanner{
class myProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    myProjectionEvaluator(const ob::StateSpacePtr &space, const std::vector<double> &cellSizes, WorkSpace* _wkSpace) : ob::ProjectionEvaluator(space)
    {
        setCellSizes(cellSizes);
        bounds_.resize(2);
        //const ob::RealVectorBounds& spacebounds = space->as<ob::RealVectorStateSpace>()->getBounds();
        bounds_.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
        bounds_.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
        bounds_.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
        bounds_.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
    }

    virtual unsigned int getDimension(void) const
    {
        return 2;
    }

    virtual void project(const ob::State *state, Kautham::VectorRef projection) const override
    {
        const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);

        projection(0) = pos[0];
        projection(1) = pos[1];
    }
};

/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
PDST2DPlanner::PDST2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    //set intial values from parent class data
    _wkSpace->moveRobotsTo(init);

    _guiName = "PDST 2D Planner";
    _idName = "PDST2DPlanner";
    dInitODE2(0);


     envPtr = oc::OpenDEEnvironmentPtr(new twoDRobotEnvironment (ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
    stateSpace = new twoDRobotStateSpace(envPtr);
    stateSpacePtr = ob::StateSpacePtr(stateSpace);

//    oc::ControlSpacePtr csp(new KauthamControlSpace(stateSpacePtr));
//    ss = new oc::OpenDESimpleSetup(csp);
    ss = new oc::OpenDESimpleSetup(stateSpacePtr);

    oc::SpaceInformationPtr si=ss->getSpaceInformation();
    ob::PlannerPtr planner(new oc::PDST(si));

    std::vector<double> cdim;
    cdim.push_back(0.01);
    cdim.push_back(0.01);
    ob::ProjectionEvaluatorPtr ope(new myProjectionEvaluator(si->getStateSpace(), cdim,_wkSpace));

    planner->as<oc::PDST>()->setProjectionEvaluator(ope);

    //planner->as<oc::PDST>()->setProjectionEvaluator(si->getStateSpace()->getDefaultProjection());
    ss->getSpaceInformation()->setMinMaxControlDuration(1,50);

    //set the planner
    ss->setPlanner(planner);

}
//! void destructor
PDST2DPlanner::~PDST2DPlanner(){

}
//! this function set the necessary parameters for PDST Planner.
bool PDST2DPlanner::setParameters()
{
    KauthamDEPlanner::setParameters();
    try{
        HASH_S_K::iterator it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            //ss->getPlanner()->as<oc::PDST>()->setGoalBias(_GoalBias);
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


