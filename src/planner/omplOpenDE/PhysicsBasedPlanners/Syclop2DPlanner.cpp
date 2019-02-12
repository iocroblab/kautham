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

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/Syclop2DPlanner.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <kautham/planner/omplOpenDE/Setup/PlanarChainEnvironment.h>
namespace Kautham {

namespace omplcplanner{
enum MotionModel { Motion_2D, Motion_3D };

//!This class provide the decomposition of the workspace for SyCLoP planner.
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
        const double *pos = s->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
        coord[0] =  pos[0];
        coord[1] =  pos[1];
    }
    virtual void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
    {
        sampler->sampleUniform(s);
        s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(12)->values[0] = coord[0];
        s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(12)->values[1] = coord[1];

    }

};

/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
Syclop2DPlanner::Syclop2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    //set intial values from parent class data
    _wkSpace->moveRobotsTo(init);
    ob::RealVectorBounds bounds(2);

    _guiName = "Syclop 2D Planner";
    _idName = "Syclop2DPlanner";
    bounds.resize(2);
    dInitODE2(0);

    envPtr= oc::OpenDEEnvironmentPtr(new PlanarChainEnvironment(ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
    //stateSpacePtr = ob::StateSpacePtr(new PlanarChainStateSpace(envPtr));
    stateSpacePtr = ob::StateSpacePtr(new PlanarChainStateSpace(envPtr));
    csp= oc::ControlSpacePtr(new PlanarChainControlSpace(stateSpacePtr));
    ss = new oc::OpenDESimpleSetup(csp);
    oc::SpaceInformationPtr si=ss->getSpaceInformation();

    bounds.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
    // Create a 10x10 grid decomposition for Syclop
    oc::DecompositionPtr decomp(new MyDecomposition(10,bounds));
    ob::PlannerPtr planner(new oc::SyclopRRT(si,decomp));

    //set the planner

    ss->setPlanner(planner);


}
//! void destructor
Syclop2DPlanner::~Syclop2DPlanner(){

}
//! this function set the necessary parameters for SyCLoP Planner.
bool Syclop2DPlanner::setParameters()
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


