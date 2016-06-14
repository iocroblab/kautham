/*************************************************************************\
   Copyright 2015 Institute of Industrial and Control Engineering (IOC)
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

#include <problem/workspace.h>
#include <sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "RRTKUKAPlanner.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>


#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>

using namespace std;

namespace Kautham {

namespace omplcplanner{



  //! void destructor
  RRTKUKAPlanner::~RRTKUKAPlanner(){

  }

   RRTKUKAPlanner::RRTKUKAPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws) : KauthamDEPlanner(stype, init, goal, samples, ws)
   {


     _guiName = "RRT KUKA Planner";
     _idName = "RRTKUKAPlanner";
     //ws->moveRobotsTo(init);
     dInitODE2(0);
      envPtr = oc::OpenDEEnvironmentPtr(new KUKAEnvironment(ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm, _isKchain));
     stateSpace = new KUKAStateSpace(envPtr);
     stateSpacePtr = ob::StateSpacePtr(stateSpace);

     ss = new oc::OpenDESimpleSetup(stateSpacePtr);
     oc::SpaceInformationPtr si=ss->getSpaceInformation();
     ob::PlannerPtr planner(new oc::RRT(si));
     addParameter("Goal Bias", _GoalBias);
     planner->as<oc::RRT>()->setGoalBias(_GoalBias);

//     //ob::PlannerPtr planner(new oc::KPIECE1(si));
//     _GoalBias=(planner->as<oc::KPIECE1>())->getGoalBias();
//     addParameter("Goal Bias", _GoalBias);
//     planner->as<oc::KPIECE1>()->setGoalBias(_GoalBias);
     //set the planner
     ss->setPlanner(planner);
     //std::cout<<"total bodies are "<<envPtr->stateBodies_.size()<<std::endl;

   }
 bool RRTKUKAPlanner::setParameters()
 {
     KauthamDEPlanner::setParameters();
     try{
         HASH_S_K::iterator it = _parameters.find("Goal Bias");
         if(it != _parameters.end()){
             _GoalBias = it->second;
             ss->getPlanner()->as<oc::RRT>()->setGoalBias(_GoalBias);
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


