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

/* Author: Joan Fontanals Martinez, Muhayyuddin */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include <kautham/planner/omplOpenDE/KauthamOpenDERRTPlanner.h>
#include <kautham/sampling/state.h>
namespace Kautham {

namespace omplcplanner{

//EL planner es basa en l'exemple de OpenDERigidBodyPlanning.
//Es crea el Environment,el State Space, i el simple setup. Se li diu que el goal es de la classe KAuthamDEGoal i es planifica mitjançant el simple setup després d'haverli declarat el planner i els bounds.

// The planner is based on the example of OpenDERigidBodyPlanning. 
// Create the Environment, the State Space, and simple setup. It is said that the Goal class KAuthamDEGoal and planning through simple setup after haverli declared planner and Bounds.
/*! Constructor create the dynamic environment, and setup all the parameters for planning.
 * it defines simple setup, Planner and Planning parameters.
 */
KauthamDERRTPlanner::KauthamDERRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
      KauthamDEPlanner(stype, init, goal, samples, ws)
  {
      //set intial values from parent class data
      _guiName = "ompl ODE RRT Planner";
      _idName = "omplODERRTPlanner";
       dInitODE2(0);

     ss = new oc::OpenDESimpleSetup(stateSpacePtr);
     oc::SpaceInformationPtr si=ss->getSpaceInformation();
     ob::PlannerPtr planner(new oc::RRT(si));

     //set RRT Ggoal Bias
     _GoalBias=(planner->as<oc::RRT>())->getGoalBias();
     addParameter("Goal Bias", _GoalBias);
     planner->as<oc::RRT>()->setGoalBias(_GoalBias);

     //set the planner
     ss->setPlanner(planner);

}
//! void destructor
  KauthamDERRTPlanner::~KauthamDERRTPlanner(){

  }
//! this function set the necessary parameters for RRT Planner.
  bool KauthamDERRTPlanner::setParameters()
  {
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


