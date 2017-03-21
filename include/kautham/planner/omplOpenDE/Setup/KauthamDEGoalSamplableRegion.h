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

/* Author:  Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#if !defined(_KauthamDEGoalSamplableRegion_H)
#define _KauthamDEGoalSamplableRegion_H
#define dDOUBLE
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/config.h>
#include <iostream>
#include<kautham/problem/workspace.h>

#include <ode/ode.h>
//#include "omplcplanner.h"
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>
#include <kautham/sampling/sample.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


#define _USE_MATH_DEFINES


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;




using namespace std;

namespace Kautham {

/** \addtogroup Planner
 *  @{
 */
namespace omplcplanner{
typedef struct
 {
     KthReal objectposition[3];
     KthReal objectorientation[4];
 }KauthamODEobject;

class KauthamDEGoalSamplableRegion: public ob::GoalSampleableRegion
{
public:
    KauthamDEGoalSamplableRegion(const ob::SpaceInformationPtr &si, WorkSpace* ws, bool a,Sample *goal):
              ob::GoalSampleableRegion(si), stateSampler_(si->allocStateSampler())
          {
        setThreshold(0.5);
              Kauthamodebodies=smp2KauthamOpenDEState(ws,goal);
              std::cout<<"goal is "<<Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[0]<<"  "<<Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[1]<<"  "<<Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[2]<<std::endl;

              onlyend=a;
          }
    KauthamDEGoalSamplableRegion(const ob::SpaceInformationPtr &si,  WorkSpace *ws, bool a, double x, double y):
         ob::GoalSampleableRegion(si), stateSampler_(si->allocStateSampler())
    {
        setThreshold(0.5);
        //threshold_ = 1;
        KauthamODEobject odeob;

        odeob.objectposition[0]=x;
        odeob.objectposition[1]=y;
        odeob.objectposition[2] = ws->getRobot(0)->getLink(0)->getElement()->getPosition()[2];

        odeob.objectorientation[0] =ws->getRobot(0)->getLink(0)->getElement()->getOrientation()[0];
        odeob.objectorientation[1] = ws->getRobot(0)->getLink(0)->getElement()->getOrientation()[1];
        odeob.objectorientation[2] = ws->getRobot(0)->getLink(0)->getElement()->getOrientation()[2];
        odeob.objectorientation[3] = ws->getRobot(0)->getLink(0)->getElement()->getOrientation()[3];

        Kauthamodebodies.push_back(odeob);
        onlyend=a;
    }

    vector<KauthamODEobject> smp2KauthamOpenDEState(WorkSpace *wkSpace,Sample *goal);


   virtual double distanceGoal(const ob::State *st) const;
   virtual void sampleGoal(ompl::base::State *st) const;
   virtual bool isSatisfied(const ob::State *st, double *distance) const;
    vector<KauthamODEobject> Kauthamodebodies; //!<  Kauthamodebodies is a vector KauthamODEobject which keeps information on all the elements that make up the kinematic chain of the robot.
    //Kauthamodebodies es un vector de KauthamODEobject on es guarda la informació de tots els elements que formen la cadena cinemàtica del robot.
    bool onlyend; //!< It is meant to adapt to the possibility that the planner GOAL involves only the TCP.
    virtual bool isSatisfied(const ob::State *st) const;

    virtual unsigned int maxSampleCount(void) const
           {
               return 100;
           }
    ~KauthamDEGoalSamplableRegion();

private:
     ob::StateSamplerPtr stateSampler_;
};

/*The KauthamDEGoal is mainly used to set as a derived class from a GoalRegion the operation distance which the planner is going to check in order to decide wether the goal has been reached or not.
In order to check the position and orientation of the objects at the goal configuration we define a method called smp2KauthamOpenDEState .
This smp2KauthamOpenDEState method moves the robot to its goal configuration and from them extracts the position and orientation of each element in the scene and fills the KauthamOdeobject structure.
We've added the onlyend bool for further improvements as we have considered interesting to set the possibility that the goal position to be just the position of the ending point (the tcp) instead of the position of each element.
the distance is considered to be the addition of the differences between each considered element's position and its goal position
*/


}
     /** @}   end of Doxygen module "Planner */

}

#endif  //_KauthamOpenDEauxiliarclasses_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
