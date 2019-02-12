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

//#if defined(KAUTHAM_USE_OMPL)
//#if defined(KAUTHAM_USE_ODE)

#if !defined(_KauthamDEGoalRegion_H)
#define _KauthamDEGoalRegion_H
#define dDOUBLE
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/config.h>
#include <iostream>
#include <kautham/problem/workspace.h>

#include <ode/ode.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>
#include <kautham/sampling/sample.h>


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

/*The KauthamDEGoal is mainly used to set as a derived class from a GoalRegion the operation distance which the planner is going to check in order to decide wether the goal has been reached or not.
In order to check the position and orientation of the objects at the goal configuration we define a method called smp2KauthamOpenDEState .
This smp2KauthamOpenDEState method moves the robot to its goal configuration and from them extracts the position and orientation of each element in the scene and fills the KauthamOdeobject structure.
We've added the onlyend bool for further improvements as we have considered interesting to set the possibility that the goal position to be just the position of the ending point (the tcp) instead of the position of each element.
the distance is considered to be the addition of the differences between each considered element's position and its goal position
*/
//! The KauthamDEGoal is mainly used to set as a derived class from a GoalRegion the operation distance which the planner is going to check in order to decide wether the goal has been reached or not.
class KauthamDEGoalRegion : public ob::GoalRegion
{
public:
    KauthamDEGoalRegion(const ob::SpaceInformationPtr &si, WorkSpace* ws, bool a,Sample *goal);
    KauthamDEGoalRegion(const ob::SpaceInformationPtr &si,  WorkSpace *ws, bool a, double x, double y);
    ~KauthamDEGoalRegion();
    virtual double distanceGoal(const ob::State *st) const; //!< This function compute the distance from the goal.

/*! This structure keeps the position and orientation of each element of the robot once the configuration has been moved GOAL.
 *This information can be used to compute the distance of any configuration.
 */
    typedef struct
    {
        KthReal objectposition[3];
        KthReal objectorientation[4];
    }KauthamODEobject;
    vector<KauthamODEobject> smp2KauthamOpenDEState(WorkSpace* w,Sample *goal);//!< This function check the position and orientation of the objects at the goal configuration.
    vector<KauthamODEobject> Kauthamodebodies; //!<  Kauthamodebodies is a vector KauthamODEobject which keeps information on all the elements that make up the kinematic chain of the robot.
    bool onlyend; //!< It is meant to adapt to the possibility that the planner GOAL involves only the TCP.
    virtual bool isSatisfied(const ob::State *st, double *distance) const;
    virtual bool isSatisfied(const ob::State *st) const;
};

}
/** @}   end of Doxygen module "Planner */

}

#endif  //_KauthamDEGoalRegion_H
//#endif //KAUTHAM_USE_ODE
//#endif // KAUTHAM_USE_OMPL
