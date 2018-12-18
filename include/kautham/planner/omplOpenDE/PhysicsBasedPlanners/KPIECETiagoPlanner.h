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
#if !defined(_KPIECETiagoPlanner_H)
#define _KPIECETiagoPlanner_H
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#define dDOUBLE
#include <ode/ode.h>
#include <ompl/extensions/opende/OpenDEEnvironment.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <kautham/problem/link.h>
#include <kautham/problem/robot.h>
//#include <problem/obstacle.h>
#include <kautham/problem/workspace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>


#define _USE_MATH_DEFINES

#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>
#include <kautham/planner/omplOpenDE/Setup/TiagoEnvironment.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/planner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


using namespace std;

namespace Kautham
{
/** \addtogroup Planner
 *  @{
 */
namespace omplcplanner
{
    class KPIECETiagoPlanner: public KauthamDEPlanner
    {
    public:
        //! The constructor will define all the necessary parameters for planning.
        KPIECETiagoPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
        ~KPIECETiagoPlanner();

        bool setParameters();//!< set the planning parameters.
        KthReal _GoalBias;
    };

}

 /** @}   end of Doxygen module "Planner */
}
#endif  //_KauthamOpenDERRTTX90Planner_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

