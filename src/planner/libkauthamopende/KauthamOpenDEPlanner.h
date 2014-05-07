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
#if !defined(_KauthamOpenDEplanner_H)
#define _KauthamOpenDEplanner_H
#define dDOUBLE
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/config.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <libompl/omplcplanner.h>

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/config.h>
#include <iostream>

#include <ode/ode.h>
#include "KauthamOpenDEauxiliarclasses.h"
#include <libompl/omplcplanner.h>
#include "planner.h"
#include "KauthamOpenDEEnvironment.h"

#include "KauthamOpenDEtableEnvironment.h"


#define _USE_MATH_DEFINES

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;

namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */
namespace omplcplanner{


/////////////////////////////////////////////////////////////////////////////////////////////////
// Class KauthamOpenDEPlanner
/////////////////////////////////////////////////////////////////////////////////////////////////

/*! KauthanDEplanner is the base class for all the planner that use the dynamic enviroment for planning.
 * All the planners will be drived from this class and reimplement the trysolve function.
 */
class KauthamDEPlanner: public Planner
{
public:
    //! The constructor will define all the necessary parameters for planning.
    KauthamDEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
    ~KauthamDEPlanner();
    double _propagationStepSize; //!< Define the step size of the world.
    KthReal _maxspeed; //!< describe the max. speed of motors.
    bool _onlyend; //!< describe that only TCP will move of complete robot.
    double _planningTime; //!< describe the max. planning time.
    //oc::SimpleSetupPtr ss;
    ob::StateSpacePtr stateSpacePtr; //!< state space pointer to KauthamDEStateSpace.
    oc::OpenDEEnvironmentPtr envPtr; //!< pointer to KauthamDE ENviroment.
    KauthamDEStateSpace *stateSpace; //!< pointer to kauthamDEStatespace.
    oc::OpenDESimpleSetup *ss;
    virtual bool trySolve();//!< Compute the path and returns the boolean value.
    bool setParameters();//!< set the planning parameters.
    int m;
};

}

}
//}
#endif  //_KauthamOpenDEplanner_H
#endif  //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

