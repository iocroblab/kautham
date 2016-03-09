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
#if !defined(_RRTTX90Planner_H)
#define _RRTTX90Planner_H
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
#include <problem/link.h>
#include <problem/robot.h>
//#include <problem/obstacle.h>
#include <problem/workspace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>


#define _USE_MATH_DEFINES

#include <math.h>
//#include <pugixml.hpp>
#include <fstream>
#include <iostream>
#include <Inventor/fields/SoSFVec3f.h>
#include <Inventor/fields/SoSFRotation.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SbLinear.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <vector>
#include <cmath>
//#include <libmt/mt/mt.h>
#include <problem/link.h>
#include <problem/robot.h>
#include <problem/ivelement.h>
#include "../../environment/KauthamOpenDEEnvironment.h"
#include "../../environment/SimpleRobotEnvironment/TX90Environment.h"
#include <ompl/base/ProjectionEvaluator.h>
#include <problem/workspace.h>
#include <sampling/sampling.h>
#include "planner/planner.h"
#include "../KauthamOpenDEPlanner.h"
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
    class RRTTX90Planner: public KauthamDEPlanner
    {
    public:
        //! The constructor will define all the necessary parameters for planning.
        RRTTX90Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
        ~RRTTX90Planner();

        bool setParameters();//!< set the planning parameters.
        KthReal _GoalBias;
    };

}

 /** @}   end of Doxygen module "Planner */
}
#endif  //_KauthamOpenDERRTTX90Planner_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

