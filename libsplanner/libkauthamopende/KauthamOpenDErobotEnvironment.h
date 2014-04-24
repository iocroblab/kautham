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

/* Author: Joan Fontanals Martinez, Muhayy ud din */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_KauthamOpenDErobotEnvironment_H)
#define _KauthamOpenDErobotEnvironment_H

#define dDOUBLE
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ode/ode.h>
#include <ompl/extensions/opende/OpenDEEnvironment.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <libproblem/link.h>
#include <libproblem/robot.h>
#include <libproblem/obstacle.h>
#include <libproblem/workspace.h>

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
//#include <ros/ros.h>
#include <libmt/mt/mt.h>
#include <libproblem/link.h>
#include <libproblem/robot.h>
#include <libproblem/ivelement.h>
#include "KauthamOpenDEEnvironment.h"




//#include <ompl/base/spaces/RealVectorStateSpace.h>
//#include <ompl/base/spaces/SE3StateSpace.h>
//#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
//#include <ompl/control/spaces/RealVectorControlSpace.h>
//#include <ompl/control/SpaceInformation.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include "planner.h"


using namespace std;
//using namespace libSampling;

namespace Kautham
//namespace libPlanner
{
namespace omplcplanner
{



    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class KauthamDEEnvironment
    /////////////////////////////////////////////////////////////////////////////////////////////////
//Aquesta classe està feta per definir els mètodes virtuals purs de OPENDEENVIRONMENT i que defineixen la manera com es fa el control dels cossos.
//Fins al robot no he pogut arribar al 100% a saber la manera correcta de donar les ordres i comandes als joints i motors perquè es moguin correctament.

// This class is meant to define the pure virtual methods OPENDEENVIRONMENT and define how it is controlling bodies. 
// Until the robot I could not reach 100% know the correct way to give commands and orders to joints and motors to move correctly.

class KauthamDErobotEnvironment: public KauthamDEEnvironment
{
    public:

    KauthamDErobotEnvironment(WorkSpace* ws, KthReal maxspeed);
    ~KauthamDErobotEnvironment(void);

    virtual unsigned int getControlDimension(void) const;
    virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;
    virtual void applyControl (const double *control) const;
    virtual void SetPlanningParameters();


};

}
}
#endif  //_KauthamOpenDErobotEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
