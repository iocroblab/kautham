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
#if !defined(_KauthamOpenDEtableEnvironment_H)
#define _KauthamOpenDEtableEnvironment_H
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
#include <kautham/problem/workspace.h>

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
#include <kautham/planner/omplOpenDE/KauthamOpenDEEnvironment.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include "planner/planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


using namespace std;

namespace Kautham
{
/** \addtogroup group4 Physics-based Planners
 *  @{
 */
namespace omplcplanner
{



    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class KauthamDEEnvironment
    /////////////////////////////////////////////////////////////////////////////////////////////////

    //Aquesta classe està feta per definir els mètodes virtuals purs de OPENDEENVIRONMENT i que defineixen la manera com es fa el control dels cossos.
    //No sé fins a quin punt es 100% correcte però en aquest cas a la taula el control actua sobre la velocitat lineal del cos de la taula.

    // This class is meant to define the pure virtual methods OPENDEENVIRONMENT and define how it is controlling bodies.
    // I do not know how far you are 100% correct but in this case the control board acts on the linear velocity of the body of the table.

   //! This class define the pure virtual methods of OpenDEEnviroment and define how the control will be applied on the bodies.
    class KauthamDEtableEnvironment: public KauthamDEEnvironment
    {
        public:

        KauthamDEtableEnvironment(WorkSpace* ws, KthReal maxspeed);//!< Constructor define the robot environment(i.e. table environment ) by calling the KauthamDEEnvironment.
        ~KauthamDEtableEnvironment(void);

        virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.
        virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, the bounding box to performe sampling
        virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities and torques.
        virtual void SetPlanningParameters();//!< Set the planning parameters for planner.
        //virtual bool    isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const;
        virtual void setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const; //!< This method set the parameters for the contact.

    };


}

 /** @}   end of Doxygen module "Planner */
}
#endif  //_KauthamOpenDEtableEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

