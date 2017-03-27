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
#if !defined(_twoDRobotEnvironment_H)
#define _twoDRobotEnvironment_H
#define dDOUBLE
#define _USE_MATH_DEFINES

#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ode/ode.h>
#include <ompl/config.h>
#include <kautham/problem/link.h>
#include <kautham/problem/robot.h>
#include <kautham/problem/workspace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/extensions/opende/OpenDEEnvironment.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
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
#include <kautham/problem/link.h>
#include <kautham/problem/robot.h>
#include <kautham/problem/ivelement.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/planner.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>

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
//!This class defines the pure virtual and virtual function of OpenDeEnviroment class for the 2DRobotEnvironment. It defines the control dimension for the robot,
//!control bounds, how the control will applied to the robot (such as in term of forces or velocities), how the robot will interact with
//!the environment (by defining isValidCollision), and the contact dynamics.
class twoDRobotEnvironment: public KauthamDEEnvironment
{
public:

    twoDRobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,
                                KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain);//!< Constructor define the 2D robot environment by calling the KauthamDEEnvironment.
    ~twoDRobotEnvironment(void);
    std::string robBase;
    std::string floor;
    virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.
    virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, minimum and maximum control range.
    virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities or torques.
    virtual bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact& /*contact*/) const ;//!< This function defines the validity of the collisions.
    virtual void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const; //!< This method set the parameters for the contact.

};
/////////////////////////////////////////////////////////////////////////////////
///                        2D Robot State Space
/////////////////////////////////////////////////////////////////////////////////

/*! The KauthamDEStateSpace intherits from OpenDEStateSpace and just defines the method distance and the registerprojections.
 * An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 * position,linear and angular velocity and then a SO3 that represents the orientation
 */
class twoDRobotStateSpace : public oc::OpenDEStateSpace
{
public:
    twoDRobotStateSpace(const oc::OpenDEEnvironmentPtr &env);//!< Constructor
    ~twoDRobotStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.
};
/////////////////////////////////////////////////////////////////////////////////
///                   2D Robot Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
/*! this class define how the state will be projected. this class inherit from the
 * ProjectionEvaluator and define the virtual functions.
 */
class twoDRobotStateProjectionEvaluator: public ob::ProjectionEvaluator
{
public:
    twoDRobotStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    twoDRobotStateProjectionEvaluator(const ob::StateSpace *space, WorkSpace *_wkSpace); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const;//!< This function calculate the projections
};

class twoDControlSampler : public oc::RealVectorControlUniformSampler
{
public:

    twoDControlSampler(const oc::ControlSpace *cm) ;
    virtual void sampleNext(oc::Control *control, const oc::Control *previous);
    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/);

};

class twoDControlSpace : public oc::OpenDEControlSpace
{
public:
    twoDControlSpace(const ob::StateSpacePtr &m);
    virtual oc::ControlSamplerPtr allocControlSampler(void) const;

};

}
/** @}   end of Doxygen module "Planner */
}
#endif  //2DRobotEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
