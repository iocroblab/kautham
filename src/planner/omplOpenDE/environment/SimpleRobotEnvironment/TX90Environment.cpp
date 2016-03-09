
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

/* Author: Muhayyuddin  */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#include "TX90Environment.h"

namespace Kautham {
namespace omplcplanner{


//! Constructor create the ODE table enviroment and setup the parameters for ODE.
TX90Environment::TX90Environment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
{
    SetPlanningParameters();
}
TX90Environment::~TX90Environment()
{

}
//! Setup the parameters for ODE.
//void KauthamDETX90Environment::SetPlanningParameters()
//{
//    stepSize_ =  _propagationStepSize;
//    maxContacts_ = _maxContacts;
//    minControlSteps_ = _minControlSteps;
//    maxControlSteps_ = _maxControlSteps;
//}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
unsigned int TX90Environment::getControlDimension(void) const
{
    return 3;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
* which describe the control bounds,the bounding box to performe sampling control.
*/
void TX90Environment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    for(int i=0; i < 3; i++)
    {
        lower[i]=-10;
        upper[i]=10;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
* that explain how the control will apply.This function apply the control by
* setting the forces, velocities and torques.
*/
void TX90Environment::applyControl (const double *control) const
{
//    for(int i=0;i<_motor.size();i++)
//    {
////        dJointSetHingeParam(_Joint[i],dParamVel,control[i]);
////        dJointSetHingeParam(_Joint[i],dParamFMax,dInfinity);
//    dJointSetAMotorParam(_motor[i], dParamVel, control[i]);
//    dJointSetAMotorParam(_motor[i], dParamFMax, dInfinity);
////        dReal ang =dJointGetHingeAngle(_Joint[i]);
////        std::cout<<"angle is:  "<<ang<<std::endl;
//}
    //std::cout<<std::endl;
    dJointSetAMotorParam(_motor[0], dParamVel, control[0]);
    dJointSetAMotorParam(_motor[0], dParamFMax, dInfinity);
    dJointSetAMotorParam(_motor[1], dParamVel, control[1]);
    dJointSetAMotorParam(_motor[1], dParamFMax, dInfinity);
    dJointSetAMotorParam(_motor[2], dParamVel, control[2]);
    dJointSetAMotorParam(_motor[2], dParamFMax, dInfinity);
}

bool TX90Environment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{

    return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment
* This method set the parameters for the contact, like what will be the value
* of friction coefficient, etc.
*/
void TX90Environment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    dBodyID body1 = dGeomGetBody(geom1);
    dBodyID body2 = dGeomGetBody(geom2);
    contact.surface.mode = 0;
        contact.surface.mu = 0.5;

}

/////////////////////////////////////////////////////////////////////////////////
///                     TX90 State Space
/////////////////////////////////////////////////////////////////////////////////
TX90StateSpace::TX90StateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
TX90StateSpace::~TX90StateSpace(){}

double TX90StateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;

    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    double dz = fabs(p1[1] - p2[1]);

    distance = distance + sqrt(dx * dx + dy * dy  + dz *dz);

    return distance;
}

void TX90StateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new TX90StateProjectionEvaluator(this)));
}
/////////////////////////////////////////////////////////////////////////////////
///                TX90 Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
TX90StateProjectionEvaluator::TX90StateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int TX90StateProjectionEvaluator::getDimension(void) const
{
    return 3;
}
void TX90StateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(3);
    cellSizes_[0] = 1.0;
    cellSizes_[1] = 1.0;
    cellSizes_[2] = 1.0;

}

void TX90StateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];

}


}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL



