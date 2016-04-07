
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
#include "KUKAEnvironment.h"
#include <ode/ode.h>
namespace Kautham {
namespace omplcplanner{


//! Constructor create the Kuka robot enviroment and setup the parameters for ODE.
KUKAEnvironment::KUKAEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
{
    SetPlanningParameters();
}
//! Destructor
KUKAEnvironment::~KUKAEnvironment()
{

}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control
//! dimensions for Kuka robot.
unsigned int KUKAEnvironment::getControlDimension(void) const
{
    return 7;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void KUKAEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    for(int i=0; i < 7; i++)
    {
        lower[i]=-2.0;
        upper[i]=3.14;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by
 * setting the velocities to the joints.
 */
void KUKAEnvironment::applyControl (const double *control) const
{

//    dJointSetAMotorParam(_motor[0], dParamVel, control[0]);
//    dJointSetAMotorParam(_motor[0], dParamFMax, dInfinity);
// std::cout<<"angle is:  [ ";
    for(unsigned int i=0;i<_motor.size();i++)
    {
    //dReal currentA = dJointGetHingeAngle(_Joint[i]);
     dReal currentA =   dJointGetHingeParam(_Joint[i],dParamVel);
//std::cout<<currentA<<"  ";
    dJointSetAMotorParam(_motor[i], dParamVel, (control[i])-currentA);
    dJointSetAMotorParam(_motor[i], dParamFMax, dInfinity);
      // dReal ang =dJointGetHingeAngle(_Joint[i]);
       // std::cout<<ang<<" , ";
}
   // std::cout<<" ]"<<std::endl;

    //std::cout<<std::endl;
//    dJointSetAMotorParam(_motor[0], dParamVel, control[0]);
//    dJointSetAMotorParam(_motor[0], dParamFMax, dInfinity);
//    dJointSetAMotorParam(_motor[1], dParamVel, control[1]);
//    dJointSetAMotorParam(_motor[1], dParamFMax, dInfinity);
//    dJointSetAMotorParam(_motor[2], dParamVel, control[2]);
//    dJointSetAMotorParam(_motor[2], dParamFMax, dInfinity);
}
//! This function describe that how the robot will interact with the environment return true if the collision between
//! robot and the object is allowed, false otherwise
bool KUKAEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{

        return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void KUKAEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{

    contact.surface.mode = 0;
        contact.surface.mu = 0.5;

}

/////////////////////////////////////////////////////////////////////////////////
///                      Kuka State Space
/////////////////////////////////////////////////////////////////////////////////
KUKAStateSpace::KUKAStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{

}
KUKAStateSpace::~KUKAStateSpace(){}

double KUKAStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;


    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(7);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(7);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    double dz = fabs(p1[1] - p2[1]);

    distance = sqrt(dx * dx + dy * dy  + dz *dz);

    return distance;
}

void KUKAStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new KUKAStateProjectionEvaluator(this)));
}
/////////////////////////////////////////////////////////////////////////////////
///                Kuka Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
KUKAStateProjectionEvaluator::KUKAStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int KUKAStateProjectionEvaluator::getDimension(void) const
{
    return 3;
}
void KUKAStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(3);
    cellSizes_[0] = 1.0;
    cellSizes_[1] = 1.0;
    cellSizes_[2] = 1.0;


}

void KUKAStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(7);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];



}
///////////////////////////////////////////////////////////////////
KukaControlSampler::KukaControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{

}
void KukaControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{
    space_->copyControl(control, previous);
    ob::RealVectorBounds b(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->getControlDimension());
    std::vector<dJointID> J;
    for(int i=0;i<7;i++)
    {
        J.push_back(dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[i+1],1));
//       dJointType type=dJointGetType(J[i]);

        dReal high = dJointGetHingeParam(J[i],dParamHiStop);
        dReal low = dJointGetHingeParam(J[i],dParamLoStop);
        b.high[i] = high;
        b.low[i] =  low;
    }

//    dJointID J1 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J2 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J3 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J4 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J5 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J6 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);
//    dJointID J7 = dBodyGetJoint(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0],0);


//    b.high[0] = dJointGetHingeParam(J1,dParamHiStop);
//    b.high[1] = dJointGetHingeParam(J2,dParamHiStop);
//    b.high[2] = dJointGetHingeParam(J3,dParamHiStop);
//    b.high[3] = dJointGetHingeParam(J4,dParamHiStop);
//    b.high[4] = dJointGetHingeParam(J5,dParamHiStop);
//    b.high[5] = dJointGetHingeParam(J6,dParamHiStop);
//    b.high[6] = dJointGetHingeParam(J7,dParamHiStop);

//    b.low[0] =  dJointGetHingeParam(J1,dParamLoStop);
//    b.low[1] =  dJointGetHingeParam(J2,dParamLoStop);
//    b.low[2] =  dJointGetHingeParam(J3,dParamLoStop);
//    b.low[3] =  dJointGetHingeParam(J4,dParamLoStop);
//    b.low[4] =  dJointGetHingeParam(J5,dParamLoStop);
//    b.low[5] =  dJointGetHingeParam(J6,dParamLoStop);
//    b.low[6] =  dJointGetHingeParam(J7,dParamLoStop);

//const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
//if (rng_.uniform01() > 0.5)

    for(int i=0;i<7;i++)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[i];
        static const double DT0 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        dReal angle = dJointGetHingeAngle(J[i]);
        angle = angle + v*0.05;
        if (angle > b.high[i])
            angle = b.high[i] - DT0;
        if (angle < b.low[i])
            angle= b.low[i] + DT0;


//        if (v > b.high[i])
//            v = b.high[i] - DT0;
//        if (v < b.low[i])
//            v = b.low[i] + DT0;
    }

//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}
//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[2];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}
//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[3];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}
//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[4];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}
//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[5];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}
//if (rng_.uniform01() > 0.5)
//{
//    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[6];
//    static const double DT1 = 0.5;
//    v += (rng_.uniformBool() ? 1 : -1) * DT1;
//    if (v > b.high[1])
//    v = b.high[1] - DT1;
//    if (v < b.low[1])
//    v = b.low[1] + DT1;
//}

}

void KukaControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{

sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

KukaControlSpace::KukaControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
{
}
oc::ControlSamplerPtr KukaControlSpace::allocControlSampler(void) const
{
return oc::ControlSamplerPtr(new KukaControlSampler(this));
}

}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL



