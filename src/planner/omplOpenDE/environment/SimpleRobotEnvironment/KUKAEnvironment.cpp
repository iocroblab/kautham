
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
KUKAEnvironment::KUKAEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm,bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
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
        lower[i]=-5;
        upper[i]=5;
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


const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();

for(int i=0;i<7;i++)
{
    if (rng_.uniform01() > 0.5)
    {

        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[i];
        static const double DT0 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        if (v > b.high[0])
            v = b.high[0] - DT0;
        if (v < b.low[0])
            v = b.low[0] - DT0;

    }

}


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



