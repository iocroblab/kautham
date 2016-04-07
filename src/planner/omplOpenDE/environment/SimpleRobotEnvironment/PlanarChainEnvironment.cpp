
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
#include "PlanarChainEnvironment.h"
#include <ode/ode.h>
namespace Kautham {
namespace omplcplanner{


//! Constructor create the Kuka robot enviroment and setup the parameters for ODE.
PlanarChainEnvironment::PlanarChainEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
{
    SetPlanningParameters();
    //configuration->resize(_Joint.size());
}
//! Destructor
PlanarChainEnvironment::~PlanarChainEnvironment()
{

}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control
//! dimensions for Kuka robot.
unsigned int PlanarChainEnvironment::getControlDimension(void) const
{
    return 2;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void PlanarChainEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{
//for(int i=0;i<2;i++)
//{
//          lower[0]=-10;
//          upper[0]=10;

//}
           lower[0]=-1;
            upper[0]=1;
            lower[1]=-1;
            upper[1]=1;
    //        lower[2]=-1;
    //        upper[2]=1;

}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by
 * setting the velocities to the joints.
 */
void PlanarChainEnvironment::applyControl (const double *control) const
{

//dBodyAddForce(stateBodies_[3],control[0],control[1],0.0);
     dReal r1=dJointGetHingeAngle(_Joint[0]);
     dReal r2=dJointGetHingeAngle(_Joint[1]);
     dReal r3=dJointGetHingeAngle(_Joint[2]);
    std::cout<<"angles are : "<<" , "<<r2<<std::endl;
    dJointSetHingeParam(_Joint[0], dParamVel, (control[0]-r1));
    dJointSetHingeParam(_Joint[0], dParamFMax, 600);
    dJointSetHingeParam(_Joint[1], dParamVel, (control[1]-r2));
    dJointSetHingeParam(_Joint[1], dParamFMax, 600);


    if(! manipulationQuery->getPlanningPhase() && manipulationQuery->getIsKinamaticChain())
    {

        for(int i=0;i<_Joint.size();i++)
        manipulationQuery->setconf( dJointGetHingeAngle(_Joint[i]));

        manipulationQuery->addJointConfiguration(manipulationQuery->getconf());
        manipulationQuery->clearconf();

    }


}
//! This function describe that how the robot will interact with the environment return true if the collision between
//! robot and the object is allowed, false otherwise
bool PlanarChainEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{

        return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void PlanarChainEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{

    contact.surface.mode = 0;
        contact.surface.mu = 0.5;

}

/////////////////////////////////////////////////////////////////////////////////
///                      Kuka State Space
/////////////////////////////////////////////////////////////////////////////////
PlanarChainStateSpace::PlanarChainStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{


}
PlanarChainStateSpace::~PlanarChainStateSpace(){}

double PlanarChainStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;


    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    double dz = fabs(p1[1] - p2[1]);

    distance = sqrt(dx * dx + dy * dy + dz * dz);

    return distance;
}

void PlanarChainStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PlanarChainStateProjectionEvaluator(this)));
}

void PlanarChainStateSpace::writeState(ob::State *state) const
{
    std::cout<<"Writing State Function================================="<<std::endl;
    const StateType *s = state->as<StateType>();
    for (int i = (int)env_->stateBodies_.size() - 1 ; i >= 0 ; --i)
    {
        unsigned int _i4 = i * 4;

        double *s_pos = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        dBodySetPosition(env_->stateBodies_[i], s_pos[0], s_pos[1], s_pos[2]);

        double *s_vel = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        dBodySetLinearVel(env_->stateBodies_[i], s_vel[0], s_vel[1], s_vel[2]);

        double *s_ang = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        dBodySetAngularVel(env_->stateBodies_[i],  s_ang[0], s_ang[1], s_ang[2]);

            const ob::SO3StateSpace::StateType &s_rot = *s->as<ob::SO3StateSpace::StateType>(_i4);
        dQuaternion q;
        q[0] = s_rot.w;
        q[1] = s_rot.x;
        q[2] = s_rot.y;
        q[3] = s_rot.z;
        dBodySetQuaternion(env_->stateBodies_[i], q);
    }

}

void PlanarChainStateSpace::readState(ob::State *state) const
{
    StateType *s = state->as<StateType>();
    for (int i = (int)env_->stateBodies_.size() - 1 ; i >= 0 ; --i)
    {
        unsigned int _i4 = i * 4;

        const dReal *pos = dBodyGetPosition(env_->stateBodies_[i]);
        const dReal *vel = dBodyGetLinearVel(env_->stateBodies_[i]);
        const dReal *ang = dBodyGetAngularVel(env_->stateBodies_[i]);
        double *s_pos = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        double *s_vel = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        double *s_ang = s->as<ob::RealVectorStateSpace::StateType>(_i4)->values; ++_i4;

        for (int j = 0; j < 3; ++j)
        {
            s_pos[j] = pos[j];
            s_vel[j] = vel[j];
            s_ang[j] = ang[j];
        }

        const dReal *rot = dBodyGetQuaternion(env_->stateBodies_[i]);
            ob::SO3StateSpace::StateType &s_rot = *s->as<ob::SO3StateSpace::StateType>(_i4);

        s_rot.w = rot[0];
        s_rot.x = rot[1];
        s_rot.y = rot[2];
        s_rot.z = rot[3];
    }

}
/////////////////////////////////////////////////////////////////////////////////
///                Kuka Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
PlanarChainStateProjectionEvaluator::PlanarChainStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int PlanarChainStateProjectionEvaluator::getDimension(void) const
{
    return 3;
}
void PlanarChainStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(3);
    cellSizes_[0] = 0.5;
    cellSizes_[1] = 0.5;
    cellSizes_[2] = 0.5;


}

void PlanarChainStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];



}
///////////////////////////////////////////////////////////////////
PlanarChainControlSampler::PlanarChainControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{

}
void PlanarChainControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{
    space_->copyControl(control, previous);
    const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
        static const double DT0 = 0.05;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        if (v > b.high[0])
            v = b.high[0] - DT0;
        if (v < b.low[0])
            v = b.low[0] + DT0;
    }
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
        static const double DT1 = 0.05;
        v += (rng_.uniformBool() ? 1 : -1) * DT1;
        if (v > b.high[1])
            v = b.high[1] - DT1;
        if (v < b.low[1])
            v = b.low[1] + DT1;
    }


}

void PlanarChainControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{

sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

PlanarChainControlSpace::PlanarChainControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
{
}
oc::ControlSamplerPtr PlanarChainControlSpace::allocControlSampler(void) const
{
return oc::ControlSamplerPtr(new PlanarChainControlSampler(this));
}

}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL



