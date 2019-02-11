
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
#include <kautham/planner/omplOpenDE/Setup/PlanarChainEnvironment.h>
#include <ode/ode.h>

namespace Kautham {
namespace omplcplanner{


//! Constructor create the Kuka robot enviroment and setup the parameters for ODE.
PlanarChainEnvironment::PlanarChainEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm,bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
//! Destructor
PlanarChainEnvironment::~PlanarChainEnvironment()
{

}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control
//! dimensions for Kuka robot.
unsigned int PlanarChainEnvironment::getControlDimension(void) const
{
    return 4;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void PlanarChainEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{
           lower[0] = -1;
           lower[1] = -1;
           upper[0] = 1;
           upper[1] = 1;

  }
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by
 * setting the velocities to the joints.
 */
void PlanarChainEnvironment::applyControl (const double *control) const
{

    dJointSetAMotorParam(motor_.at("Chainbase_link+Chainlink1"),dParamVel,control[0]);
    dJointSetAMotorParam(motor_.at("Chainbase_link+Chainlink1"),dParamFMax,7);
    dJointSetAMotorParam(motor_.at("Chainlink1+Chainlink2"),dParamVel,control[1]);
    dJointSetAMotorParam(motor_.at("Chainlink1+Chainlink2"),dParamFMax,7);

}
//! This function describe that how the robot will interact with the environment return true if the collision between
//! robot and the object is allowed, false otherwise
bool PlanarChainEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{

    std::string bodyType1 = geomNames_.at(geom1);
    std::string bodyType2 = geomNames_.at(geom2);

    if((bodyType1 == "robBody" && bodyType2 == "fixed")
            || (bodyType1 == "fixed" && bodyType2 == "robBody"))
    {
        return false;
    }
    else if((bodyType1 == "robBody" && bodyType2 == "freeManipulatable")
           || (bodyType1 == "freeManipulatable" && bodyType2 == "robBody"))
   {
       return false;
   }
    else if((bodyType1 == "fixed" && bodyType2 == "freeManipulatable")
           || (bodyType1 == "freeManipulatable" && bodyType2 == "fixed"))
   {
       return false;
   }
    else if((bodyType1 == "tcp" && bodyType2 == "freeManipulatable")
           || (bodyType1 == "freeManipulatable" && bodyType2 == "tcp"))
   {
       return true;
   }
    else if((bodyType1 == "tcp" && bodyType2 == "coManipulatable")
           || (bodyType1 == "coManipulatable" && bodyType2 == "tcp"))
   {
        const dReal *pos1 = dGeomGetPosition(geom1);
        const dReal *pos2 = dGeomGetPosition(geom2);
        if(bodyType1 == "tcp")
        {
            RigidBody rbody = Instknowledge->getManipulationConstraints(geom2);
            bool isColAllowed=rbody.isCollisionAllowed(pos1[0],pos1[1]);

            if(isColAllowed )
                return true;
            else
                return false;
        }
        else if(bodyType2 == "tcp")
        {
            RigidBody rbody = Instknowledge->getManipulationConstraints(geom1);
            bool isColAllowed=rbody.isCollisionAllowed(pos2[0],pos2[1]);
            if(isColAllowed )
                return true;
            else
                return false;
        }
   }
    else if((bodyType1 == "tcp" && bodyType2 == "fixed")
           || (bodyType1 == "fixed" && bodyType2 == "tcp"))
   {
       return false;
   }

    else if((bodyType1 == "fixed" && bodyType2 == "fixed")
           || (bodyType1 == "fixed" && bodyType2 == "fixed"))
   {
       return false;
   }
    else
        return true;

    return false;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void PlanarChainEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{

        contact.surface.mode = dContactSoftERP | dContactSoftCFM;


        std::string geom1_body = geomNames_.at(geom1);
        std::string geom2_body = geomNames_.at(geom2);
        //std::cout<<"Bodies are: "<<geom1_body<<" : "<<geom2_body<<std::endl;
        if ((geom1_body == "floor" && geom2_body == "odeGround") ||
                (geom1_body == "odeGround" && geom2_body == "floor" ))
            contact.surface.mu = dInfinity;
        else
            if ((geom1_body == "odeGround" && geom2_body == "robBody") ||
                    (geom1_body == "robBody" && geom2_body == "odeGround" ))
                contact.surface.mu = 0.1;
            else
                if ((geom1_body == "fixed" && geom2_body == "odeGround") ||
                        (geom1_body == "odeGround" && geom2_body == "fixed" ))
                    contact.surface.mu = 1;
                else
                    if ((geom1_body == "freeManipulatable" || geom2_body == "freeManipulatable"))
                    contact.surface.mu = 0.1;
        else
                        contact.surface.mu=0.1;
        contact.surface.soft_erp = _erp;
        contact.surface.soft_cfm = _cfm;

}

/////////////////////////////////////////////////////////////////////////////////
///                      Plannar State Space
/////////////////////////////////////////////////////////////////////////////////


/*! The PlanarChainStateSpace class is inherited from OpenDEStateSpace. It mainly reimplement the distance function and
 * register the projects for the state space.
 */
PlanarChainStateSpace::PlanarChainStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{


}
PlanarChainStateSpace::~PlanarChainStateSpace(){}
/*! Distance function describe that how the distance will be measured between two states in the PlanarChainStateSpace.
 * It measures simple cartesian distance of the first link of the gripper.
 */
double PlanarChainStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;
    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3); // body representing gripper first link
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    distance = sqrt(dx * dx + dy * dy);
    return distance;
}
/*! Register the projections by setting the pointer to the CarStateProjectionEvaluator
 */
void PlanarChainStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PlanarChainStateProjectionEvaluator(this)));
}

/////////////////////////////////////////////////////////////////////////////////
///                Planar Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////

/*! PlanarChainStateProjectionEvaluator is inherited from the ProjecttionEvaluator class of OMPL. It reimplement the virtual dunctions
 * such as getDimension, defaultCellSizes and project
 */
PlanarChainStateProjectionEvaluator::PlanarChainStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{}

/*! This function specify the dimensions of the projected space. In case of car, the state is project in 2D space. So
 * the dimensions of the projected space will be two.
 */
unsigned int PlanarChainStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}

/*! This function specify the cell size of the project space in meters.
 */
void PlanarChainStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.01;
    cellSizes_[1] = 0.01;
}

/*! This function describes how the state will be projected. In case of Planar chain, the position of the gripper
 *  is projected into 2D space x-axis and y-axis.
 */
void PlanarChainStateProjectionEvaluator::project(const ob::State *state, Kautham::VectorRef projection) const
{
    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    projection[0] = pos[0];
    projection[1] = pos[1];
}
///////////////////////////////////////////////////////////////////

/*! PlanarChainControlSampler class inherit from the RealVectorControlUniformSampler. It mainly reimplement the
 * sampleNext function that define how the control will be sampled.
 */
PlanarChainControlSampler::PlanarChainControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{}
/*! sampleNext function define the way the control will be sampled. It takes the previous control value and randomly
 * increase or decrease control value (velocity) by a factor DetlaT (DT).
 */
void PlanarChainControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{
    space_->copyControl(control, previous);
    const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();

    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
        static const double DT0 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        if (v > b.high[0] )
            v = b.high[0] - DT0;
        if (v < b.low[0]  )
            v = b.low[0] + DT0;
    }
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
        static const double DT1 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT1;
        if (v > b.high[1] )
            v = b.high[1] - DT1;
        if (v < b.low[1] )
            v = b.low[1] + DT1;
    }

}

void PlanarChainControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{

sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////
/*! PlanarChainControlSpace class inherit from the OpenDEControlSpace. It mainly reimplement the
 * allocControlSampler function by setting the pointer to the PlanarChainControlSampler.
 */
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



