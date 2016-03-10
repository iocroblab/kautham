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

#include <problem/workspace.h>
#include <sampling/sampling.h>
#include <boost/bind/mem_fn.hpp>
#include "ConstraintAware2DRobotEnvironment.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include "../KauthamOpenDEEnvironment.h"
using namespace std;
namespace Kautham {
namespace omplcplanner{
//! Constructor create the ODE table enviroment and setup the parameters for ODE.
ConstraintAware2DRobotEnvironment::ConstraintAware2DRobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
{
     //robBase = (ws->getRobot(0)->getName())+(ws->getRobot(0)->getLink(0)->getName());
    // floor = (ws->getObstacle(ws->getNumObstacles()-1)->getName())+(ws->getObstacle(ws->getNumObstacles()-1)->getLink(0)->getName());
    //potser he de fer lo de setKinematic aquí o en el applyControl;
    SetPlanningParameters();
}
ConstraintAware2DRobotEnvironment::~ConstraintAware2DRobotEnvironment()
{

}

//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
unsigned int ConstraintAware2DRobotEnvironment::getControlDimension(void) const
{
    return 2;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the bounding box to performe sampling control.
 */
void ConstraintAware2DRobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(2);
    upper.resize(2);
    for(int i=0; i < 2; i++)
    {   lower[i] = -10;
        upper[i] = 10;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply.This function apply the control by
 * setting the forces, velocities and torques.
 */
void ConstraintAware2DRobotEnvironment::applyControl (const double *control) const
{
    if(manipulationQuery->getActionType()=="move")
    {
        dBodyAddForce(bodies[0],control[0],control[1],0.0);
    }
    else
        if(manipulationQuery->getActionType()=="pull" || manipulationQuery->getActionType()=="Pull"
                                                      || manipulationQuery->getActionType()=="push"
                                                      || manipulationQuery->getActionType()=="Push")
        {
            dBodyAddForce(bodies[0],manipulationQuery->getforce().at(0),manipulationQuery->getforce().at(1),manipulationQuery->getforce().at(2));
        }
        else
            std::cout<<"Invalid Action"<<std::endl;
}

bool ConstraintAware2DRobotEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/)const
{

    std::string bodyType1 = geomNames_.at(geom1);
    std::string bodyType2 = geomNames_.at(geom2);
    //std::cout<<"Bodies are: "<<bodyType1<<" : "<<bodyType2<<std::endl;

    if((bodyType1 == "robBody" && bodyType2 == "floor")
            || (bodyType1 == "floor" && bodyType2 == "robBody"))
    {
        //std::cout<<"evaluationg collision1"<<std::endl;
        return true;
    }
    else
        if((bodyType1 == "robBody" && bodyType2 == "coManipulatable")
                || (bodyType1 == "coManipulatable" && bodyType2 == "robBody"))

        {
            //std::cout<<"evaluationg collision2"<<std::endl;

            const dReal *pos1 = dGeomGetPosition(geom1);
            const dReal *pos2 = dGeomGetPosition(geom2);
            if(bodyType1 == "robBody")
            {
                RigidBody rbody = Instknowledge->getManipulationConstraints(geom2);
                bool isColAllowed=rbody.isCollisionAllowed(pos1[0],pos1[1]);

                if(isColAllowed )
                    return true;
                else
                    return false;
            }
            else if(bodyType2 == "robBody")
            {
                RigidBody rbody = Instknowledge->getManipulationConstraints(geom1);
                bool isColAllowed=rbody.isCollisionAllowed(pos2[0],pos2[1]);
                if(isColAllowed )
                    return true;
                else
                    return false;
            }
        }
        else
            if((bodyType1 == "robBody" && bodyType2 == "fixed")
                    || (bodyType1 == "fixed" && bodyType2 == "robBody"))
            {
                return false;
            }
            else
               return true;

}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment
 * This method set the parameters for the contact, like what will be the value
 * of friction coefficient, etc.
 */
void ConstraintAware2DRobotEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    contact.surface.mode = dContactSoftERP | dContactSoftCFM;


    std::string geom1_body = geomNames_.at(geom1);
    std::string geom2_body = geomNames_.at(geom2);
    //std::cout<<"Bodies are: "<<geom1_body<<" : "<<geom2_body<<std::endl;
    if ((geom1_body == "floor" && geom2_body == "odeGround") ||
            (geom1_body == "odeGround" && geom2_body == "floor" ))
        contact.surface.mu = dInfinity;
    else
        if ((geom1_body == "floor" && geom2_body == "robBody") ||
                (geom1_body == "robBody" && geom2_body == "floor" ))
            contact.surface.mu = 0.0;
        else
            if ((geom1_body == "floor" && geom2_body == "fixed") ||
                    (geom1_body == "fixed" && geom2_body == "floor" ))
                contact.surface.mu = dInfinity;
            else
                contact.surface.mu = 0.7;
    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;

}

ConstraintAwaretwoDRobotStateProjectionEvaluator::ConstraintAwaretwoDRobotStateProjectionEvaluator(const ob::StateSpace *space, WorkSpace* _wkSpace) : ob::ProjectionEvaluator(space)
{
    bounds_.resize(2);
    bounds_.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds_.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds_.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds_.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
}
ConstraintAwaretwoDRobotStateProjectionEvaluator::ConstraintAwaretwoDRobotStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int ConstraintAwaretwoDRobotStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}
void ConstraintAwaretwoDRobotStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 1.0;
    cellSizes_[1] = 1.0;

}

void ConstraintAwaretwoDRobotStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];

}

ConstraintAwaretwoDRobotStateSpace::ConstraintAwaretwoDRobotStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
ConstraintAwaretwoDRobotStateSpace::~ConstraintAwaretwoDRobotStateSpace()
{
}

double ConstraintAwaretwoDRobotStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    //double distance = 0.0;
    //for (int i=0; i <= (((KauthamDEEnvironment*) env_.get())->getNumLinksFirstRobot()-1); i++)
    //for (int i=0; i <= (env_->getNumLinksFirstRobot()-1); i++)

        const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(p1[0] - p2[0]);
        double dy = fabs(p1[1] - p2[1]);
        //double dz = fabs(p1[1] - p2[1]);

    return sqrt(dx * dx + dy * dy );//+dz*dz);;

}
void ConstraintAwaretwoDRobotStateSpace::registerProjections(void)
{

    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new ConstraintAwaretwoDRobotStateProjectionEvaluator(this)));

}

///////////////////////////////////////////////////////////////////
ConstraintAwaretwoDControlSampler::ConstraintAwaretwoDControlSampler(const oc::ControlSpace *cm, oc::OpenDEEnvironmentPtr env) : oc::RealVectorControlUniformSampler(cm)
{
    //for (int i=0; i <= (((KauthamDEEnvironment*) env_.get())->getNumLinksFirstRobot()-1); i++)


}
void ConstraintAwaretwoDControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{
//const dReal *pos=dBodyGetPosition(space_->as<oc::OpenDEControlSpace>()->getEnvironment()->stateBodies_[0]);
//std::cout<<"position of rob is : [ "<< pos[0]<<" , "<<pos[1]<<" ]"<<std::endl;
space_->copyControl(control, previous);
//if(pos[0]<0)
{
    std::cout<<"Robot is in Cmove"<<std::endl;
    std::cout<<"control value = [ "<< control->as<oc::OpenDEControlSpace::ControlType>()->values[0]<<" , "<<control->as<oc::OpenDEControlSpace::ControlType>()->values[1]<<" ] "<<std::endl;
const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
if (rng_.uniform01() > 0.3)
{
    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
    static const double DT0 = 0.5;
    v += (rng_.uniformBool() ? 1 : -1) * DT0;
    if (v > b.high[0])
    v = b.high[0] - DT0;
    if (v < b.low[0])
    v = b.low[0] + DT0;
}
if (rng_.uniform01() > 0.3)
{
    double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
    static const double DT1 = 0.5;
    v += (rng_.uniformBool() ? 1 : -1) * DT1;
    if (v > b.high[1])
    v = b.high[1] - DT1;
    if (v < b.low[1])
    v = b.low[1] + DT1;
}
}
//else
//{
//    std::cout<<"Robot is in Cint"<<std::endl;

////    ob::RealVectorBounds rb(2);
////    rb.low[0] = -20;
////    rb.low[1] = -20;
////    rb.high[0] = 20;
////    rb.high[1] = 20;
////    const ob::RealVectorBounds &rbb=rb;
////    space_->as<oc::OpenDEControlSpace>()->setBounds(rbb);
////    const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();

//        ob::RealVectorBounds rb(2);
//        rb.low[0] = -20;
//        rb.low[1] = -20;
//        rb.high[0] = 20;
//        rb.high[1] = 20;
//        std::cout<<"control value = [ "<< control->as<oc::OpenDEControlSpace::ControlType>()->values[0]<<" , "<<control->as<oc::OpenDEControlSpace::ControlType>()->values[1]<<" ] "<<std::endl;

//    if (rng_.uniform01() > 0.3)
//    {
//        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
//        static const double DT0 = 0.5;
//        v += (rng_.uniformBool() ? 1 : -1) * DT0;
//        if (v > rb.high[0])
//        v = rb.high[0] - DT0;
//        if (v < rb.low[0])
//        v = rb.low[0] + DT0;
//    }
//    if (rng_.uniform01() > 0.3)
//    {
//        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
//        static const double DT1 = 0.5;
//        v += (rng_.uniformBool() ? 1 : -1) * DT1;
//        if (v > rb.high[1])
//        v = rb.high[1] - DT1;
//        if (v < rb.low[1])
//        v = rb.low[1] + DT1;
//    }
//}
}

void ConstraintAwaretwoDControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{
sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

ConstraintAwaretwoDControlSpace::ConstraintAwaretwoDControlSpace(const ob::StateSpacePtr &m, oc::OpenDEEnvironmentPtr env) : oc::OpenDEControlSpace(m)
{
    Envp= env;
}

oc::ControlSamplerPtr ConstraintAwaretwoDControlSpace::allocControlSampler(void) const
{
return oc::ControlSamplerPtr(new ConstraintAwaretwoDControlSampler(this,Envp));
}

}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

