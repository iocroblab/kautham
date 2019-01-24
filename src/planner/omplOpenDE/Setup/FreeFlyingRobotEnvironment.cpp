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

/* Author: Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <boost/bind/mem_fn.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>

#include <kautham/planner/omplOpenDE/Setup/FreeFlyingRobotEnvironment.h>

using namespace std;
namespace Kautham {
namespace omplcplanner{
//! Constructor create the free-flying enviroment and setup the parameters for ODE.
FreeFlyingRobotEnvironment::FreeFlyingRobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm,bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
//! Destructor
FreeFlyingRobotEnvironment::~FreeFlyingRobotEnvironment(){}

//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control
//! dimensions for free flying robot.
unsigned int FreeFlyingRobotEnvironment::getControlDimension(void) const
{
    return 3;
}

/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void FreeFlyingRobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(3);
    upper.resize(3);
    for(int i=0; i < 3; i++)
    {   lower[i] = -10;
        upper[i] = 10;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply for the free flying robot. Here the
 * controls are applying by simply applying the force.
 */
void FreeFlyingRobotEnvironment::applyControl (const double *control) const
{
    dBodyAddForce(bodies[0],control[0],control[1],control[2]);
}
//! This function describe that how the robot will interact with the environment return true if the collision between
//! robot and the object is allowed, false otherwise
bool FreeFlyingRobotEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/)const
{
    (void) geom1; //unused
    (void) geom2; //unused
    return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void FreeFlyingRobotEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    contact.surface.mode = dContactSoftERP | dContactSoftCFM;


    std::string geom1_body = geomNames_.at(geom1);
    std::string geom2_body = geomNames_.at(geom2);
    if ((geom1_body == "DEPlaneBase" && geom2_body == "odeGround") ||
            (geom1_body == "odeGround" && geom2_body == "DEPlaneBase" ))
        contact.surface.mu = dInfinity;

    else
        if ((geom1_body == "OpenDEroomsBase" && geom2_body == "DEPlaneBase") ||
                (geom1_body == "DEPlaneBase" && geom2_body == "OpenDEroomsBase" ))

            contact.surface.mu=dInfinity;
        else
            if ((geom1_body == "odeGround" || geom2_body == "odeGround") ||
                    (geom1_body == "DEPlaneBase" || geom2_body == "DEPlaneBase" ))
                contact.surface.mu=dInfinity;
            else
                if ((geom1_body == "SphereDEbase" && geom2_body == "OpenDEroomsBase") ||
                        (geom1_body == "OpenDEroomsBase" && geom2_body == "SphereDEbase" ))

                    contact.surface.mu = 0.0;
                else
                    contact.surface.mu=2;

    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;

}
freeFlyingRobotStateProjectionEvaluator::freeFlyingRobotStateProjectionEvaluator(const ob::StateSpace *space, WorkSpace* _wkSpace) : ob::ProjectionEvaluator(space)
{
    bounds_.resize(2);
    bounds_.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds_.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds_.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds_.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
}
freeFlyingRobotStateProjectionEvaluator::freeFlyingRobotStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int freeFlyingRobotStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}
void freeFlyingRobotStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;
    //cellSizes_[2] = 1.0;

}

void freeFlyingRobotStateProjectionEvaluator::project(const ob::State *state, Kautham::VectorRef projection) const
{

    const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
    //projection[2] = pos[2];

}

freeFlyingRobotStateSpace::freeFlyingRobotStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
freeFlyingRobotStateSpace::~freeFlyingRobotStateSpace()
{
}

double freeFlyingRobotStateSpace::distance(const ob::State *s1, const ob::State *s2) const
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
void freeFlyingRobotStateSpace::registerProjections(void)
{

    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new freeFlyingRobotStateProjectionEvaluator(this)));

}

///////////////////////////////////////////////////////////////////
freeFlyingRobotControlSampler::freeFlyingRobotControlSampler(const oc::ControlSpace *cm, oc::OpenDEEnvironmentPtr env) : oc::RealVectorControlUniformSampler(cm)
{
    (void) env;//unused
}
void freeFlyingRobotControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
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

void freeFlyingRobotControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{
sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

freeFlyingRobotControlSpace::freeFlyingRobotControlSpace(const ob::StateSpacePtr &m, oc::OpenDEEnvironmentPtr env) : oc::OpenDEControlSpace(m)
{
    Envp= env;
}

oc::ControlSamplerPtr freeFlyingRobotControlSpace::allocControlSampler(void) const
{
return oc::ControlSamplerPtr(new freeFlyingRobotControlSampler(this,Envp));
}

}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

