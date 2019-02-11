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
#include <kautham/planner/omplOpenDE/Setup/CarEnvironment.h>


using namespace std;
namespace Kautham {
namespace omplcplanner{
//! Constructor create the Car enviroment and setup the parameters for ODE.
CarEnvironment::CarEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm, bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
//! Destructor
CarEnvironment::~CarEnvironment(){}

//! This is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control dimensions.
//! In case of car we have two control dimensions steering and joint angular velocity.
unsigned int CarEnvironment::getControlDimension(void) const
{
    return 2;
}
/*! This is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 * We define the control bound for steering [-0.5,0.5] and for velocity [-5, 5]m/s.
 */
void CarEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(2);
    upper.resize(2);
    lower[0] = -0.5;
    lower[1] = -5;
    upper[0] = 0.5;
    upper[1] = 10;
}
/*! This is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by setting the velocities.
 */
void CarEnvironment::applyControl (const double *control) const
{

    dReal turn = control[0];
    dReal speed = control[1];
    for (int j = 0; j < 4; j++)
    {
       dReal curturn = dJointGetHinge2Angle1 (cJoint[j]);
        //dJointAddHinge2Torques(cJoint[j],0,speed);
        dJointSetHinge2Param(cJoint[j],dParamVel,(turn-curturn)*1.0);
        dJointSetHinge2Param(cJoint[j],dParamFMax,2);

        dJointSetHinge2Param(cJoint[j],dParamVel2,speed);
        dJointSetHinge2Param(cJoint[j],dParamFMax2,50);
    }
}
//! This function describe that how the robot will interact with the environment by enabling and disabling collision
bool CarEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& contact)const
{
    (void)contact;
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
       return true;
   }
    else

        return true;

}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (the contact dynamics), like what will be the value of friction coefficient, and ode specific parameters such as constraint force mixing
 * and error reduction parameters etc.
 */
void CarEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
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
            contact.surface.mu = 0.4;
        else
            if ((geom1_body == "robBody" && geom2_body == "freeManipulatable") ||
                    (geom1_body == "freeManipulatable" && geom2_body == "robBody" ))
                contact.surface.mu = 0.0001;
            else
                if ((geom1_body == "robBody" && geom2_body == "odeGround") ||
                        (geom1_body == "odeGround" && geom2_body == "robBody" ))
                    contact.surface.mu = 5;
        else
            if ((geom1_body == "fixed" || geom2_body == "fixed") )
                contact.surface.mu = 0.1;
            else
                if ((geom1_body == "freeManipulatable" || geom2_body == "freeManipulatable"))
                contact.surface.mu = 0.1;
                else
                    if ((geom1_body == "robBody" || geom2_body == "robBody"))
                    contact.surface.mu = 1;
    else
                    contact.surface.mu=0.1;
    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;
}
/*! CarStateProjectionEvaluator is inherited from the ProjecttionEvaluator class of OMPL. It reimplement the virtual dunctions
 * such as getDimension, defaultCellSizes and project
 */
CarStateProjectionEvaluator::CarStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{}

/*! This function specify the dimensions of the projected space. In case of car, the state is project in 2D space. So
 * the dimensions of the projected space will be two.
 */
unsigned int CarStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}

/*! This function specify the cell size of the project space in meters.
 */
void CarStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;

}

/*! This function describes how the state will be projected. In case of car, the position of the car chassi
 *  is projected into 2D space x-axis and y-axis.
 */
void CarStateProjectionEvaluator::project(const ob::State *state, Kautham::VectorRef projection) const
{

    const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0); // representing car chassi
    projection[0] = pos[0];
    projection[1] = pos[1];

}

/*! The CarStateSpace class is inherited from OpenDEStateSpace. It mainly reimplement the distance function and
 * register the projects for the state space.
 */
CarStateSpace::CarStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
CarStateSpace::~CarStateSpace()
{
}

/*! Distance function describe that how the distance will be measured between two states of the CarStateSpace.
 * It measures simple cartesian distance in 2D space.
 */
double CarStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{

    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);// representing car chassi
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);// representing car chassi
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    return sqrt(dx * dx + dy * dy );

}

/*! Register the projections by setting the pointer to the CarStateProjectionEvaluator
 */
void CarStateSpace::registerProjections(void)
{

    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarStateProjectionEvaluator(this)));

}

/*! CarControlSampler class inherit from the RealVectorControlUniformSampler. It mainly reimplement the
 * sampleNext function that define how the control will be sampled.
 */
CarControlSampler::CarControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{

}
/*! sampleNext function define the way the control will be sampled. It takes the previous control value and randomly
 * increase or decrease control value (velocity) by a factor DetlaT (DT).
 */
void CarControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{

    space_->copyControl(control, previous);
    

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

void CarControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{
    sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////
/*! CarControlSpace class inherit from the OpenDEControlSpace. It mainly reimplement the
 * allocControlSampler function by setting the pointer to the CarControlSampler.
 */
CarControlSpace::CarControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
{

}
/*! allocControlSampler basically set the pointer to the CarControlSampler.
 */
oc::ControlSamplerPtr CarControlSpace::allocControlSampler(void) const
{
    return oc::ControlSamplerPtr(new CarControlSampler(this));
}

}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

