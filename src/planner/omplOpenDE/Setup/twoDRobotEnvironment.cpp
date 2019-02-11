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
#include <kautham/planner/omplOpenDE/Setup/twoDRobotEnvironment.h>

using namespace std;
namespace Kautham {
namespace omplcplanner{
//! Constructor create the 2D robot enviroment and setup the parameters for ODE.
twoDRobotEnvironment::twoDRobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm,bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
//! Destructor
twoDRobotEnvironment::~twoDRobotEnvironment(){}

//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control dimensions.
unsigned int twoDRobotEnvironment::getControlDimension(void) const
{
    return 2;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void twoDRobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(2);
    upper.resize(2);
    for(int i=0; i < 2; i++)
    {  lower[i] = -20;
        upper[i] = 20;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply depending on the action type such as pull, push or move.
 *  Here the controls are applying by simply applying the force.
 */
void twoDRobotEnvironment::applyControl (const double *control) const
{

    if(manipulationQuery->getActionType()=="move" || manipulationQuery->getActionType()=="Move")
    {
        dBodyAddForce(bodies[0],control[0],control[1],0.0);
    }
    else
        if(manipulationQuery->getActionType()=="pull" || manipulationQuery->getActionType()=="Pull"
                || manipulationQuery->getActionType()=="push"
                || manipulationQuery->getActionType()=="Push") {
            dBodyAddForce(bodies[0],manipulationQuery->getforce().at(0),manipulationQuery->getforce().at(1),manipulationQuery->getforce().at(2));
        } else {
            std::cout<<"Invalid Action"<<std::endl;}
}
//! This function describe that how the robot will interact with the environment by enabling and disabling collision
bool twoDRobotEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/)const
{

    std::string bodyType1 = geomNames_.at(geom1);
    std::string bodyType2 = geomNames_.at(geom2);

    if((bodyType1 == "robBody" && bodyType2 == "fixed")
            || (bodyType1 == "fixed" && bodyType2 == "robBody"))
    {
        return false;
    }
    else
        if((bodyType1 == "robBody" && bodyType2 == "floor")
                || (bodyType1 == "floor" && bodyType2 == "robBody"))
        {
            return true;
        }
        else
            if((bodyType1 == "robBody" && bodyType2 == "freeManipulatable")
                    || (bodyType1 == "freeManipulatable" && bodyType2 == "robBody"))
            {
                return true;
            }
            else
                return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void twoDRobotEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    contact.surface.mode = dContactSoftERP | dContactSoftCFM;


    std::string geom1_body = geomNames_.at(geom1);
    std::string geom2_body = geomNames_.at(geom2);
    if ((geom1_body == "floor" && geom2_body == "odeGround") ||
            (geom1_body == "odeGround" && geom2_body == "floor" ))
        contact.surface.mu = dInfinity;
    else
        if ((geom1_body == "floor" && geom2_body == "robBody") ||
                (geom1_body == "robBody" && geom2_body == "floor" ))
            contact.surface.mu = 0.5;
        else
            if ((geom1_body == "floor" && geom2_body == "fixed") ||
                    (geom1_body == "fixed" && geom2_body == "floor" ))
                contact.surface.mu = 0.7;
            else
                contact.surface.mu = 0.01;
    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;
}

/*! PlanarChainStateProjectionEvaluator is inherited from the ProjecttionEvaluator class of OMPL. It reimplement the virtual dunctions
 * such as getDimension, defaultCellSizes and project
 */
twoDRobotStateProjectionEvaluator::twoDRobotStateProjectionEvaluator(const ob::StateSpace *space, WorkSpace* _wkSpace) : ob::ProjectionEvaluator(space)
{
    bounds_.resize(2);
    bounds_.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds_.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds_.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds_.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
}
twoDRobotStateProjectionEvaluator::twoDRobotStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{}
/*! This function specify the dimensions of the projected space. In case of sphere, the state is project in 2D space. So
 * the dimensions of the projected space will be two.
 */
unsigned int twoDRobotStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}
/*! This function specify the cell size of the project space in meters.
 */
void twoDRobotStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.01;
    cellSizes_[1] = 0.01;
}
/*! This function describes how the state will be projected. In case of sphere, the position
 *  is projected into 2D space x-axis and y-axis.
 */
void twoDRobotStateProjectionEvaluator::project(const ob::State *state, Kautham::VectorRef projection) const
{

    const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
}
/*! The twoDRobotStateSpace class is inherited from OpenDEStateSpace. It mainly reimplement the distance function and
 * register the projects for the state space.
 */
twoDRobotStateSpace::twoDRobotStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
twoDRobotStateSpace::~twoDRobotStateSpace()
{
}
/*! Distance function describe that how the distance will be measured between two states in the twoDRobotStateSpace.
 * It measures simple cartesian distance between two states.
 */
double twoDRobotStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);

    return sqrt(dx * dx + dy * dy );
}
/*! Register the projections by setting the pointer to the twoDRobotStateProjectionEvaluator
 */
void twoDRobotStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new twoDRobotStateProjectionEvaluator(this)));
}

///////////////////////////////////////////////////////////////////

/*! twoDControlSampler class inherit from the RealVectorControlUniformSampler. It mainly reimplement the
 * sampleNext function that define how the control will be sampled.
 */
twoDControlSampler::twoDControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{ }
/*! sampleNext function define the way the control will be sampled. It takes the previous control value and randomly
 * increase or decrease control value (velocity) by a factor DetlaT (DT).
 */
void twoDControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
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

void twoDControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{
    sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

twoDControlSpace::twoDControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
{
}
/*! twoDControlSpace class inherit from the OpenDEControlSpace. It mainly reimplement the
 * allocControlSampler function by setting the pointer to the twoDControlSampler.
 */
oc::ControlSamplerPtr twoDControlSpace::allocControlSampler(void) const
{
    return oc::ControlSamplerPtr(new twoDControlSampler(this));
}
}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

