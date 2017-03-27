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

#include <kautham/planner/omplOpenDE/Setup/CarEnvironment.h>


using namespace std;
namespace Kautham {
namespace omplcplanner{
//! Constructor create the 2D robot enviroment and setup the parameters for ODE.
CarEnvironment::CarEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm, bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
//! Destructor
CarEnvironment::~CarEnvironment(){}

//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control dimensions.
unsigned int CarEnvironment::getControlDimension(void) const
{
    return 6;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void CarEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(6);
    upper.resize(6);
    lower[0] = -0.5;
    lower[1] = -5;

    upper[0] = 0.5;
    upper[1] = 10;
    for(int i=2;i<6;i++)
    {
        lower[i] = -5;
        upper[i] = 5;
    }

}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by simply
 * applying the force.
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
        dJointSetHinge2Param(cJoint[j],dParamFMax2,mkinematics->getTorqueLimit().at(j));
    }
    dJointGetFeedback(cJoint[0]);
    dJointGetFeedback(cJoint[1]);
    dJointGetFeedback(cJoint[2]);
    dJointGetFeedback(cJoint[3]);
    //std::cout<<"Torque : "<<feedback1->t2[1]<<" , "<<feedback2->t2[1]<<" , "<<feedback3->t2[1]<<" , "<<feedback4->t2[1]<<std::endl;
    //        const dReal *pos = dBodyGetPosition(bodies[0]);
    //        std::cout<<"position "<<pos[0]<<" , "<<pos[1]<<std::endl;
}
//! This function describe that how the robot will interact with the environment by enabling and disabling collision
bool CarEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& contact)const
{
    (void) contact; //unused

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
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
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
                contact.surface.mu = dInfinity;
            else
                if ((geom1_body == "freeManipulatable" || geom2_body == "freeManipulatable"))
                contact.surface.mu = 0.7;
                else
                    if ((geom1_body == "robBody" || geom2_body == "robBody"))
                    contact.surface.mu = 1;
    else
                    contact.surface.mu=0.7;
    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;


}

CarStateProjectionEvaluator::CarStateProjectionEvaluator(const ob::StateSpace *space, WorkSpace* _wkSpace) : ob::ProjectionEvaluator(space)
{
    bounds_.resize(2);
    bounds_.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds_.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds_.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds_.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
}
CarStateProjectionEvaluator::CarStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int CarStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}
void CarStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 1;
    cellSizes_[1] = 1;

}

void CarStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];

}

CarStateSpace::CarStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
CarStateSpace::~CarStateSpace()
{
}

double CarStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    //for (int i=0; i <= (((KauthamDEEnvironment*) env_.get())->getNumLinksFirstRobot()-1); i++)
    //for (int i=0; i <= (env_->getNumLinksFirstRobot()-1); i++)

    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);

    return sqrt(dx * dx + dy * dy );//+dz*dz);;

}
void CarStateSpace::registerProjections(void)
{

    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarStateProjectionEvaluator(this)));

}

///////////////////////////////////////////////////////////////////
CarControlSampler::CarControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{

}

void CarControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{

    space_->copyControl(control, previous);
    double mass=0;
    //todo:automatically detect the bodies related to end effector
    //last three bodies of the robot are belong to the gripper
    for(unsigned int i = 0; i< 5; i++)
    {
        const dReal *carpose = dBodyGetPosition(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->bodies[i]);
        mass= ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
               getEnvironment().get())->Instknowledge->isRobotInManipulationRegion(carpose[0],carpose[1]);
        if(mass!=-1)
            break;
    }

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

    if(mass==-1)
    {
        std::vector<double> torque = ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->mkinematics->getdefaultTorqueLimit();
        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                getEnvironment().get())->mkinematics->setTorqueLimit(torque);

        //std::cout<<"CMove Torque "<<torque[0]<<" , "<<torque[1]<<std::endl;
    }
    else
    {

        std::vector<double> torque = ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->mkinematics->getdefaultTorqueLimit();

//torque=3/2(mu*m*g)*r
        torque[0]=torque[0]+(1.5*0.7*mass*9.8*0.1);
        torque[1]=torque[1]+(1.5*0.7*mass*9.8*0.1);
        torque[2]=torque[2]+(1.5*0.7*mass*9.8*0.1);
        torque[3]=torque[3]+(1.5*0.7*mass*9.8*0.1);
        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                getEnvironment().get())->mkinematics->setTorqueLimit(torque);
        //to update the manipulation regions
//        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->
//          Instknowledge->updateKnowledge(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->bodies);
        //std::cout<<"Cinteraction Torque "<<torque[0]<<" , "<<torque[1]<<std::endl;;

    }
    double &v1 = control->as<oc::OpenDEControlSpace::ControlType>()->values[2];
    double &v2 = control->as<oc::OpenDEControlSpace::ControlType>()->values[3];
    double &v3 = control->as<oc::OpenDEControlSpace::ControlType>()->values[4];
    double &v4 = control->as<oc::OpenDEControlSpace::ControlType>()->values[5];
    v1=((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->feedback1->t2[1];
    v2=((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->feedback2->t2[1];
    v3=((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->feedback3->t2[1];
    v4=((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->feedback4->t2[1];




}

void CarControlSampler::sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/)
{
    sampleNext(control, previous);
}
////////////////////////////////////////////////////////////////////////////////////////////////

CarControlSpace::CarControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
{

}

oc::ControlSamplerPtr CarControlSpace::allocControlSampler(void) const
{
    return oc::ControlSamplerPtr(new CarControlSampler(this));
}

}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

