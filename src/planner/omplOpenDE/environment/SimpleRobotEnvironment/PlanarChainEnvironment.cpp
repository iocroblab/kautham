
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

           lower[0]=-5;
            upper[0]=5;
            lower[1]=-5;
            upper[1]=5;
            lower[2]=-10;
             upper[2]=10;
             lower[3]=-10;
             upper[3]=10;

  }
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by
 * setting the velocities to the joints.
 */
void PlanarChainEnvironment::applyControl (const double *control) const
{
    dReal a1=dJointGetHingeAngle(joint_.at("Chainbase_link+Chainlink1"));
    dReal a2=dJointGetHingeAngle(joint_.at("Chainbase_link+Chainlink1"));

    dReal v1=dJointGetHingeAngleRate(joint_.at("Chainbase_link+Chainlink1"));
    dReal v2=dJointGetHingeAngleRate(joint_.at("Chainbase_link+Chainlink1"));

    // dReal v1=dJointGetAMotorParam(motor_.at("Chainbase_link+Chainlink1"),dParamVel);
    // dReal v2=dJointGetAMotorParam(motor_.at("Chainlink1+Chainlink2"),dParamVel);
    //std::cout<<"velocity   : "<<v1<<" , "<<v2<<std::endl;

    dJointSetAMotorParam(motor_.at("Chainbase_link+Chainlink1"),dParamVel,(a1-control[0]));
    dJointSetAMotorParam(motor_.at("Chainbase_link+Chainlink1"),dParamFMax,20);//mkinematics->getTorqueLimit().at(0));
    dJointSetAMotorParam(motor_.at("Chainlink1+Chainlink2"),dParamVel,(a2-control[1]));
    dJointSetAMotorParam(motor_.at("Chainlink1+Chainlink2"),dParamFMax,20);//mkinematics->getTorqueLimit().at(1));

//    dJointAddHingeTorque(joint_.at("Chainbase_link+Chainlink1"), control[0]);
//    dJointAddHingeTorque(joint_.at("Chainlink1+Chainlink2"), control[1]);

    //dJointSetHingeParam(joint_.at("Chainbase_link+Chainlink1"), dParamVel, (v1-control[0]));
    //dJointSetHingeParam(joint_.at("Chainbase_link+Chainlink1"), dParamFMax, mkinematics->getTorqueLimit().at(0));
    //dJointSetHingeParam(joint_.at("Chainlink1+Chainlink2"), dParamVel, (v2-control[1]));
    //dJointSetHingeParam(joint_.at("Chainlink1+Chainlink2"), dParamFMax, mkinematics->getTorqueLimit().at(1));
    //feedback1=dJointGetFeedback(joint_.at("Chainbase_link+Chainlink1"));
    //feedback2=dJointGetFeedback(joint_.at("Chainlink1+Chainlink2"));

    //std::cout<<"applied Torqe Limit is "<<mkinematics->getTorqueLimit().at(0)<<" "<<mkinematics->getTorqueLimit().at(1)<<std::endl;
    //std::cout<<"Current Trque is "<< feedback1->t2[2]<<" "<< feedback2->t2[2]<<std::endl;

    if(! manipulationQuery->getPlanningPhase() && manipulationQuery->getIskinematicsChain())
    {

//        manipulationQuery->setconf(dJointGetHingeAngle(joint_.at("Chainbase_link+Chainlink1")));
//        manipulationQuery->setconf(dJointGetHingeAngle(joint_.at("Chainlink1+Chainlink2")));
//        manipulationQuery->addJointConfiguration(manipulationQuery->getconf());
//        //std::cout<<"Joint Conf : "<<manipulationQuery->getconf().q.at(0)<<" , "<<manipulationQuery->getconf().q.at(1)<<std::endl;
//        manipulationQuery->clearconf();
//        manipulationQuery->setconf(feedback1->t2[2]);
//        manipulationQuery->setconf(feedback2->t2[2]);
//        manipulationQuery->AddjointTorque(manipulationQuery->getconf());
//        //std::cout<<"Torque     : "<<manipulationQuery->getconf().q.at(0)<<" , "<<manipulationQuery->getconf().q.at(1)<<std::endl;
//        manipulationQuery->clearconf();
//        manipulationQuery->setconf(control[0]);
//        manipulationQuery->setconf(control[1]);
//        manipulationQuery->setjointVelocity(manipulationQuery->getconf());
        //std::cout<<"velocity   : "<<manipulationQuery->getconf().q.at(0)<<" , "<<manipulationQuery->getconf().q.at(1)<<std::endl;
        manipulationQuery->clearconf();

    }


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
                contact.surface.mu = 0.02;
            else
                if ((geom1_body == "fixed" && geom2_body == "odeGround") ||
                        (geom1_body == "odeGround" && geom2_body == "fixed" ))
                    contact.surface.mu = 1;
                else
                    if ((geom1_body == "freeManipulatable" || geom2_body == "freeManipulatable"))
                    contact.surface.mu = 0.07;
        else
                        contact.surface.mu=0.07;
        contact.surface.soft_erp = _erp;
        contact.surface.soft_cfm = _cfm;

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
    const ob::SO3StateSpace::StateType &rot1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(3);
    const ob::SO3StateSpace::StateType &rot2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(3);

    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    //double dz = fabs(p1[1] - p2[1]);

//    double w = fabs(rot1.w*rot2.w);
//    double x = fabs(rot1.x*rot2.x);
//    double y = fabs(rot1.y*rot2.y);
//    double z = fabs(rot1.z*rot2.z);
    distance = sqrt(dx * dx + dy * dy);//+fabs(w+x+y+z);// + dz * dz);

    return distance;
}

void PlanarChainStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PlanarChainStateProjectionEvaluator(this)));
}

/////////////////////////////////////////////////////////////////////////////////
///                Kuka Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
PlanarChainStateProjectionEvaluator::PlanarChainStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int PlanarChainStateProjectionEvaluator::getDimension(void) const
{
    return 2;
}
void PlanarChainStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.2;
    cellSizes_[1] = 0.2;
    //cellSizes_[2] = 1;
}

void PlanarChainStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
    projection[0] = pos[0];
    projection[1] = pos[1];
   // projection[2] = pos[2];

}
///////////////////////////////////////////////////////////////////
PlanarChainControlSampler::PlanarChainControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
{

}

void PlanarChainControlSampler::sampleNext(oc::Control *control, const oc::Control *previous)
{
    space_->copyControl(control, previous);
    double mass=0;
    //todo:automatically detect the bodies related to end effector
    //last three bodies of the robot are belong to the gripper
    for(unsigned int i = 3; i< 6; i++)
    {
        const dReal *gripperbodypose = dBodyGetPosition(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->bodies[i]);
        mass= ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
               getEnvironment().get())->Instknowledge->isRobotInManipulationRegion(gripperbodypose[0],gripperbodypose[1]);
        if(mass!=-1)
            break;
    }
    std::vector<double> q;
    q.push_back(dJointGetHingeAngle(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                                     getEnvironment().get())->joint_.at("Chainbase_link+Chainlink1")));
    q.push_back(dJointGetHingeAngle(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                                     getEnvironment().get())->joint_.at("Chainlink1+Chainlink2")));
    const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
    if(mass==-1)
    {
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
        static const double DT0 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        if (v > b.high[0] )//|| q[0]>1.3)
            v = b.high[0] - DT0;
        if (v < b.low[0]  )//|| q[0]<-1.3)
            v = b.low[0] + DT0;
    }
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
        static const double DT1 = 0.5;
        v += (rng_.uniformBool() ? 1 : -1) * DT1;
        if (v > b.high[1] )//|| q[1]>1.3)
            v = b.high[1] - DT1;
        if (v < b.low[1] )//|| q[1]<-1.3)
            v = b.low[1] + DT1;
    }


        std::vector<double> torque = ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->mkinematics->getdefaultTorqueLimit();
        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                getEnvironment().get())->mkinematics->setTorqueLimit(torque);

        std::cout<<"CMove Torque "<<torque[0]<<" , "<<torque[1]<<std::endl;
    }
    else
    {
        std::vector<double> f;
        f.push_back(0.07*mass*9.8);
        f.push_back(0.07*mass*9.8);
        std::vector<double> tq=  ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
                                      getEnvironment().get())->mkinematics->getJointTorque(q,f);
        std::vector<double> torque = ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->mkinematics->getdefaultTorqueLimit();
        if (rng_.uniform01() > 0.3)
        {
            double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
            static const double DT0 = 0.5;
            v += (rng_.uniformBool() ? 1 : -1) * DT0;
            if (v > b.high[0]+tq[0])//|| q[0]>1.3)
                v = b.high[0] - DT0;
            if (v < b.low[0]-tq[0]  )//|| q[0]<-1.3)
                v = b.low[0] - DT0;
        }
        if (rng_.uniform01() > 0.3)
        {
            double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
            static const double DT1 = 0.5;
            v += (rng_.uniformBool() ? 1 : -1) * DT1;
            if (v > b.high[1]+tq[1] )//|| q[1]>1.3)
                v = b.high[1] - DT1;
            if (v < b.low[1]-tq[1] )//|| q[1]<-1.3)
                v = b.low[1] + DT1;
        }

//        torque[0]=torque[0]+fabs(tq[0]);
//        torque[1]=torque[1]+fabs(tq[1]);
//        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
//                getEnvironment().get())->mkinematics->setTorqueLimit(torque);
        //to update the manipulation regions
//        ((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->
//          Instknowledge->updateKnowledge(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->getEnvironment().get())->bodies);
        std::cout<<"Cinteraction Torque "<<torque[0]<<" , "<<torque[1]<<std::endl;;

    }
    double &v1 = control->as<oc::OpenDEControlSpace::ControlType>()->values[2];
    double &v2 = control->as<oc::OpenDEControlSpace::ControlType>()->values[3];
    v1=dBodyGetAngularVel(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->bodies[1])[2];
    v2=dBodyGetAngularVel(((KauthamDEEnvironment*)space_->as<oc::OpenDEControlSpace>()->
        getEnvironment().get())->bodies[2])[2];
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



