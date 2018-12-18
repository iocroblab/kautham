
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
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
#include <ode/ode.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <kautham/planner/omplOpenDE/Setup/TiagoEnvironment.h>

namespace Kautham {
namespace omplcplanner{


//! Constructor create the Kuka robot enviroment and setup the parameters for ODE.
TiagoEnvironment::TiagoEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm, bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    threshold =0.5;
    wkspace=ws;
    SetPlanningParameters();
}
//! Destructor
TiagoEnvironment::~TiagoEnvironment()
{

}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the control
//! dimensions for Kuka robot.
unsigned int TiagoEnvironment::getControlDimension(void) const
{
    //get control dimension
    return 6;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the the max and min limits for sampling the control.
 */
void TiagoEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{
    //xml value
    //lower[0]=-0.1;
    //upper[0]= 0.1;
    for(int i=0; i < 6; i++)
    {
        lower[i]=-0.1;
        upper[i]= 0.1;
    }

}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply. Here the controls are applying by
 * setting the velocities to the joints.
 */
void TiagoEnvironment::applyControl(const double *control) const
{

//    dJointSetLMotorParam(motor_.at("tiagotorso_fixed_link+tiagotorso_lift_link"), dParamVel, control[0]);
//    dJointSetLMotorParam(motor_.at("tiagotorso_fixed_link+tiagotorso_lift_link"), dParamFMax, 2000);

//    dJointSetAMotorParam(motor_.at("tiagotorso_lift_link+tiagohead_1_link"), dParamVel, control[1]);
//    dJointSetAMotorParam(motor_.at("tiagotorso_lift_link+tiagohead_1_link"), dParamFMax, 150);

//    dJointSetAMotorParam(motor_.at("tiagohead_1_link+tiagohead_2_link"), dParamVel, control[2]);
//    dJointSetAMotorParam(motor_.at("tiagohead_1_link+tiagohead_2_link"), dParamFMax, 150);

    dJointSetAMotorParam(motor_.at("tiagotorso_lift_link+tiagoarm_1_link"), dParamVel, control[0]);
    dJointSetAMotorParam(motor_.at("tiagotorso_lift_link+tiagoarm_1_link"), dParamFMax, 50);

    dJointSetAMotorParam(motor_.at("tiagoarm_1_link+tiagoarm_2_link"), dParamVel, control[1]);
    dJointSetAMotorParam(motor_.at("tiagoarm_1_link+tiagoarm_2_link"), dParamFMax,50);

    dJointSetAMotorParam(motor_.at("tiagoarm_2_link+tiagoarm_3_link"), dParamVel, control[2]);
    dJointSetAMotorParam(motor_.at("tiagoarm_2_link+tiagoarm_3_link"), dParamFMax, 50);

    dJointSetAMotorParam(motor_.at("tiagoarm_3_link+tiagoarm_4_link"), dParamVel, control[3]);
    dJointSetAMotorParam(motor_.at("tiagoarm_3_link+tiagoarm_4_link"), dParamFMax, 50);

    dJointSetAMotorParam(motor_.at("tiagoarm_4_link+tiagoarm_5_link"), dParamVel, control[4]);
    dJointSetAMotorParam(motor_.at("tiagoarm_4_link+tiagoarm_5_link"), dParamFMax, 50);

//    dJointSetAMotorParam(motor_.at("tiagoarm_5_link+tiagoarm_6_link"), dParamVel, 0);
//    dJointSetAMotorParam(motor_.at("tiagoarm_5_link+tiagoarm_6_link"), dParamFMax, 100);


    dJointSetAMotorParam(motor_.at("tiagoarm_6_link+tiagoarm_7_link"), dParamVel,control[5]);
    dJointSetAMotorParam(motor_.at("tiagoarm_6_link+tiagoarm_7_link"), dParamFMax, 50);
}

//! This function describe that how the robot will interact with the environment return true if the collision between
//! robot and the object is allowed, false otherwise
bool TiagoEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{
    std::string bodyType1 = geomNames_.at(geom1);
    std::string bodyType2 = geomNames_.at(geom2);
    if((bodyType1 == "robBody" && bodyType2 == "fixed")
            || (bodyType1 == "fixed" && bodyType2 == "robBody"))
    {
        return false;
    }
    else
        if((bodyType1 == "tcp" && bodyType2 == "fixed")
                || (bodyType1 == "fixed" && bodyType2 == "tcp"))
        {
            return false;
        }
        else
            if((bodyType1 == "tcp" && bodyType2 == "robBody")
                    || (bodyType1 == "robBody" && bodyType2 == "tcp"))
            {
                return true;
            }
            else
                if((bodyType1 == "robBody" && bodyType2 == "robBody"))
                {
                    return true;
                }
                else
                    if((bodyType1 == "tcp" && bodyType2 == "target")
                            || (bodyType1 == "target" && bodyType2 == "tcp"))
                    {
                        return false;
                    }

                    else
                        if(bodyType1 == "freeManipulatable" && bodyType2 == "freeManipulatable")

                        {
                             return true;
                        }
                        else
                            if((bodyType1 == "freeManipulatable" && bodyType2 == "target")
                                    || (bodyType1 == "target" && bodyType2 == "freeManipulatable"))
                            {
                                return false;
                            }
                            else
                                if((bodyType1 == "freeManipulatable" && bodyType2 == "robBody")
                                        || (bodyType1 == "robBody" && bodyType2 == "freeManipulatable"))
                                {
//                                    bool flag1=false,flag2=false;

//                                    if(bodyType1=="robBody")
//                                    {
//                                        const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom1));
//                                        const dReal *pos = dBodyGetPosition(dGeomGetBody(geom1));

//                                        double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
//                                        if(velocity < 1.5)
//                                            flag1=true;
//                                        if(pos[2]>Instknowledge->taskRegion.at(0).z_min && pos[2]<=Instknowledge->taskRegion.at(0).z_max)
//                                            flag2=true;
//                                    }
//                                    else

//                                    {
//                                        const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom2));
//                                        const dReal *pos = dBodyGetPosition(dGeomGetBody(geom2));

//                                        double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
//                                        if(velocity < 1.5)
//                                            flag1=true;
//                                        if(pos[2]>Instknowledge->taskRegion.at(0).z_min && pos[2]<=Instknowledge->taskRegion.at(0).z_max)
//                                            flag2=true;
//                                    }
//                                    return (flag1 && flag2);
                                    return true;
                                }
                                else
                                    if((bodyType1 == "freeManipulatable" && bodyType2 == "fixed")
                                            || (bodyType1 == "fixed" && bodyType2 == "freeManipulatable"))
                                    {
                                        return true;
                                    }
                                    else
                                        if((bodyType1 == "freeManipulatable" && bodyType2 == "tcp")
                                                || (bodyType1 == "tcp" && bodyType2 == "freeManipulatable"))
                                        {
//                                            bool flag1=false,flag2=false;
//                                            if(bodyType1=="tcp")
//                                            {
//                                                const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom1));
//                                                const dReal *pos = dBodyGetPosition(dGeomGetBody(geom1));

//                                                double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
//                                                if(velocity < 1.5)
//                                                    flag1=true;
//                                                if(pos[2]>Instknowledge->taskRegion.at(0).z_min && pos[2]<=Instknowledge->taskRegion.at(0).z_max)
//                                                    flag2=true;
//                                            }
//                                            else

//                                            {
//                                                const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom2));
//                                                const dReal *pos = dBodyGetPosition(dGeomGetBody(geom2));

//                                                double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
//                                                if(velocity < 1.5)
//                                                    flag1=true;
//                                                if(pos[2]>Instknowledge->taskRegion.at(0).z_min && pos[2]<=Instknowledge->taskRegion.at(0).z_max)
//                                                    flag2=true;
//                                            }

//                                            return (flag1 && flag2);
                                            return true;
                                        }
                                            else
                                                if((bodyType1 == "tcp" && bodyType2 == "odeGround")
                                                        || (bodyType1 == "odeGround" && bodyType2 == "tcp"))
                                                {
                                                    return false;
                                                }
                                                else
                                                    if((bodyType1 == "robBody" && bodyType2 == "coManipulatable")
                                                            || (bodyType1 == "coManipulatable" && bodyType2 == "robBody"))
                                                    {
                                                        std::cout<<"Robot coManipulatable Evaluation: "<<std::endl;
                                                        const dReal *pos1 = dGeomGetPosition(geom1);
                                                        const dReal *pos2 = dGeomGetPosition(geom2);
                                                        if(bodyType1 == "robBody")
                                                        {
                                                            RigidBody rbody = Instknowledge->getManipulationConstraints(geom2);
                                                            bool isColAllowed=rbody.isCollisionAllowed(pos1[0],pos1[1]);

                                                            if(isColAllowed )
                                                            {
                                                                std::cout<<"Robot is in mRegion: "<<pos1[0]<<" , "<<pos1[1]<<std::endl;
                                                                return true;
                                                            }
                                                            else
                                                                return false;
                                                        }
                                                        else if(bodyType2 == "robBody")
                                                        {
                                                            RigidBody rbody = Instknowledge->getManipulationConstraints(geom1);
                                                            bool isColAllowed=rbody.isCollisionAllowed(pos2[0],pos2[1]);
                                                            if(isColAllowed )
                                                            {
                                                                std::cout<<"Robot is in mRegion: "<<pos2[0]<<" , "<<pos2[1]<<std::endl;
                                                                return true;
                                                            }
                                                            else
                                                                return false;
                                                        }
                                                    }
                                                    else
                                                        if((bodyType1 == "robBody" && bodyType2 == "table")
                                                                || (bodyType1 == "table" && bodyType2 == "robBody"))

                                                        {
                                                            return false;
                                                        }
                                                        else
                                                            if((bodyType1 == "tcp" && bodyType2 == "coManipulatable")
                                                                    || (bodyType1 == "coManipulatable" && bodyType2 == "tcp"))

                                                            {
                                                                std::cout<<"Robot coManipulatable Evaluation: "<<std::endl;
                                                                const dReal *pos1 = dGeomGetPosition(geom1);
                                                                const dReal *pos2 = dGeomGetPosition(geom2);
                                                                if(bodyType1 == "tcp")
                                                                {
                                                                    RigidBody rbody = Instknowledge->getManipulationConstraints(geom2);
                                                                    bool isColAllowed=rbody.isCollisionAllowed(pos1[0],pos1[1]);

                                                                    if(isColAllowed )
                                                                    {
                                                                        std::cout<<"Robot is in mRegion: "<<pos1[0]<<" , "<<pos1[1]<<std::endl;
                                                                        return true;
                                                                    }
                                                                    else
                                                                        return false;
                                                                }
                                                                else if(bodyType2 == "tcp")
                                                                {
                                                                    RigidBody rbody = Instknowledge->getManipulationConstraints(geom1);
                                                                    bool isColAllowed=rbody.isCollisionAllowed(pos2[0],pos2[1]);
                                                                    if(isColAllowed )
                                                                    {
                                                                        std::cout<<"Robot is in mRegion: "<<pos2[0]<<" , "<<pos2[1]<<std::endl;
                                                                        return true;
                                                                    }
                                                                    else
                                                                        return false;
                                                                }
                                                            }
                                                            else  if((bodyType1 == "tcp" && bodyType2 == "table")
                                                                     || (bodyType1 == "table" && bodyType2 == "tcp"))

                                                            {

                                                                return false;
                                                            }
                                                            else
                                                                return true;
    //  return true;
}


/*! This is the reimplementation of the virtual function of OpenDEEnvironment. This method set the parameters for the contact
 * (for the contact dynamics), like what will be the value of friction coefficient, etc.
 */
void TiagoEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{

    contact.surface.mode = dContactSoftCFM | dContactSoftERP;//|dContactBounce ;
    std::string geom1_body = geomNames_.at(geom1);
    std::string geom2_body = geomNames_.at(geom2);
//    //std::cout<<"Bodies are: "<<geom1_body<<" : "<<geom2_body<<std::endl;
    if ((geom1_body == "table" && geom2_body == "odeGround") ||
            (geom1_body == "odeGround" && geom2_body == "table" ))
    {
        contact.surface.mu = dInfinity;
        contact.surface.soft_erp = 0.5;
        contact.surface.soft_cfm = 0.1;
    }
    else
//        if ((geom1_body == "fixed" && geom2_body == "table") ||
//                (geom1_body == "table" && geom2_body == "fixed" ))
//        {
//            contact.surface.mu = 1;
//            contact.surface.soft_erp =1;
//            contact.surface.soft_cfm = 0.3;
//        }
//        else
//            if ((geom1_body == "freeManipulatable" && geom2_body == "table")||
//                    (geom1_body == "table" && geom2_body == "freeManipulatable" ))
//            {

//                contact.surface.mu = 0.1;
//                contact.surface.soft_erp =0.9;
//                contact.surface.soft_cfm = 0.3;
//            }
//            else
//                if((geom1_body == "tcp" && geom2_body == "target")
//                        || (geom2_body == "target" && geom1_body == "tcp"))
//                {
//                    contact.surface.mu=0.1;
//                    contact.surface.soft_erp =1;
//                    contact.surface.soft_cfm = 0.1;
//                }
//                else
 //               {
//                    contact.surface.soft_erp =1;
//                    contact.surface.soft_cfm = 0.1;
//                    contact.surface.slip1 = 0.1;
//                    contact.surface.slip2 = 0.1;

//                    contact.surface.mu = 0.03;

   //}
       // else
    {
//    contact.surface.mu = 0.01;
//    contact.surface.soft_cfm = 0.01;
//    contact.surface.bounce=0.005;
//    contact.surface.bounce_vel=0.005;
//    contact.surface.soft_erp = 0.6;


    contact.surface.mu = 0.02;
    contact.surface.soft_cfm = 0.001;
    //contact.surface.bounce=0.05;
    //contact.surface.bounce_vel=0.05;
    contact.surface.soft_erp = 0.2;

}


}

TiagoStateValidityChecker::TiagoStateValidityChecker(const oc::SpaceInformationPtr &si):oc::OpenDEStateValidityChecker(si)
{

}
//This function will determine the validity of the KauthamDE state, by evaluating that the robot is within
//the bound, the collisions are valid and robot is satisfying all the other contraints imposed by the Instantiated
//knowledge such as the pose of tcp.
bool TiagoStateValidityChecker::isValid(const ob::State *state) const
{
    std::vector<bool> flag(2);
    flag[0]=flag[1]=false;
    const oc::OpenDEStateSpace::StateType *s = state->as<oc::OpenDEStateSpace::StateType>();
    //to determine that the position of tcp is valid in the task Region
    //(z-axis of the tcp must be less than the given threshold
    unsigned int tcp_bodies_size = ((KauthamDEEnvironment*)osm_->getEnvironment().get())->tcp_bodies.size();
    unsigned int bodyIndex=0;
    for (unsigned int i=0;i<tcp_bodies_size;i++)
    {
        bodyIndex=((KauthamDEEnvironment*)osm_->getEnvironment().get())->
                geomIndex.at(((KauthamDEEnvironment*)osm_->getEnvironment().get())->tcp_bodies[i]);
        const dReal *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(bodyIndex);
        //Region r=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion[0];
        for(unsigned int i=0;i<((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.size();i++)
        {
            double xmin=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).x_min;
            double xmax=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).x_max;
            double ymin=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).y_min;
            double ymax=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).y_max;
            double zmin=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).z_min;
            double zmax=((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->taskRegion.at(i).z_max;

            if( pos[0] >  xmin &&  pos[1] >  ymin &&
                    pos[0] <  xmax &&  pos[1] <  ymax)
            {
                //std::cout<<"Region Dimensions: ["<< xmin<<" , " <<xmax<<" ] ["<< ymin<<" , " <<ymax<<" ] ["<< zmin<<" , " <<zmax<<" ] "<<std::endl;
                //std::cout<<"Gripper Pos: ["<< pos[0]<<" , " <<pos[1]<<" , "<<pos[2]<<" ] "<<std::endl;

                if(pos[2]>zmin && pos[2]<zmax)
                    flag[0]=true;
                else
                    flag[0]=false;

            }
            else
                flag[0]=true;
        }
    }
    // if we know the value of the validity flag for this state, we return it
    if (s->collision & (1 << oc::OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT))
    {
        flag[1]=s->collision & (1 << oc::OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT);
        return flag[0] && flag[1];
    }
    // if not, we compute it:
    flag[1]=false;
    if (!osm_->evaluateCollision(state))
        flag[1] = osm_->satisfiesBoundsExceptRotation(s);
    if (flag[1])
        s->collision &= (1 << oc::OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT);
    //std::vector<dBodyID> &bodies = osm_->getEnvironment().get()->stateBodies_;
    //((KauthamDEEnvironment*)osm_->getEnvironment().get())->Instknowledge->updateKnowledge(bodies);
    // mark the fact we know the value of the validity bit
    s->collision &= (1 << oc::OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT);
    //std::cout<<"flags   "<<flag[0]<<" "<<flag[1]<<std::endl;
    return flag[0] && flag[1];
}
/////////////////////////////////////////////////////////////////////////////////
///                      Kuka State Space
/////////////////////////////////////////////////////////////////////////////////


TiagoStateSpace::TiagoStateSpace(const oc::OpenDEEnvironmentPtr &env, Planner *planner) : oc::OpenDEStateSpace(env)
{
    _planner=planner;
}
TiagoStateSpace::~TiagoStateSpace(){}

double TiagoStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;
    //for(unsigned int j=0;j<_planner->wkSpace()->getNumRobots();j++)
    {
        for(unsigned int i=0;i<8;i++)//_planner->wkSpace()->getRobot(0)->getNumLinks()-2;i++)
        {
            const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(i);
            const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(i);
            const ob::SO3StateSpace::StateType &r1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(i);
            const ob::SO3StateSpace::StateType &r2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(i);
            double dx = fabs(p1[0] - p2[0]);
            double dy = fabs(p1[1] - p2[1]);
            double dz = fabs(p1[2] - p2[2]);
            double tD=0.5*sqrt(dx*dx + dy*dy + dz*dz);
            double rD = 0.7*acos(fabs(r1.x*r2.x+r1.y*r2.y+r1.z*r2.z+r1.w*r2.w));
            //std::cout<<"translational  and rotational distances are : "<<tD<<"   "<<rD<<std::endl;
            distance = distance + tD+rD;
        }
    }
    return distance;
}

void TiagoStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new TiagoStateProjectionEvaluator(this)));
}
/////////////////////////////////////////////////////////////////////////////////
///                Kuka Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
TiagoStateProjectionEvaluator::TiagoStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int TiagoStateProjectionEvaluator::getDimension(void) const
{
    return 3;
}
void TiagoStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(3);
    cellSizes_[0] = 0.001;
    cellSizes_[1] = 0.001;
    cellSizes_[2] = 0.001;
}

void TiagoStateProjectionEvaluator::project(const ob::State *state, Kautham::VectorRef projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(8);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];
}

///////////////////////////////////////////////////////////////////
TiagoControlSampler::TiagoControlSampler(const oc::ControlSpace *cm, Planner *planner) : oc::RealVectorControlUniformSampler(cm)
{

    _planner=planner;
    goal=((KauthamDEPlanner*)_planner)->goalstate;
    numcontrolssmp=3;
    sii=((KauthamDEPlanner*)_planner)->sii;

}
void TiagoControlSampler::sample(oc::Control *control)
{
    const unsigned int dim = space_->getDimension();
    const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace*>(space_)->getBounds();

    oc::RealVectorControlSpace::ControlType *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType*>(control);
    for (unsigned int i = 0 ; i < dim ; ++i)
        rcontrol->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}
////////////////////////////////////////////////////////////////////////////////////////////////

TiagoControlSpace::TiagoControlSpace(const ob::StateSpacePtr &m, Planner *planner) : oc::OpenDEControlSpace(m)
{
    _planner=planner;
}
oc::ControlSamplerPtr TiagoControlSpace::allocControlSampler(void) const
{
    return oc::ControlSamplerPtr(new TiagoControlSampler(this,_planner));
}



}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL


