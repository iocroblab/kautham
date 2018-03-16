
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
#include <kautham/planner/omplOpenDE/Setup/YumiEnvironment.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>


namespace Kautham {
namespace omplcplanner{


//! Constructor create the ODE table enviroment and setup the parameters for ODE.
YumiEnvironment::YumiEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm,bool isKchain):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm, isKchain)
{
    SetPlanningParameters();
}
YumiEnvironment::~YumiEnvironment()
{

}
//! Setup the parameters for ODE.
//void KauthamDETX90Environment::SetPlanningParameters()
//{
//    stepSize_ =  _propagationStepSize;
//    maxContacts_ = _maxContacts;
//    minControlSteps_ = _minControlSteps;
//    maxControlSteps_ = _maxControlSteps;
//}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
unsigned int YumiEnvironment::getControlDimension(void) const
{
    if(_Planfor=='L'|| _Planfor=='R')
        return 9;
    else
        return 18;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
* which describe the control bounds,the bounding box to performe sampling control.
*/
void YumiEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    if(_Planfor=='L'|| _Planfor=='R')
    {
        for(int i=0; i < 9; i++)
        {
            lower[i]=-0.2;
            upper[i]= 0.2;
        }

    }
    else
    {
        for(int i=0; i < 18; i++)
        {
            lower[i]=-0.3;
            upper[i]= 0.3;
        }
    }
//            lower[0]=-0.4;
//            upper[0]= 0.1;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
* that explain how the control will apply.This function apply the control by
* setting the forces, velocities and torques.
*/
void YumiEnvironment::applyControl (const double *control) const
{
    if(_Planfor=='R')
    {
        dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_r"), dParamVel, control[0]);
        dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_1_r+yumilink_2_r"), dParamVel, control[1]);
        dJointSetAMotorParam(motor_.at("yumilink_1_r+yumilink_2_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_2_r+yumilink_3_r"), dParamVel, control[2]);
        dJointSetAMotorParam(motor_.at("yumilink_2_r+yumilink_3_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_3_r+yumilink_4_r"), dParamVel, control[3]);
        dJointSetAMotorParam(motor_.at("yumilink_3_r+yumilink_4_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_4_r+yumilink_5_r"), dParamVel, control[4]);
        dJointSetAMotorParam(motor_.at("yumilink_4_r+yumilink_5_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_5_r+yumilink_6_r"), dParamVel, control[5]);
        dJointSetAMotorParam(motor_.at("yumilink_5_r+yumilink_6_r"), dParamFMax, 100);
        dJointSetAMotorParam(motor_.at("yumilink_6_r+yumilink_7_r"), dParamVel, control[6]);
        dJointSetAMotorParam(motor_.at("yumilink_6_r+yumilink_7_r"), dParamFMax, 100);
        dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_r"), dParamVel, 0);
        dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_r"), dParamFMax, dInfinity);
        dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_l"), dParamVel, 0);
        dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_l"), dParamFMax, dInfinity);


//        dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamVel, 0);
//        dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamFMax, dInfinity);
//        dJointSetAMotorParam(motor_.at("yumilink_6_l+yumilink_7_l"), dParamVel, (0));
//        dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamVel, (0));
//        dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamFMax, dInfinity);
//        dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamVel, (0));
//        dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamFMax, dInfinity);

    }
    else
        if(_Planfor=='L')
        {
            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamVel, (control[0]));
            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamVel, (control[1]));
            dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamVel, (control[2]));
            dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamVel, (control[3]));
            dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamVel, (control[4]));
            dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamVel, (control[5]));
            dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_6_l+yumilink_7_l"), dParamVel, (0));
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamVel, 0);
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamFMax, 100);
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamVel, 0);
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamFMax, 100);
        }
        else
            if(_Planfor=='B')
        {
            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_r"), dParamVel, (control[0]));
            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_1_r+yumilink_2_r"), dParamVel, (control[1]));
            dJointSetAMotorParam(motor_.at("yumilink_1_r+yumilink_2_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_2_r+yumilink_3_r"), dParamVel, (control[2]));
            dJointSetAMotorParam(motor_.at("yumilink_2_r+yumilink_3_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_3_r+yumilink_4_r"), dParamVel, (control[3]));
            dJointSetAMotorParam(motor_.at("yumilink_3_r+yumilink_4_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_4_r+yumilink_5_r"), dParamVel, (control[4]));
            dJointSetAMotorParam(motor_.at("yumilink_4_r+yumilink_5_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_5_r+yumilink_6_r"), dParamVel, (control[5]));
            dJointSetAMotorParam(motor_.at("yumilink_5_r+yumilink_6_r"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_6_r+yumilink_7_r"), dParamVel, (control[6]));
            dJointSetAMotorParam(motor_.at("yumilink_6_r+yumilink_7_r"), dParamFMax, 300);
            dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_r"), dParamVel, (0));
            dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_r"), dParamFMax, 300);
            dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_l"), dParamVel, (0));
            dJointSetLMotorParam(motor_.at("yumilink_7_r+yumigripper_r_finger_l"), dParamFMax, 300);

            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamVel, (control[9]));
            dJointSetAMotorParam(motor_.at("yumibody+yumilink_1_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamVel, (control[10]));
            dJointSetAMotorParam(motor_.at("yumilink_1_l+yumilink_2_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamVel, (control[11]));
            dJointSetAMotorParam(motor_.at("yumilink_2_l+yumilink_3_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamVel, (control[12]));
            dJointSetAMotorParam(motor_.at("yumilink_3_l+yumilink_4_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamVel, (control[13]));
            dJointSetAMotorParam(motor_.at("yumilink_4_l+yumilink_5_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamVel, (control[14]));
            dJointSetAMotorParam(motor_.at("yumilink_5_l+yumilink_6_l"), dParamFMax, 300);
            dJointSetAMotorParam(motor_.at("yumilink_6_l+yumilink_7_l"), dParamVel, (control[15]));
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamVel, (0));
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_r"), dParamFMax, 300);
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamVel, (0));
            dJointSetLMotorParam(motor_.at("yumilink_7_l+yumigripper_l_finger_l"), dParamFMax, 300);
        }

}

bool YumiEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{
    std::string bodyType1 = geomNames_.at(geom1);
    std::string bodyType2 = geomNames_.at(geom2);
    if((bodyType1 == "base" && bodyType2 == "table")
            || (bodyType1 == "table" && bodyType2 == "base"))
    {
        return true;
    }
    else
    if((bodyType1 == "robBody" && bodyType2 == "fixed")
            || (bodyType1 == "fixed" && bodyType2 == "robBody"))
    {
        return false;
    }
    else
        if((bodyType1 == "robBody" && bodyType2 == "robBody"))
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
                return false;
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
                                        //                       bool flag1 = CheckPoseUncertanity(geom1,target[0],Instknowledge->delta);
                                        //                       bool flag2 = CheckPoseUncertanity(geom2,target[0],Instknowledge->delta);
                                        //                       bool flag3;
                                        //                       if(bodyType1=="tcp")
                                        //                       {
                                        //                         const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom1));
                                        //                         double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
                                        //                         if(velocity > 1)
                                        //                             flag3=false;
                                        //                         else
                                        //                             flag3=true;
                                        //                       }
                                        //                       else
                                        //                       {
                                        //                           const dReal *vel = dBodyGetLinearVel(dGeomGetBody(geom2));
                                        //                           double velocity= sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
                                        //                           if(velocity > 1)
                                        //                               flag3=false;
                                        //                           else
                                        //                               flag3=true;
                                        //                       }
                                        //return (flag1 && flag2 && flag3);
                                        return true;
                                    }
                                    else
                                        if((bodyType1 == "freeManipulatable" && bodyType2 == "robBody")
                                                || (bodyType1 == "robBody" && bodyType2 == "freeManipulatable"))
                                        {
                                            //                        bool flag1 = CheckPoseUncertanity(geom1,target[0],Instknowledge->delta);
                                            //                        bool flag2 = CheckPoseUncertanity(geom2,target[0],Instknowledge->delta);
                                            // return (flag1 && flag2);
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
                                                    return false;
                                                }
                                                else
                                                    if((bodyType1 == "robBody" && bodyType2 == "target")
                                                            || (bodyType1 == "target" && bodyType2 == "robBody"))
                                                    {
                                                        return false;
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
    return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment
* This method set the parameters for the contact, like what will be the value
* of friction coefficient, etc.
*/
void YumiEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    //contact.surface.mode = dContactSoftCFM | dContactSoftERP;
    std::string geom1_body = geomNames_.at(geom1);
    std::string geom2_body = geomNames_.at(geom2);
    //std::cout<<"Bodies are: "<<geom1_body<<" : "<<geom2_body<<std::endl;

    contact.surface.mode = dContactSoftCFM | dContactSoftERP;

    contact.surface.mu = 0.02;
    contact.surface.soft_cfm = 0.015;
    contact.surface.soft_erp = 0.5;

        if ((geom1_body == "table" && geom2_body == "fxtable") ||
                (geom1_body == "fxtable" && geom2_body == "table" ))
        {
            contact.surface.mu = dInfinity;

        }
//           contact.surface.mu = 0.01;
//           contact.surface.soft_cfm = 0.01;
//           contact.surface.soft_erp = 0.5;

//    if ((geom1_body == "table" && geom2_body == "odeGround") ||
//            (geom1_body == "odeGround" && geom2_body == "table" ))
//    {
//        contact.surface.mu = dInfinity;
//        contact.surface.soft_erp = 0.5;
//        contact.surface.soft_cfm = 0.1;
//    }
//    else
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

//                contact.surface.mu = 0.2;
//                contact.surface.soft_erp =1;
//                contact.surface.soft_cfm = 0.01;
//            }
//            else
//                if((geom1_body == "tcp" && geom2_body == "target")
//                        || (geom2_body == "target" && geom1_body == "tcp"))
//                {
//                    contact.surface.mu=1;
//                    contact.surface.soft_erp =1;
//                    contact.surface.soft_cfm = 0.1;
//                }
//                else
//                    if((geom1_body == "table" && geom2_body == "target")
//                            || (geom2_body == "target" && geom1_body == "table"))
//                    {
//                        contact.surface.mu=0.2;
//                        contact.surface.soft_erp =1;
//                        contact.surface.soft_cfm = 0.01;
//                    }
//                else
//                {
//                    contact.surface.soft_erp =0.8;
//                    contact.surface.soft_cfm = 0.04;
//                    contact.surface.mu = 1.5;
//                }

//    if(beliefComputionPhase==false)
//    {

//        if ((geom1_body == "freeManipulatable" && geom2_body == "tcp")||
//                (geom1_body == "tcp" && geom2_body == "freeManipulatable" ))
//        {

//            contact.surface.mu = 1.5;
//            contact.surface.soft_erp =1;
//            contact.surface.soft_cfm = 0.1;
//            //std::cout<<"Fix value of Mu is  : "<<contact.surface.mu<<std::endl;

//        }
//        else
//            if ((geom1_body == "coManipulatable" && geom2_body == "table")||
//                    (geom1_body == "table" && geom2_body == "coManipulatable" ))
//            {
//                contact.surface.mu = 0.2;
//                contact.surface.soft_erp =1;
//                contact.surface.soft_cfm = 0.1;
//                //std::cout<<"Fix value of Mu is  : "<<contact.surface.mu<<std::endl;

//            }
//    }
//    else
//    {
//        std::random_device rd;
//        std::mt19937  Rgenerator(rd());
//        std::normal_distribution<double> mu(0.2,0.01); //todo: the values should be set through dynamic parameters
//        if ((geom1_body == "freeManipulatable" && geom2_body == "tcp")||
//                (geom1_body == "tcp" && geom2_body == "freeManipulatable" ))
//        {

//            contact.surface.mu =mu(Rgenerator);
//            contact.surface.soft_erp =1;
//            contact.surface.soft_cfm = 0.1;
//            //std::cout<<"Gaussian value of Mu is  : "<<contact.surface.mu<<std::endl;

//        }
//        else
//            if ((geom1_body == "coManipulatable" && geom2_body == "table")||
//                    (geom1_body == "table" && geom2_body == "coManipulatable" ))
//            {
//                contact.surface.mu = mu(Rgenerator);
//                contact.surface.soft_erp =1;
//                contact.surface.soft_cfm = 0.1;
//                //std::cout<<"Gaussian value of Mu is  : "<<contact.surface.mu<<std::endl;

//            }
//    }

}

/////////////////////////////////////////////////////////////////////////////////
///                     Yumi State Space
/////////////////////////////////////////////////////////////////////////////////
YumiStateSpace::YumiStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{

    ((KauthamDEEnvironment*)env.get())->getReferecneBodyIndex(((KauthamDEEnvironment*)env.get())->stateBodiesmap_,"yumi",&linkname,&index);
    for(unsigned int i=0;i<index.size();i++)
    std::cout<<"Reference body for Yumi State sapace : "<<linkname[i]<<" , "<<index[i]<<std::endl;
}
YumiStateSpace::~YumiStateSpace(){}

double YumiStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{
    double distance = 0.0;
    for(unsigned int i=0;i<9;i++)
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
    return distance;
}

void YumiStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new YumiStateProjectionEvaluator(this)));
}
/////////////////////////////////////////////////////////////////////////////////
///                TX90 Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
YumiStateProjectionEvaluator::YumiStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{
   Planfor = ((KauthamDEEnvironment*)space_->as<oc::OpenDEStateSpace>()->getEnvironment().get())->_Planfor;
}

unsigned int YumiStateProjectionEvaluator::getDimension(void) const
{
    if(Planfor=='L'|| Planfor=='R')
        return 3;
    else
        return 6;

}
void YumiStateProjectionEvaluator :: defaultCellSizes(void)
{

    if(Planfor=='L'|| Planfor=='R')
    {
        cellSizes_.resize(3);
        cellSizes_[0] = 0.001;
        cellSizes_[1] = 0.001;
        cellSizes_[2] = 0.001;
    }
    else
    {
        cellSizes_.resize(6);
        cellSizes_[0] = 0.01;
        cellSizes_[1] = 0.01;
        cellSizes_[2] = 0.01;
        cellSizes_[3] = 0.01;
        cellSizes_[4] = 0.01;
        cellSizes_[5] = 0.01;
    }
}

void YumiStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    std::vector<unsigned int> index;
    std::vector<std::string> linkname;
    ((KauthamDEEnvironment*)space_->as<oc::OpenDEStateSpace>()->getEnvironment().get())->getReferecneBodyIndex(((KauthamDEEnvironment*)space_->as<oc::OpenDEStateSpace>()->getEnvironment().get())->stateBodiesmap_,"yumi",&linkname,&index);
    unsigned int j=0;
    for(unsigned int i=0;i<index.size();i++)
    {
    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(index[i]);
    projection[j] = pos[0];
    j++;
    projection[j] = pos[1];
    j++;
    projection[j] = pos[2];
    j++;
    }

}
//////////////////////////////////////////////////////////////////

YumiStateValidityChecker::YumiStateValidityChecker(const oc::SpaceInformationPtr &si):oc::OpenDEStateValidityChecker(si)
{

}
//This function will determine the validity of the KauthamDE state, by evaluating that the robot is within
//the bound, the collisions are valid and robot is satisfying all the other contraints imposed by the Instantiated
//knowledge such as the pose of tcp.
bool YumiStateValidityChecker::isValid(const ob::State *state) const
{
    std::vector<bool> flag(2);
    flag[0]=flag[1]=false;
    const oc::OpenDEStateSpace::StateType *s = state->as<oc::OpenDEStateSpace::StateType>();
    //to determine that the position of tcp is valid in the task Region
    //(z-axis of the tcp must be less than the given threshold
    unsigned int tcp_bodies_size = ((KauthamDEEnvironment*)osm_->getEnvironment().get())->tcp_bodies.size();
    unsigned int bodyIndex=0;
    //std::cout<<"TCP Bodies Size is : "<<tcp_bodies_size<<std::endl;
    for (unsigned int i=0;i<tcp_bodies_size-2;i++)
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
    // mark the fact we know the value of the validity bit
    s->collision &= (1 << oc::OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT);
    //std::cout<<"flags   "<<flag[0]<<" "<<flag[1]<<std::endl;
    return flag[0] && flag[1];
}

///////////////////////////////////////////////////////////////////
YumiControlSampler::YumiControlSampler(const oc::ControlSpace *cm, Planner *planner) : oc::RealVectorControlUniformSampler(cm)
{

    _planner=planner;
    goal=((KauthamDEPlanner*)_planner)->goalstate;
    numcontrolssmp=3;
    sii=((KauthamDEPlanner*)_planner)->sii;

}

void YumiControlSampler::sample(oc::Control *control)
{
    const unsigned int dim = space_->getDimension();
    const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace*>(space_)->getBounds();

    oc::RealVectorControlSpace::ControlType *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType*>(control);
    for (unsigned int i = 0 ; i < dim ; ++i)
        rcontrol->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

////////////////////////////////////////////////////////////////////////////////////////////////

YumiControlSpace::YumiControlSpace(const ob::StateSpacePtr &m, Planner *planner) : oc::OpenDEControlSpace(m)
{
    _planner=planner;
}
oc::ControlSamplerPtr YumiControlSpace::allocControlSampler(void) const
{
    return oc::ControlSamplerPtr(new YumiControlSampler(this,_planner));
}

}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL


