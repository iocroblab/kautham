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

///* Author: Muhayyuddin  */


//#if defined(KAUTHAM_USE_OMPL)
//#if defined(KAUTHAM_USE_ODE)
//#include "TestEnvironment.h"
//namespace Kautham {
//namespace omplcplanner{

////! Constructor create the ODE 3Robot enviroment and setup the parameters for ODE.
//KauthamDE2RobotEnvironment::KauthamDE2RobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
//{

//    //potser he de fer lo de setKinematic aquí o en el applyControl;
//}
//KauthamDE2RobotEnvironment::~KauthamDE2RobotEnvironment()
//{

//}
////! Setup the parameters for ODE.
////void KauthamDE2RobotEnvironment::SetPlanningParameters()
////{
////    stepSize_ =  _propagationStepSize;
////    maxContacts_ = _maxContacts;
////    minControlSteps_ = _minControlSteps;
////    maxControlSteps_ = _maxControlSteps;
////}
////! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
//unsigned int KauthamDE2RobotEnvironment::getControlDimension(void) const
//{
//    return 4;
//}
///*! this is the reimplementation of the virtual function of OpenDEEnvironment
//*   which describe the control bounds,the bounding box to performe sampling control.
//*/
//void KauthamDE2RobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
//{

//    lower.resize(4);
//    upper.resize(4);
//    for(int i=0; i < 4; i++)
//    {
//        lower[i]=-_maxspeed;
//        upper[i]=_maxspeed;
//    }
//}
///*! this is the reimplementation of the virtual function of OpenDEEnvironment
//*   that explain the way that how the control will apply on 3Robot system.
//*/
//void KauthamDE2RobotEnvironment::applyControl (const double *control) const
//{
//    for(int i=0;i<_motor.size();i++)
//    {
//    dJointSetAMotorParam(_motor[i], dParamVel, control[i]);
//    dJointSetAMotorParam(_motor[i], dParamFMax, dInfinity);
//    }

//}
//bool KauthamDE2RobotEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
//{

////        dBodyID b1 = dGeomGetBody(geom1);
////        dBodyID b2 = dGeomGetBody(geom2);
////        if(b1 == bodies[2] && (b2 == bodies[0]
////                           ||  b2 == bodies[1]))
////            return true;
////            else if(b2 == bodies[2] && (b1 == bodies[0]
////                                    ||  b1 == bodies[1]))
//                return true;
////                else
////                   return false;

//}

///*! This is the reimplementation of the virtual function of OpenDEEnvironment
//* This method set the parameters for the contact, like what will be the value
//* of friction coefficient, etc.
//*/
//void KauthamDE2RobotEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
//{
//    contact.surface.mode = 0;
//        contact.surface.mu = 0.5;
//}
///////////////////////////////////////////////////////////////////////////////////
/////                      KauthaDE 3Robot StateSpace
///////////////////////////////////////////////////////////////////////////////////
//KauthamDE2RobotStateSpace::KauthamDE2RobotStateSpace(const oc::OpenDEEnvironmentPtr &env) : KauthamDEStateSpace(env)
//{
//}
//KauthamDE2RobotStateSpace::~KauthamDE2RobotStateSpace(){}

//double KauthamDE2RobotStateSpace::distance(const ob::State *s1, const ob::State *s2) const
//{
//    double distance = 0.0;
//    //for(int i=0;i<2;i++)
//    {
//        const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
//        const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
//        distance = distance + fabs(p1[0]-p2[0])+fabs(p1[1]-p2[1])+fabs(p1[2]-p2[2]);
//    }
//    return sqrt(distance);
//}

//void KauthamDE2RobotStateSpace::registerProjections(void)
//{
//    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new KauthamDE2RobotStateProjectionEvaluator(this)));
//}
///////////////////////////////////////////////////////////////////////////////////
/////                KauthamDE 3Robot ProjectionEvaluator
///////////////////////////////////////////////////////////////////////////////////
//KauthamDE2RobotStateProjectionEvaluator::KauthamDE2RobotStateProjectionEvaluator(const ob::StateSpace *space) : KauthamDEStateProjectionEvaluator(space)
//{

//}

//unsigned int KauthamDE2RobotStateProjectionEvaluator::getDimension(void) const
//{
//    return 3;
//}
//void KauthamDE2RobotStateProjectionEvaluator :: defaultCellSizes(void)
//{
//    cellSizes_.resize(3);
//    cellSizes_[0] = 1.0;
//    cellSizes_[1] = 1.0;
//    cellSizes_[2] = 1.0;

//}
//void KauthamDE2RobotStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
//{

//    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(4);
//    projection[0] = pos[0];
//    projection[1] = pos[1];
//    projection[2] = pos[2];


//}
//}
//}

//#endif//KAUTHAM_USE_ODE
//#endif// KAUTHAM_USE_OMPL
