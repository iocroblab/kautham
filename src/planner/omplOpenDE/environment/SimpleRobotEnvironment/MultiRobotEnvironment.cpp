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
#include "MultiRobotEnvironment.h"

namespace Kautham {
namespace omplcplanner{

//! Constructor create the ODE 3Robot enviroment and setup the parameters for ODE.
MultiRobotEnvironment::MultiRobotEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlSteps,KthReal maxControlSteps, KthReal erp, KthReal cfm):KauthamDEEnvironment(ws, maxspeed,maxContacts,minControlSteps,maxControlSteps, erp, cfm)
{
    //potser he de fer lo de setKinematic aquí o en el applyControl;
    SetPlanningParameters();
}
MultiRobotEnvironment::~MultiRobotEnvironment()
{

}

//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
unsigned int MultiRobotEnvironment::getControlDimension(void) const
{
    return 4;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
*   which describe the control bounds,the bounding box to performe sampling control.
*/
void MultiRobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{
    lower.resize(4);
    upper.resize(4);
    for(int i=0; i < 4; i++)
    {
        lower[i]= -10;
        upper[i]= 10;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
*   that explain the way that how the control will apply on 3Robot system.
*/
void MultiRobotEnvironment::applyControl (const double *control) const
{
    dBodyAddForce(bodies[0], control[0], control[1],0.0);
    dBodyAddForce(bodies[1], control[2], control[3],0.0);
    //dBodyAddForce(bodies[2], control[4], control[5],0.0);

   /* {
    const dReal *pos = dBodyGetPosition(bodies[0]);
    dMass mass;
    dBodyGetMass(bodies[0], &mass);
    float minf = 0.5 * mass.mass * (-9.8);
    std::cout<<" Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    std::cout<<" Controls are  "<<control[0]<<" "<<control[1];
    std::cout<<" minimum required force is   "<< minf;
    std::cout<<" mass is "<< mass.mass <<std::endl;
    }
    {
    const dReal *pos = dBodyGetPosition(bodies[1]);
    dMass mass;
    dBodyGetMass(bodies[1], &mass);
    float minf = 0.5 * mass.mass * (-9.8);
    std::cout<<" Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    std::cout<<" Controls are  "<<control[2]<<" "<<control[3];
    std::cout<<" minimum required force is   "<< minf;
    std::cout<<" mass is "<< mass.mass <<std::endl;
    }
    {
    const dReal *pos = dBodyGetPosition(bodies[2]);
    dMass mass;
    dBodyGetMass(bodies[2], &mass);
    float minf = 0.5 * mass.mass * (-9.8);
    std::cout<<" Position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
    std::cout<<" Controls are  "<<control[4]<<" "<<control[5];
    std::cout<<" minimum required force is   "<< minf;
    std::cout<<" mass is "<< mass.mass <<std::endl;
    }*/
}

bool MultiRobotEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
{

//    dBodyID b1 = dGeomGetBody(geom1);
//    dBodyID b2 = dGeomGetBody(geom2);
//    if(b1 == bodies[3] && (b2 == bodies[0]
//                       ||  b2 == bodies[1]
//                       ||  b2 == bodies[2]))
//        return true;
//        else if(b2 == bodies[3] && (b1 == bodies[0]
//                                ||  b1 == bodies[1]
//                                ||  b1 == bodies[2]))
//            return true;
//            else
               return true;
}

/*! This is the reimplementation of the virtual function of OpenDEEnvironment
* This method set the parameters for the contact, like what will be the value
* of friction coefficient, etc.
*/
void MultiRobotEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
{
    contact.surface.mode = dContactSoftCFM | dContactApprox1;
    contact.surface.mu = 2;
    contact.surface.soft_erp = _erp;
    contact.surface.soft_cfm = _cfm;
}
/////////////////////////////////////////////////////////////////////////////////
///                      KauthaDE 3Robot StateSpace
/////////////////////////////////////////////////////////////////////////////////
MultiRobotStateSpace::MultiRobotStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
{
}
MultiRobotStateSpace::~MultiRobotStateSpace(){}

double MultiRobotStateSpace::distance(const ob::State *s1, const ob::State *s2) const
{

    double distance = 0.0;
    for(int i=0;i<2;i++)
    {
        const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(i);
        const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(i);
         double dx = fabs(p1[0] - p2[0]);
         double dy = fabs(p1[1] - p2[1]);
         //double dz = fabs(p1[1] - p2[1]);

        distance = distance + sqrt(dx * dx + dy );//* dy + dz *dz);
    }
    return distance;
}

void MultiRobotStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new MultiRobotStateProjectionEvaluator(this)));
}
/////////////////////////////////////////////////////////////////////////////////
///                KauthamDE 3Robot ProjectionEvaluator
/////////////////////////////////////////////////////////////////////////////////
MultiRobotStateProjectionEvaluator::MultiRobotStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{

}

unsigned int MultiRobotStateProjectionEvaluator::getDimension(void) const
{
    return 4;
}
void MultiRobotStateProjectionEvaluator :: defaultCellSizes(void)
{
    cellSizes_.resize(4);
    cellSizes_[0] = 1.0;
    cellSizes_[1] = 1.0;
    cellSizes_[2] = 1.0;
    cellSizes_[3] = 1.0;
//    cellSizes_[4] = 1.0;
//    cellSizes_[5] = 1.0;
//    cellSizes_[6] = 1.0;
//    cellSizes_[7] = 1.0;
//    cellSizes_[8] = 1.0;
}

void MultiRobotStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{

    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
    //projection[2] = pos[2];

    const double *pos1 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(1);
    projection[2] = pos1[0];
    projection[3] = pos1[1];
    //projection[5] = pos1[2];

//    const double *pos2 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(2);
//    projection[4] = pos2[0];
//    projection[5] = pos2[1];
    //projection[8] = pos2[2];


}

}
}

#endif//KAUTHAM_USE_ODE
#endif// KAUTHAM_USE_OMPL
