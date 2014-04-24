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

/* Author: Joan Fontanals Martinez, Muhayy ud din */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "KauthamOpenDEtableEnvironment.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>


#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>

using namespace std;

namespace Kautham {

namespace omplcplanner{


  //! Constructor create the ODE table enviroment and setup the parameters for ODE.

  KauthamDEtableEnvironment::KauthamDEtableEnvironment(WorkSpace* ws, KthReal maxspeed):KauthamDEEnvironment(ws, maxspeed)
  {

      //potser he de fer lo de setKinematic aquí o en el applyControl;
     SetPlanningParameters();
  }
  KauthamDEtableEnvironment::~KauthamDEtableEnvironment()
  {

  }
  //! Setup the parameters for ODE.
void KauthamDEtableEnvironment::SetPlanningParameters()
{

    stepSize_ = 0.02;
    maxContacts_ = 3;
    minControlSteps_ = 10;
    maxControlSteps_ = 50;
}
//! this is the reimplementation of the virtual function of OpenDEEnvironment, that describe the number of parameter used to describe control input.
unsigned int KauthamDEtableEnvironment::getControlDimension(void) const
{
    return 2;
    //return 3;
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * which describe the control bounds,the bounding box to performe sampling control.
 */
void KauthamDEtableEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(2);
    upper.resize(2);
    //lower.resize(3);
    //upper.resize(3);
    for(int i=0; i < 2; i++) //3 for 3d
    {
        lower[i]=-_maxspeed;
        upper[i]=_maxspeed;
    }
}
/*! this is the reimplementation of the virtual function of OpenDEEnvironment
 * that explain how the control will apply.This function apply the control by
 * setting the forces, velocities and torques.
 */
void KauthamDEtableEnvironment::applyControl (const double *control) const
{
    //es podria mirar de posar velocitats angulars etc...

    //dBodySetKinematic(stateBodies_[0]);
    //dBodySetLinearVel(stateBodies_[0],control[0],control[1],control[2]);
    dBodyAddForce(stateBodies_[0], control[0], control[1], control[2]);
    //dBodyAddForce(stateBodies_[4], .1,.1,.1);

}

// bool KauthamDEtableEnvironment::isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& /*contact*/) const
  //  {
    //if (geom1 == box[0] || geom2 == box[0])
   //     if (geom1 == avoid_avoid_box_geombox_geom || geom2 == avoid_box_geom)
   //     return false;
   // return true;
   // }

/*! This is the reimplementation of the virtual function of OpenDEEnvironment
 * This method set the parameters for the contact, like what will be the value
 * of friction coefficient, etc.
 */
 void KauthamDEtableEnvironment::setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
   {

   contact.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
   if (dGeomGetClass(geom1) == dSphereClass || dGeomGetClass(geom2) == dSphereClass)
       contact.surface.mu = 20;
   else
       contact.surface.mu = 0.5;
   contact.surface.slip1 = 0.0;
   contact.surface.slip2 = 0.0;
   contact.surface.soft_erp = 0.8;
   contact.surface.soft_cfm = 0.01;
   }

}
}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL


