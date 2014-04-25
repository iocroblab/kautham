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

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "KauthamOpenDErobotEnvironment.h"



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


  //! Constructor
  KauthamDErobotEnvironment::KauthamDErobotEnvironment(WorkSpace* ws, KthReal maxspeed):KauthamDEEnvironment(ws, maxspeed)
  {
      SetPlanningParameters();
      //potser he de fer lo de setKinematic aquí o en el applyControl
  }




void KauthamDErobotEnvironment::SetPlanningParameters()
{
    stepSize_ = 0.05;
    maxContacts_ = 3;
    minControlSteps_ = 10;
    maxControlSteps_ = 500;
}

unsigned int KauthamDErobotEnvironment::getControlDimension(void) const
{
    return _NumLinksFirstRobot;
}
void KauthamDErobotEnvironment::getControlBounds(std::vector< double > &lower, std::vector< double > &upper) const
{

    lower.resize(_NumLinksFirstRobot);
    upper.resize(_NumLinksFirstRobot);
    for(int i=0; i < _NumLinksFirstRobot; i++)
    {
        lower[i]=-_maxspeed;
        upper[i]=_maxspeed;
    }
}
void KauthamDErobotEnvironment::applyControl (const double *control) const
{
    for(int i=0; i < _NumLinksFirstRobot; i++)
    {
        dBodySetKinematic(stateBodies_[i]);
        dBodySetLinearVel(stateBodies_[i],control[0],control[1],control[2]);

    }


}


}

}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
