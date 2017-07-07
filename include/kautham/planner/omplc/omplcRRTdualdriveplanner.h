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

/* Author: Alexander Perez, Jan Rosell */

#if !defined(_omplcRRTdualdrivePLANNER_H)
#define _omplcRRTdualdrivePLANNER_H


#if defined(KAUTHAM_USE_OMPL)
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/config.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
namespace ob = ompl::base;
namespace oc = ompl::control;


#include <kautham/planner/omplc/omplcplanner.h>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>


using namespace std;

namespace Kautham {
/** \addtogroup ControlPlanners
 *  @{
 */

  namespace omplcplanner{
    class omplcRRTdualdrivePlanner:public omplcPlanner {
	    public:
        omplcRRTdualdrivePlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, oc::SimpleSetup *ssptr);
        ~omplcRRTdualdrivePlanner();

        bool setParameters();

         KthReal _GoalBias;
         double _propagationStepSize;
         unsigned int _durationMax;
         unsigned int _durationMin;
         double _controlBound;

         int _onlyForward;
         double _dualdriveRadius;
         double _dualdriveDistance;
	  };
  }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplcRRTdualdrivePLANNER_H

