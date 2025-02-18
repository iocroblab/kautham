/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Nestor Garcia Hidalgo, Jan Rosell */

#if !defined(_omplFOSRRTSTARPLANNER_H)
#define _omplRRTSTARPLANNER_H

#if defined(KAUTHAM_USE_OMPL)

#include "omplplanner.h"
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */
  namespace omplplanner{
    class omplRRTStarPlanner:public omplPlanner {
    public:
        omplRRTStarPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                              SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);

        ~omplRRTStarPlanner();

        bool trySolve();//!< Overloaded trySolve function to include evaluation of final path cost

        bool setParameters();
        ob::OptimizationObjectivePtr _lengthopti;
        ob::OptimizationObjectivePtr _clearanceopti;
        ob::OptimizationObjectivePtr _opti;
    };
  }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplRRTStarPlanner_H
