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

/* Author: Nestor Garcia Hidalgo */

#if !defined(_omplLazyTRRTPLANNER_H)
#define _omplLazyTRRTPLANNER_H

#if defined(KAUTHAM_USE_OMPL)

#include <kautham/planner/omplg/omplplanner.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

namespace Kautham {
    /** \addtogroup GeometricPlanners
    *  @{
    */
    namespace omplplanner {
        class omplLazyTRRTPlanner:public omplPlanner {
        public:
            omplLazyTRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
            ~omplLazyTRRTPlanner();

            bool setParameters();
            bool trySolve();

            bool setPotentialCost(string filename);

            KthReal _Range;
            KthReal _GoalBias;
            KthReal _maxStatesFailed;//nFail_{max}
            KthReal _tempChangeFactor;//alpha
            KthReal _frontierThreshold;//delta
            KthReal _frontierNodesRatio;//rho

            ob::OptimizationObjectivePtr _opti;
        };
    }
    /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL

#endif  //_omplLazyTRRTPLANNER_H

