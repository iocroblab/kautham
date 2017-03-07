/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#if !defined(_omplFOSRRTPlanner_H)
#define _omplFOSRRTPlanner_H

#if defined(KAUTHAM_USE_OMPL)
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <kautham/planner/omplg/omplplanner.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>



namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

namespace Kautham {
namespace omplplanner {
/** \addtogroup GeometricPlanners
 *  @{
 */
class omplFOSRRTPlanner:public omplPlanner {
public:
    omplFOSRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
    ~omplFOSRRTPlanner();

    void setSynergyTree(string filename);

    bool setParameters();

    KthReal _Range;
    KthReal _TimeStep;
    KthReal _GoalBias;
    KthReal _PMDBias;
};
/** @}   end of Doxygen module */
}
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplFOSRRTPlanner_H
