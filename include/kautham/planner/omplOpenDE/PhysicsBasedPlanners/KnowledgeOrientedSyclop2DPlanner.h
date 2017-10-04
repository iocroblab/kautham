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

/* Author:  Muhayyuddin */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_KnowledgeOrientedSyclop2DPlanner_H)
#define _KnowledgeOrientedSyclop2DPlanner_H
#define dDOUBLE

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
#include <kautham/planner/omplOpenDE/Setup/ConstraintAware2DRobotEnvironment.h>
#include <kautham/planner/omplc/omplcplanner.h>

#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#define _USE_MATH_DEFINES

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;

namespace Kautham {
/** \addtogroup Planner
 *  @{
 */

namespace omplcplanner{

/////////////////////////////////////////////////////////////////////////////////////////////////
// Class KauthamOpenDEPlanner
/////////////////////////////////////////////////////////////////////////////////////////////////
//! This class implement the Syclop Planner (provied by the ompl) to plan in dynamic enviroment.
class KnowledgeOrientedSyclop2DPlanner: public KauthamDEPlanner
{
public:
    //! Constructor create dynamic enviroment and setup all the necessary parameters for planning.
    KnowledgeOrientedSyclop2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
    ~KnowledgeOrientedSyclop2DPlanner();
    bool setParameters();//!< this function set the planning parameters for KPIECE.

    KthReal _Range;
    KthReal _GoalBias;
    int _NumFreeVolumeSamples;
    KthReal _getProbShortestPathLead;
    KthReal _getProbAddingToAvailableRegions;
    int _getNumRegionExpansions;
    KthReal _getProbAbandonLeadEarly;
    int _getNumTreeExpansions;

};

}
 /** @}   end of Doxygen module "Planner */
}

#endif  //_KauthamOpenDEKPIECEplanner_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

