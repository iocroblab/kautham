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
#if !defined(_KauthamDELTL2DPlanner_H)
#define _KauthamDELTL2DPlanner_H
#define dDOUBLE

#include "KauthamOpenDEPlanner.h"
#include "../environment/ConstraintAwareRobotEnvironment/ConstraintAware2DRobotEnvironment.h"
#include "../environment/SimpleRobotEnvironment/CarEnvironment.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>

#include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
#include <ompl/control/planners/ltl/PropositionalDecomposition.h>
#include <ompl/control/planners/ltl/Automaton.h>
#include <ompl/control/planners/ltl/ProductGraph.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/ltl/LTLProblemDefinition.h>

#include "LTLPoblemDiscription.h"


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

//class LTLProblemSetup
//{
//public:
//    LTLProblemSetup(oc::PropositionalTriangularDecomposition* decomp,WorkSpace* ws);
//    std::string TypeOfAutomaton;
//    void PropositionsAndHoles(oc::PropositionalTriangularDecomposition* decomp);

//};
/////////////////////////////////////////////////////////////////////////////////////////////////
// Class KauthamOpenDEPlanner
/////////////////////////////////////////////////////////////////////////////////////////////////
//! This class implement the KPIECE Planner (provied by the ompl) to plan in dynamic enviroment.
class KauthamDELTL2DPlanner: public KauthamDEPlanner
{
public:
    //! Constructor create dynamic enviroment and setup all the necessary parameters for planning.
    KauthamDELTL2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
    ~KauthamDELTL2DPlanner();
    bool setParameters();//!< this function set the planning parameters for KPIECE.



};

}
 /** @}   end of Doxygen module "Planner */
}

#endif  //_KauthamDELTL2DPlanner_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

