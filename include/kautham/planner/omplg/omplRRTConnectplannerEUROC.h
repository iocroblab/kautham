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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

#if !defined(_omplRRTconnectPLANNEReuroc_H)
#define _omplRRTconnectPLANNEReuroc_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;


#include <kautham/planner/omplg/omplplanner.h>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

using namespace std;

namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */
  namespace omplplanner{
    class omplRRTConnectPlannerEUROC:public omplPlanner {
	    public:
        omplRRTConnectPlannerEUROC(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
        ~omplRRTConnectPlannerEUROC();

        int _filtersamples;//! flag to set/unset the filtering of samples included in the validity check

        bool setParameters();
        bool filtersample(Sample* smp);
        bool trySolve();//reimplemented

         KthReal _Range;
	  };
  }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplRRTconnectPLANNEReuroc_H

