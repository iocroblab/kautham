
#if !defined(omplTSRRT_H)
#define omplTSRRT_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <kautham/planner/omplg/omplplanner.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

namespace Kautham {

  namespace omplplanner{

    class omplTSRRT:public omplPlanner {
        public:
        omplTSRRT(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
        ~omplTSRRT();

        bool setParameters();

         double _Range;
         double _GoalBias;

      };
  }
}

#endif // KAUTHAM_USE_OMPL
#endif  //omplTSRRT_H
