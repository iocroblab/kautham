#if defined(KAUTHAM_USE_OMPL)

#include "ompl/geometric/planners/rrt/TSRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplTSRRT.h>
#include <kautham/planner/omplg/omplValidityChecker.h>

namespace Kautham {
  namespace omplplanner{

	//! Constructor
    omplTSRRT::omplTSRRT(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl TSRRT";
        _idName = "omplTSRRT";

        //create planner
        ob::PlannerPtr planner(new og::TSRRT(si, task_space));

        _Range=0.05;
        _GoalBias=(planner->as<og::TSRRT>())->getGoalBias();
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);

        space->setLongestValidSegmentFraction(0.01);

// RRT algoritmo

        if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
            space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
        }
        space->setup();
        planner->as<og::TSRRT>()->setGoalBias(_GoalBias);

        ss->setPlanner(planner);
    }

	//! void destructor
    omplTSRRT::~omplTSRRT(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplTSRRT::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
          ss->getPlanner()->as<og::TSRRT>()->setRange(_Range);
          space->setLongestValidSegmentFraction(0.01);
          if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
              space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
          }
          space->setup();
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<og::TSRRT>()->setGoalBias(_GoalBias);
        }
        else
          return false;

      }catch(...){
        return false;
      }
      return true;
    }
  }
}


#endif // KAUTHAM_USE_OMPL
