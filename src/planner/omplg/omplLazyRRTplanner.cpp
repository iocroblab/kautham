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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#if defined(KAUTHAM_USE_OMPL)
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplLazyRRTplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>



namespace Kautham {
  namespace omplplanner{

	//! Constructor
    omplLazyRRTPlanner::omplLazyRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl Lazy RRT Planner";
        _idName = "omplLazyRRT";

        //create planner
        ob::PlannerPtr planner(new og::LazyRRT(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<og::LazyRRT>())->getGoalBias();
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        planner->as<og::LazyRRT>()->setRange(_Range);
        //set the longest valid segment (i.e. segment without need to be collision-checked) as a fraction of
        //the maximum extend of the space (set to e.g. the diagonal of a R^n cubic space)
        space->setLongestValidSegmentFraction(0.01);
        //correct if necessary to allow that the (_Range/_validSegmentCount) be the longest valid segment,
        //so as to allow _validSegmentCount collisionchecks within a range step.
        if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
            space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
        }
        space->setup();//needed to update the setting of the longest valid segment
        planner->as<og::LazyRRT>()->setGoalBias(_GoalBias);

        //set the planner
        ss->setPlanner(planner);
    }

	//! void destructor
    omplLazyRRTPlanner::~omplLazyRRTPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplLazyRRTPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
          ss->getPlanner()->as<og::LazyRRT>()->setRange(_Range);
          //set the longest valid segment (i.e. segment without need to be collision-checked) as a fraction of
          //the maximum extend of the space (set to e.g. the diagonal of a R^n cubic space)
          space->setLongestValidSegmentFraction(0.01);
          //correct if necessary to allow that the (_Range/_validSegmentCount) be the longest valid segment,
          //so as to allow _validSegmentCount collisionchecks within a range step.
          if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
              space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
          }
          space->setup();//needed to update the setting of the longest valid segment
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<og::LazyRRT>()->setGoalBias(_GoalBias);
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
