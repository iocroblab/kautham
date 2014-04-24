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

 

#if defined(KAUTHAM_USE_OMPL)
#include <libproblem/workspace.h>
#include <libsampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "omplpRRTplanner.h"
#include "omplValidityChecker.h"



namespace Kautham {
  namespace omplplanner{

	//! Constructor
    omplpRRTPlanner::omplpRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl pRRT Planner";
        _idName = "omplpRRT";


        //alloc valid state sampler
        si->setValidStateSamplerAllocator(boost::bind(&omplplanner::allocValidStateSampler, _1, (Planner*)this));
        //alloc state sampler
        space->setStateSamplerAllocator(boost::bind(&omplplanner::allocStateSampler, _1, (Planner*)this));

        //create planner
        ob::PlannerPtr planner(new og::pRRT(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<og::pRRT>())->getGoalBias();
        _ThreadCount=(planner->as<og::pRRT>())->getThreadCount();
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        addParameter("Thread Count", _ThreadCount);
        planner->as<og::pRRT>()->setRange(_Range);
        planner->as<og::pRRT>()->setGoalBias(_GoalBias);
        planner->as<og::pRRT>()->setThreadCount(_ThreadCount);

        //set the planner
        ss->setPlanner(planner);
    }

	//! void destructor
    omplpRRTPlanner::~omplpRRTPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplpRRTPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
          ss->getPlanner()->as<og::pRRT>()->setRange(_Range);
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<og::pRRT>()->setGoalBias(_GoalBias);
        }
        else
          return false;

        it = _parameters.find("Thread Count");
        if(it != _parameters.end()){
            _ThreadCount = (unsigned int)it->second;
            ss->getPlanner()->as<og::pRRT>()->setThreadCount(_ThreadCount);
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
