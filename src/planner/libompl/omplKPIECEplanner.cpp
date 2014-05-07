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


#if defined(KAUTHAM_USE_OMPL)
#include <libproblem/workspace.h>
#include <libsampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "omplKPIECEplanner.h"
#include "omplValidityChecker.h"



namespace Kautham {
  namespace omplplanner{

    //! Constructor
    omplKPIECEPlanner::omplKPIECEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
    {
        _guiName = "ompl KPIECE Planner";
        _idName = "omplKPIECE";


        //alloc valid state sampler
        si->setValidStateSamplerAllocator(boost::bind(&omplplanner::allocValidStateSampler, _1, (Planner*)this));
        //alloc state sampler
        space->setStateSamplerAllocator(boost::bind(&omplplanner::allocStateSampler, _1, (Planner*)this));

        //create planner
        ob::PlannerPtr planner(new og::KPIECE1(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<og::KPIECE1>())->getGoalBias();
        _minValidPathFraction=(planner->as<og::KPIECE1>())->getMinValidPathFraction();
        _failedExpansionScoreFactor=(planner->as<og::KPIECE1>())->getFailedExpansionCellScoreFactor();

        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        addParameter("minValidPathFraction", _minValidPathFraction);
        addParameter("failedExpansionScoreFactor",_failedExpansionScoreFactor);
        planner->as<og::KPIECE1>()->setRange(_Range);
        planner->as<og::KPIECE1>()->setGoalBias(_GoalBias);
        planner->as<og::KPIECE1>()->setMinValidPathFraction(_minValidPathFraction);
        planner->as<og::KPIECE1>()->setFailedExpansionCellScoreFactor(_failedExpansionScoreFactor);
        planner->as<og::KPIECE1>()->setProjectionEvaluator(space->getDefaultProjection());

        //set the planner
        ss->setPlanner(planner);
    }

    //! void destructor
    omplKPIECEPlanner::~omplKPIECEPlanner(){

    }

    //! setParameters sets the parameters of the planner
    bool omplKPIECEPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
          ss->getPlanner()->as<og::KPIECE1>()->setRange(_Range);
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<og::KPIECE1>()->setGoalBias(_GoalBias);
        }
        else
          return false;

        it = _parameters.find("minValidPathFraction");
        if(it != _parameters.end()){
            _minValidPathFraction = it->second;
            ss->getPlanner()->as<og::KPIECE1>()->setMinValidPathFraction (_minValidPathFraction);
        }
        else
          return false;

        it = _parameters.find("failedExpansionScoreFactor");
        if(it != _parameters.end()){
            _failedExpansionScoreFactor = it->second;
            ss->getPlanner()->as<og::KPIECE1>()->setFailedExpansionCellScoreFactor(_failedExpansionScoreFactor);
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
