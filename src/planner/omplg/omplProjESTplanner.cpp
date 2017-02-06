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
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplProjESTplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>



namespace Kautham {
  namespace omplplanner{

    //! Constructor
    omplProjESTPlanner::omplProjESTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
    {
        _guiName = "ompl ProjEST Planner";
        _idName = "omplProjEST";

        //create planner
#if OMPL_VERSION_VALUE >= 1002000
        ob::PlannerPtr planner(new og::ProjEST(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<og::ProjEST>())->getGoalBias();
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        planner->as<og::ProjEST>()->setRange(_Range);
        planner->as<og::ProjEST>()->setGoalBias(_GoalBias);
        planner->as<og::ProjEST>()->setProjectionEvaluator(space->getDefaultProjection());
#else
        ob::PlannerPtr planner(new og::EST(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<og::EST>())->getGoalBias();
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        planner->as<og::EST>()->setRange(_Range);
        planner->as<og::EST>()->setGoalBias(_GoalBias);
        planner->as<og::EST>()->setProjectionEvaluator(space->getDefaultProjection());
#endif
        //set the planner
        ss->setPlanner(planner);
    }

    //! void destructor
    omplProjESTPlanner::~omplProjESTPlanner(){

    }

    //! setParameters sets the parameters of the planner
    bool omplProjESTPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
#if OMPL_VERSION_VALUE >= 1002000
          ss->getPlanner()->as<og::ProjEST>()->setRange(_Range);
#else
          ss->getPlanner()->as<og::EST>()->setRange(_Range);
#endif
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
#if OMPL_VERSION_VALUE >= 1002000
            ss->getPlanner()->as<og::ProjEST>()->setGoalBias(_GoalBias);
#else
            ss->getPlanner()->as<og::EST>()->setGoalBias(_GoalBias);
#endif
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
