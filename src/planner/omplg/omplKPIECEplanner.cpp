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

#include <kautham/planner/omplg/omplKPIECEplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/kpiece/Discretization.h>
#include <limits>
#include <cassert>

namespace Kautham {
  namespace omplplanner{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //! This is a class derived form the class ompl::geometric::KPIECE. Its purpose is to reimplement the solve function to slightly change its behavior, by:
    //!      1) Allowing to grow several steps in the same direction (cpontrolled by a new parameter called numMaxSteps)
    //!      2) Limiting the growing towards the goal to maxNumSteps*maxDistance_
    //!      3) Not allowing to grow in directions such that the dot product between consecutive motions be negative
    //!         (it is implemented using distances)
    class myKPIECE1:public og::KPIECE1
    {
        private:
            int maxNumSteps;

        public:
        myKPIECE1( ob::SpaceInformationPtr &si) : KPIECE1(si)
        {

        }
        void setMaxNumSteps(unsigned int t) {maxNumSteps = t;};

        ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc)
        {
            checkValidity();
            ob::Goal *goal = pdef_->getGoal().get();
            ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion *>(goal);

            og::Discretization<Motion>::Coord xcoord;

            while (const ob::State *st = pis_.nextStart())
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                projectionEvaluator_->computeCoordinates(motion->state, xcoord);
                disc_.addMotion(motion, xcoord, 1.0);
            }

            if (disc_.getMotionCount() == 0)
            {
                OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                return ob::PlannerStatus::INVALID_START;
            }

            if (!sampler_)
                sampler_ = si_->allocStateSampler();

            OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                        disc_.getMotionCount());

            Motion *solution = nullptr;
            Motion *approxsol = nullptr;
            double approxdif = std::numeric_limits<double>::infinity();
            ob::State *xstate = si_->allocState();
            ob::State *firststate = si_->allocState();

            while (ptc == false)
            {
                disc_.countIteration();

                /* Decide on a state to expand from */
                Motion *existing = nullptr;
                og::Discretization<Motion>::Cell *ecell = nullptr;
                disc_.selectMotion(existing, ecell);
                assert(existing);

                /* Specification of number of steps moves and sense */
                int maxsteps = 1+rng_.uniform01()*maxNumSteps;
                int sense = 1; //to change the sens of motion if needed.

                /* sample random state (with goal biasing) */
                if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                {
                    /* sample goal */
                    goal_s->sampleGoal(xstate);
                    sense = 1;
                    /* find state to add. At most at a distance maxsteps*maxDistance_ from the state being expanded */
                    double d = si_->distance(existing->state, xstate);
                    if (d > maxsteps*maxDistance_)
                    {
                        /* xstate is the new state towards to grow in maxsteps steps */
                        si_->getStateSpace()->interpolate(existing->state, xstate, maxsteps*maxDistance_ / d, xstate);
                        /* firststate is the first state towards xstate */
                        si_->getStateSpace()->interpolate(existing->state, xstate, 1.0/maxsteps, firststate);
                        if(existing->parent!=nullptr)
                        {
                            /* if firststate is closer to the parent than to existing->state it means we must move in the reverse direction */
                            double dparent = si_->distance(existing->parent->state, firststate);
                            if(dparent>maxDistance_) sense = 1;
                            else sense = -1;
                        }
                    }
                }
                else
                {
                    /* sample a state close to exisiting->state */
                    sampler_->sampleUniformNear(xstate, existing->state, maxsteps*maxDistance_);
                    si_->getStateSpace()->interpolate(existing->state, xstate, 1.0/maxsteps, firststate);
                    if(existing->parent!=nullptr)
                    {
                        double dparent = si_->distance(existing->parent->state, firststate);
                        if(dparent>maxDistance_) sense = 1;
                        else sense = -1;
                    }
                }

            //Start Old code from OMPL::KPIECE1:
                //std::pair<ob::State *, double> fail(xstate, 0.0);
                //bool keep = si_->checkMotion(existing->state, xstate, fail);
                //if (!keep && fail.second > minValidPathFraction_)
                //    keep = true;
                //if (keep)

                //if (si_->checkMotion(existing->state, xstate))
                //{
                //    /* create a motion */
                //    auto *motion = new Motion(si_);
                //    si_->copyState(motion->state, xstate);
                //    motion->parent = existing;

                //    double dist = 0.0;
                //    bool solv = goal->isSatisfied(motion->state, &dist);
                //    projectionEvaluator_->computeCoordinates(motion->state, xcoord);
                //    disc_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no
                //                                                 // call to updateCell() is needed

                //    if (solv)
                //    {
                //        approxdif = dist;
                //        solution = motion;
                //        break;
                //    }
                //    if (dist < approxdif)
                //    {
                //        approxdif = dist;
                //        approxsol = motion;
                //    }

                //}

              //End Old code from OMPL::KPIECE1:

                Motion *m1=existing;
                ob::State *s1 = existing->state;
                ob::State *s2 = si_->allocState();

                //Loop to grow towards xstate in maxsteps
                for(int i=1; i<= maxsteps; i++)
                {
                    //interpolate
                    si_->getStateSpace()->interpolate(existing->state, xstate, (double)sense*i/maxsteps, s2);

                    if (si_->checkMotion(s1, s2))
                    {
                        /* create the motion */
                        auto *motion = new Motion(si_);
                        si_->copyState(motion->state, s2);
                        motion->parent = m1;

                        double dist = 0.0;
                        bool solv = goal->isSatisfied(motion->state, &dist);
                        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
                        disc_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no
                                                                 // call to updateCell() is needed

                        if (solv)
                        {
                            approxdif = dist;
                            solution = motion;
                            break;
                        }
                        if (dist < approxdif)
                        {
                            approxdif = dist;
                            approxsol = motion;
                        }

                        //prepare next step
                        s1=s2;
                        m1=motion;
                    }
                    else
                    {
                        ecell->data->score *= failedExpansionScoreFactor_;
                        break;
                    }
                }
                si_->freeState(s2);

                disc_.updateCell(ecell);
            }

            bool solved = false;
            bool approximate = false;
            if (solution == nullptr)
            {
                solution = approxsol;
                approximate = true;
            }

            if (solution != nullptr)
            {
                lastGoalMotion_ = solution;

                /* construct the solution path */
                std::vector<Motion *> mpath;
                while (solution != nullptr)
                {
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                /* set the solution path */
                auto path(std::make_shared< og::PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                    path->append(mpath[i]->state);
                pdef_->addSolutionPath(path, approximate, approxdif, getName());
                solved = true;
            }

            si_->freeState(xstate);

            OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                        disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(),
                        disc_.getGrid().countExternal());

            return ob::PlannerStatus(solved, approximate);
      }
    };




    //! Constructor
    omplKPIECEPlanner::omplKPIECEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
    {
        _guiName = "ompl KPIECE Planner";
        _idName = "omplKPIECE";

        //create planner
        ob::PlannerPtr planner(new myKPIECE1(si));
        //set planner parameters: range and goalbias
        _Range=0.05;
        _GoalBias=(planner->as<myKPIECE1>())->getGoalBias();
        _maxNumSteps = 4;
        //_minValidPathFraction=(planner->as<og::KPIECE1>())->getMinValidPathFraction();
        _failedExpansionScoreFactor=(planner->as<myKPIECE1>())->getFailedExpansionCellScoreFactor();

        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        //addParameter("minValidPathFraction", _minValidPathFraction);
        addParameter("maxNumSteps", _maxNumSteps);
        addParameter("failedExpansionScoreFactor",_failedExpansionScoreFactor);
        planner->as<myKPIECE1>()->setRange(_Range);
        planner->as<myKPIECE1>()->setGoalBias(_GoalBias);
        //planner->as<og::KPIECE1>()->setMinValidPathFraction(_minValidPathFraction);
        planner->as<myKPIECE1>()->setFailedExpansionCellScoreFactor(_failedExpansionScoreFactor);
        planner->as<myKPIECE1>()->setProjectionEvaluator(space->getDefaultProjection());
        addParameter("Cell Size",planner->as<myKPIECE1>()->getProjectionEvaluator()->getCellSizes().at(0));

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
          ss->getPlanner()->as<myKPIECE1>()->setRange(_Range);
         }
        else
          return false;

        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<myKPIECE1>()->setGoalBias(_GoalBias);
        }
        else
          return false;

//        it = _parameters.find("minValidPathFraction");
//        if(it != _parameters.end()){
//            _minValidPathFraction = it->second;
//            ss->getPlanner()->as<og::KPIECE1>()->setMinValidPathFraction (_minValidPathFraction);
//        }
//        else
//          return false;

        it = _parameters.find("maxNumSteps");
        if(it != _parameters.end()){
            ss->getPlanner()->as<myKPIECE1>()->setMaxNumSteps(it->second);
        }
        else
          return false;

        it = _parameters.find("failedExpansionScoreFactor");
        if(it != _parameters.end()){
            _failedExpansionScoreFactor = it->second;
            ss->getPlanner()->as<myKPIECE1>()->setFailedExpansionCellScoreFactor(_failedExpansionScoreFactor);
        }
        else
          return false;


        it = _parameters.find("Cell Size");
        if (it == _parameters.end()) return false;
        if (it->second < 0.) {
            it->second = ss->getPlanner()->as<myKPIECE1>()->getProjectionEvaluator()->getCellSizes().at(0);
        } else {
            std::vector<double> sizes(ss->getPlanner()->as<myKPIECE1>()->getProjectionEvaluator()->getDimension(),it->second);
            ss->getPlanner()->as<myKPIECE1>()->getProjectionEvaluator()->setCellSizes(sizes);
        }

      }catch(...){
        return false;
      }
      return true;
    }
  }
}


#endif // KAUTHAM_USE_OMPL
