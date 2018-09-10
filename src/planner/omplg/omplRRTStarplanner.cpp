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
#include <iostream>

#include <kautham/planner/omplg/omplRRTStarplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>


#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <kautham/planner/omplg/omplPCAalignmentOptimizationObjective.h>
#include <kautham/planner/omplg/omplMyOptimizationObjective.h>

//for myRRTstar
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <boost/math/constants/constants.hpp>
//#include <ompl/geometric/planners/PlannerIncludes.h>
//#include <ompl/datastructures/NearestNeighbors.h>
//#include <limits>
//#include <vector>
//#include <utility>

namespace Kautham {
    namespace omplplanner{
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        class myRRTstar:public og::RRTstar
        {
        private:
            bool _optimize; //!< flag that allows to disable optimization when set to false. Defaults to true.
            double _factor_k_rrg;//!< Value that modifies the number K of nearest neighbors (multiplication factor)
            double _nodeRejection; //!< Flag to set the filter that eliminates no promising samples (akgun and Stilman, 2011)
            double _pathBias; //!< Value to bias the sampling around the solution path, once it has been found (akgun and Stilman, 2011)
            double _pathSamplingRangeFactor; //!< Factor that multiplied by the RRT range gives the radius for sampling near the path
            ob::Cost bestCostP_;
            ob::Cost bestCostI_;
            ob::Cost bestCostD_;


        public:

            myRRTstar(const ob::SpaceInformationPtr &si) : RRTstar(si),bestCostP_(std::numeric_limits<double>::quiet_NaN()),
                bestCostI_(std::numeric_limits<double>::quiet_NaN()),bestCostD_(std::numeric_limits<double>::quiet_NaN()) {
                setName("myRRTstar");
                _optimize = true;
                _factor_k_rrg = 1.0;
                _pathBias = 0.1;
                _nodeRejection = 1;
                _pathSamplingRangeFactor = 2.0;

                addPlannerProgressProperty("best costP REAL",std::bind(&myRRTstar::getBestCostP,this));
                addPlannerProgressProperty("best costI REAL",std::bind(&myRRTstar::getBestCostI,this));
                addPlannerProgressProperty("best costD REAL",std::bind(&myRRTstar::getBestCostD,this));
            }

            bool getOptimize(){ return _optimize;} //!< Returns the _optimize flag
            void setOptimize(bool s){_optimize=s;} //!< Sets the _optimize flag
            bool getNeighFactor(){ return _factor_k_rrg;} //!< Returns _factor_k_rrg that modifies the number K of nearest neighbors
            void setNeighFactor(double f){_factor_k_rrg=f;} //!< Sets  _factor_k_rrg to modify the number K of nearest neighbors
            double getNodeRejection(){return _nodeRejection;} //!< Returns NodeRejection probability
            void setNodeRejection(double v){if (v>1.0) _nodeRejection=1.0; else if (v<0.0) _nodeRejection=0.0; else  _nodeRejection=v;} //!< Sets NodeRejection flag
            double getPathBias(){return _pathBias;} //!< Returns the _PAthBias
            void setPathBias(double f){if (f>1.0) _pathBias=1.0; else if (f<0.0) _pathBias=0.0; else  _pathBias=f;} //!< Sets _PathBias to sample near the solution path
            double getPathSamplingRangeFactor(){return _pathSamplingRangeFactor;} //!< Returns the factor that multiplied by the RRT range gives the radius for sampling near the path
            void setPathSamplingRangeFactor(double f){_pathSamplingRangeFactor = max(f,1.);} //!< Sets the factor that multiplied by the RRT range gives the radius for sampling near the path

            void clear() {
                og::RRTstar::clear();
                bestCostP_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
                bestCostI_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
                bestCostD_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
            }

            std::string getBestCostP() const {
                return boost::lexical_cast<std::string>(bestCostP_);
            }

            std::string getBestCostI() const {
                return boost::lexical_cast<std::string>(bestCostI_);
            }

            std::string getBestCostD() const {
                return boost::lexical_cast<std::string>(bestCostD_);
            }

            /** \brief Compute distance between motions (actually distance between contained states)
       * for finding nearest neighbors. If states are already connected their distance is set to infinity*/
            double distanceFunction(const Motion* a, const Motion* b) const {
                return si_->distance(a->state, b->state);
            }


            void updateCostPID(Motion *solution) {
                myICOptimizationObjective *optiPID = dynamic_cast<myICOptimizationObjective*>(opt_.get());
                if (optiPID) {
                    std::vector<Motion*> mpath;
                    Motion *motion = solution;
                    while (motion) {
                        mpath.push_back(motion);
                        motion = motion->parent;
                    }
                    og::PathGeometric *geoPath = new og::PathGeometric(si_);
                    for (int i = mpath.size() - 1; i >= 0; --i) {
                        geoPath->append(mpath[i]->state);
                    }

                    double KP = optiPID->getKP();
                    double KI = optiPID->getKI();
                    double KD = optiPID->getKD();

                    optiPID->setKP(1.);
                    optiPID->setKI(0.);
                    optiPID->setKD(0.);
                    bestCostP_ = geoPath->cost(opt_);

                    optiPID->setKP(0.);
                    optiPID->setKI(1.);
                    //optiPID->setKD(0.);
                    bestCostI_ = geoPath->cost(opt_);

                    //optiPID->setKP(0.);
                    optiPID->setKI(0.);
                    optiPID->setKD(1.);
                    bestCostD_ = geoPath->cost(opt_);

                    optiPID->setKP(KP);
                    optiPID->setKI(KI);
                    optiPID->setKD(KD);

                    delete geoPath;
                }
            }

            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) {
                checkValidity();
                ob::Goal                  *goal   = pdef_->getGoal().get();
                ob::GoalSampleableRegion  *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

                bool symDist = si_->getStateSpace()->hasSymmetricDistance();
                bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
                bool symCost = opt_->isSymmetric();

                while (const ob::State *st = pis_.nextStart())
                {
                    Motion *motion = new Motion(si_);
                    si_->copyState(motion->state, st);
                    motion->cost = opt_->identityCost();
                    nn_->add(motion);
                }

                if (nn_->size() == 0)
                {
                    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                    return ob::PlannerStatus::INVALID_START;
                }


                if (!sampler_)
                    sampler_ = si_->allocStateSampler();

                OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
                Motion *solution       = bestGoalMotion_;
#else
                Motion *solution       = lastGoalMotion_;
#endif

                // \TODO Make this variable unnecessary, or at least have it
                // persist across solve runs
                ob::Cost bestCost    = opt_->infiniteCost();
                if (solution){
                    bestCost_ = solution->cost;
                    OMPL_INFORM("%s: Starting with existing solution of cost %.5f",
                                getName().c_str(), bestCost_.value());
                    updateCostPID(solution);
                } else {
                    bestCost_ = opt_->infiniteCost();
                    bestCostP_ = opt_->infiniteCost();
                    bestCostI_ = opt_->infiniteCost();
                    bestCostD_ = opt_->infiniteCost();
                }

                Motion *approximation = NULL;
                double approximatedist = std::numeric_limits<double>::infinity();
                bool sufficientlyShort = false;

                Motion *rmotion = new Motion(si_);
                ob::State *rstate = rmotion->state;
                ob::State *xstate = si_->allocState();

                // e+e/d.  K-nearest RRT*
                double k_rrg;
                if (_optimize) {
                    k_rrg = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());
                    k_rrg = _factor_k_rrg*k_rrg;
                } else {
                    k_rrg = 0.0; //When set to zero RRTStar behavies as a standard RRT
                }

                std::vector<Motion*>       nbh;

                std::vector<ob::Cost>    costs;
                std::vector<ob::Cost>    incCosts;
                std::vector<std::size_t>   sortedCostIndices;

                std::vector<int>           valid;
                unsigned int               rewireTest = 0;
                unsigned int               statesGenerated = 0;

                OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size()+1))));


                // our functor for sorting nearest neighbors
                CostIndexCompare compareFn(costs, *opt_);

                while (ptc == false)
                {
                    iterations_++;
                    // sample random state (with goal biasing)
                    // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.

                    /* JAN
              int kk1=goalMotions_.size();
              int kk2=goal_s->maxSampleCount();
              int kk3=nn_->size();

              if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                  goal_s->sampleGoal(rstate);
              else
                  sampler_->sampleUniform(rstate);
              */

                    //JAN: comment the next OMPL lines
                    //if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
                    //    goal_s->sampleGoal(rstate);
                    //else
                    //    sampler_->sampleUniform(rstate);

                    //JAN added these lines to subsitute the above ones
                    //Motion *nmotion;//the nearest neighbor node

                    int trials;
                    int maxtrials;
                    bool found;
                    //if goal not yet reached
                    if (solution == 0)
                    {
                        //sample goal with a certain probability goalBias_
                        if ( rng_.uniform01() < goalBias_ && goal_s->canSample() ) {
                            goal_s->sampleGoal(rstate);
                        }
                        //or randomly otehrwise
                        else {
                            sampler_->sampleUniform(rstate);
                        }

                        //nmotion = nn_->nearest(rmotion);
                    }
                    //else if goal already reached
                    else
                    {
                        std::vector<Motion*> mpath;
                        //load the path
                        Motion *currentsolution = solution;
                        while (currentsolution != 0)
                        {
                            mpath.push_back(currentsolution);
                            currentsolution = currentsolution->parent;
                        }

                        //get sample
                        //sample around path with a certain probability pathBias_
                        if (rng_.uniform01() < getPathBias())
                        {
                            //sample a nodeof the path
                            double pmin = 0.5;
                            double pmax = getPathSamplingRangeFactor(); //2.0;
                            double threshold = (pmin+(pmax-pmin)*rng_.uniform01())*maxDistance_; //10*epsilon
                            unsigned node = (unsigned)(rng_.uniform01() * mpath.size());
                            if (node==mpath.size()) node = mpath.size()-1; //just in case rng_.uniform01 returned 1


                            //cout<<"node="<<node<<" trheshold="<<threshold<<endl;

                            if (node==0) //the goal node (the path is stored from goal to start in mpath)
                            {
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(mpath[0]->state, threshold);
                                sampler_->sampleUniform(rstate);
                                //restore
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(NULL, threshold);
                            }
                            else if (node==mpath.size()-1)//the start node
                            {
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(mpath[mpath.size()-1]->state, threshold);
                                sampler_->sampleUniform(rstate);
                                //restore
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(NULL, threshold);
                            }
                            else //a node in between start and goal nodes
                            {
                                ob::State *astate= si_->allocState();
                                const ob::State *priornode = mpath[node-1]->state;
                                const ob::State *nextnode = mpath[node+1]->state;
                                ob::StateSpacePtr space = getSpaceInformation()->getStateSpace();


                                space->interpolate(priornode, nextnode, 0.5, astate);
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(astate, threshold);
                                sampler_->sampleUniform(rstate);
                                //restore
                                ((KauthamStateSampler*)sampler_.get())->setCenterSample(NULL, threshold);

                                //space->printState(priornode,std::cout);
                                //space->printState(nextnode,std::cout);
                                //space->printState(astate,std::cout);
                                //space->printState(rstate,std::cout);

                                //Motion *themotion = new Motion(si_);
                                //themotion->state = mpath[node]->state;
                                //nmotion = nn_->nearest(themotion);
                            }
                        }
                        //or sample randomly otehrwise
                        else {
                            if (rng_.uniform01() < getNodeRejection())
                            {
                                trials = 0;
                                maxtrials = 100;
                                found = false;
                                do //loop until non reject or maxtrials
                                {
                                    sampler_->sampleUniform(rstate);
                                    //node rejection test
                                    //Find nearest state in tree
                                    Motion *nearestmotion = nn_->nearest(rmotion);
                                    //c1 = cost from start to nearestmotion (path in tree)
                                    ob::Cost c1 = nearestmotion->cost;

                                    //c2 = cost of added edge from nearestmotion to rmotion
                                    ob::Cost c2;
                                    ob::Cost c3;
                                    if (opt_->getDescription() == "PMD alignment"){
                                        ob::State *s0;
                                        if (!nearestmotion->parent) s0 = NULL;
                                        else s0 = nearestmotion->parent->state;

                                        double d = si_->distance(nearestmotion->state, rmotion->state);
                                        si_->getStateSpace()->interpolate(nearestmotion->state, rmotion->state, maxDistance_ / d, xstate);
                                        c2 = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nearestmotion->state, xstate);
                                        //c2 = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nearestmotion->state, rmotion->state);

                                        //c3 = heuristic cost from rmotion to goal (the edge may not exist)
                                        c3=opt_->motionCost(xstate, mpath[0]->state); //from rmotion to goal (straight)

                                    }
                                    else
                                    {
                                        c2=opt_->motionCost(nearestmotion->state, rmotion->state); //from rmotion to goal (straight)

                                        //c3 = heuristic cost from rmotion to goal (the edge may not exist)
                                        c3=opt_->motionCost(rmotion->state, mpath[0]->state); //from rmotion to goal (straight)
                                    }


                                    //combine costs
                                    ob::Cost newcost = opt_->combineCosts(c1, opt_->combineCosts(c2,c3));

                                    if (opt_->isCostBetterThan(solution->cost, newcost)) //rejected: cannot imprve the current solution
                                    {
                                        trials++;
                                    }
                                    else{//not rejected
                                        found=true;
                                    }
                                }while(trials<maxtrials && found==false);
                            }
                            else{//if node rejection not activated then accept always
                                sampler_->sampleUniform(rstate);
                            }

                        }
                    }

                    //JAN
                    //sampler_->sampleUniform(rstate);

                    // find closest state in the tree
                    Motion *nmotion = nn_->nearest(rmotion);

                    //JAN destination state
                    //recompute the state towards which the tree is growing
                    //in this way a leave may grow towards the goal even if it is not the leave closest to the goal
                    //if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                    //    goal_s->sampleGoal(rstate);



                    ob::State *dstate = rstate;

                    // find state to add to the tree
                    double d = si_->distance(nmotion->state, rstate);
                    if (d > maxDistance_)
                    {
                        si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
                        dstate = xstate;
                    }

                    // Check if the motion between the nearest state and the state to add is valid
                    if (si_->checkMotion(nmotion->state, dstate))
                    {
                        // create a motion
                        Motion *motion = new Motion(si_);
                        si_->copyState(motion->state, dstate);
                        motion->parent = nmotion;

                        // This sounds crazy but for asymmetric distance functions this is necessary
                        // For this case, it has to be FROM every other point TO our new point
                        // NOTE THE ORDER OF THE std::bind PARAMETERS
                        if (!symDist)
                            nn_->setDistanceFunction(std::bind(&myRRTstar::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));

                        // Find nearby neighbors of the new motion - k-nearest RRT*
                        unsigned int k = std::ceil(k_rrg * log((double)(nn_->size()+1)));
                        nn_->nearestK(motion, k, nbh);

                        //JAN: prune those that are too far away (more than 10 times the range.
                        //for(int i=nbh.size()-1;i>=0;i--)
                        //{
                        //    if (si_->distance(motion->state, nbh[i]->state) > 10*maxDistance_){
                        //        nbh.pop_back();//delete last element
                        //    }
                        //}



                        rewireTest += nbh.size();
                        statesGenerated++;

                        // cache for distance computations
                        //
                        // Our cost caches only increase in size, so they're only
                        // resized if they can't fit the current neighborhood
                        if (costs.size() < nbh.size())
                        {
                            costs.resize(nbh.size());
                            incCosts.resize(nbh.size());
                            sortedCostIndices.resize(nbh.size());
                        }

                        // cache for motion validity (only useful in a symmetric space)
                        //
                        // Our validity caches only increase in size, so they're
                        // only resized if they can't fit the current neighborhood
                        if (symDist && symInterp)
                        {
                            if (valid.size() < nbh.size())
                                valid.resize(nbh.size());
                            std::fill(valid.begin(), valid.begin()+nbh.size(), 0);
                        }


                        // Finding the nearest neighbor to connect to
                        // By default, neighborhood states are sorted by cost, and collision checking
                        // is performed in increasing order of cost
                        if (delayCC_)
                        {
                            // calculate all costs and distances
                            for (std::size_t i = 0 ; i < nbh.size(); ++i)
                            {

                                if (opt_->getDescription()=="PMD alignment"){
                                    ob::State *s0;
                                    if (nbh[i]->parent==NULL) s0=NULL;
                                    else s0=nbh[i]->parent->state;
                                    incCosts[i] = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nbh[i]->state, motion->state);
                                }
                                else
                                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                                costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                            }

                            // sort the nodes
                            //
                            // we're using index-value pairs so that we can get at
                            // original, unsorted indices
                            for (std::size_t i = 0; i < nbh.size(); ++i)
                                sortedCostIndices[i] = i;
                            std::sort(sortedCostIndices.begin(), sortedCostIndices.begin()+nbh.size(),
                                      compareFn);

                            // collision check until a valid motion is found
                            for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                                 i != sortedCostIndices.begin()+nbh.size();
                                 ++i)
                            {
                                if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
                                {
                                    motion->incCost = incCosts[*i];
                                    motion->cost = costs[*i];
                                    motion->parent = nbh[*i];
                                    if (symDist && symInterp)
                                        valid[*i] = 1;
                                    break;
                                }
                                else if (symDist && symInterp)
                                    valid[*i] = -1;
                            }
                        }
                        else // if not delayCC
                        {

                            if (opt_->getDescription()=="PMD alignment"){
                                ob::State *s0;
                                if (nmotion->parent==NULL) s0=NULL;
                                else s0=nmotion->parent->state;
                                motion->incCost = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nmotion->state, motion->state);
                            }
                            else
                                motion->incCost = opt_->motionCost(nmotion->state, motion->state);


                            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                            // find which one we connect the new state to
                            for (std::size_t i = 0 ; i < nbh.size(); ++i)
                            {
                                if (nbh[i] != nmotion)
                                {

                                    if (opt_->getDescription()=="PMD alignment"){
                                        ob::State *s0;
                                        if (nbh[i]->parent==NULL) s0=NULL;
                                        else s0=nbh[i]->parent->state;
                                        incCosts[i] = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nbh[i]->state, motion->state);
                                    }
                                    else
                                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);

                                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                                    if (opt_->isCostBetterThan(costs[i], motion->cost))
                                    {
                                        if (si_->checkMotion(nbh[i]->state, motion->state))
                                        {
                                            motion->incCost = incCosts[i];
                                            motion->cost = costs[i];
                                            motion->parent = nbh[i];
                                            if (symDist && symInterp)
                                                valid[i] = 1;
                                        }
                                        else if (symDist && symInterp)
                                            valid[i] = -1;
                                    }
                                }
                                else
                                {
                                    incCosts[i] = motion->incCost;
                                    costs[i] = motion->cost;
                                    if (symDist && symInterp)
                                        valid[i] = 1;
                                }
                            }
                        }

                        // add motion to the tree
                        nn_->add(motion);
                        motion->parent->children.push_back(motion);

                        bool checkForSolution = false;
                        // rewire tree if needed
                        //
                        // This sounds crazy but for asymmetric distance functions this is necessary
                        // For this case, it has to be FROM our new point TO each other point
                        // NOTE THE ORDER OF THE std::bind PARAMETERS
                        if (!symDist)
                        {
                            nn_->setDistanceFunction(std::bind(&myRRTstar::distanceFunction, this, std::placeholders::_2, std::placeholders::_1));
                            nn_->nearestK(motion, k, nbh);
                            //JAN: prune those that are too far away (more than 10 times the range.
                            //for(int i=nbh.size()-1;i>=0;i--)
                            //{
                            //    if (si_->distance(motion->state, nbh[i]->state) > 10*maxDistance_){
                            //        nbh.pop_back();//delete last element
                            //    }
                            //}

                            rewireTest += nbh.size();
                        }

                        for (std::size_t i = 0; i < nbh.size(); ++i)
                        {
                            if (nbh[i] != motion->parent)
                            {
                                ob::Cost nbhIncCost;
                                if (symDist && symCost)
                                    nbhIncCost = incCosts[i];
                                else{
                                    if (opt_->getDescription()=="PMD alignment"){
                                        ob::State *s0;
                                        if (motion->parent==NULL) s0=NULL;
                                        else s0=motion->parent->state;
                                        nbhIncCost = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,motion->state, nbh[i]->state);
                                    }
                                    else
                                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                                }

                                ob::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                                if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                                {
                                    bool motionValid;
                                    if (symDist && symInterp)
                                    {
                                        if (valid[i] == 0)
                                        {
                                            motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                                        }
                                        else
                                            motionValid = (valid[i] == 1);
                                    }
                                    else
                                    {
                                        motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                                    }
                                    if (motionValid)
                                    {
                                        // Remove this node from its parent list
                                        removeFromParent (nbh[i]);

                                        // Add this node to the new parent
                                        nbh[i]->parent = motion;
                                        nbh[i]->incCost = nbhIncCost;
                                        nbh[i]->cost = nbhNewCost;
                                        nbh[i]->parent->children.push_back(nbh[i]);

                                        // Update the costs of the node's children
                                        updateChildCosts(nbh[i]);

                                        checkForSolution = true;
                                    }
                                }
                            }
                        }

                        // Add the new motion to the goalMotion_ list, if it satisfies the goal
                        double distanceFromGoal;
                        if (goal->isSatisfied(motion->state, &distanceFromGoal))
                        {
                            goalMotions_.push_back(motion);
                            //JAN
                            //cout<<"goalMotions_.size() = "<<goalMotions_.size()<<endl;
                            //cout<<"added Goal Motion with cost = "<<motion->cost.v<<endl;
                            //cout<<"goalMotions_[0]->cost = "<<goalMotions_[0]->cost.v<<endl;

                            //JAN
                            //nn_->remove(motion);



                            checkForSolution = true;
                        }

                        // Checking for solution or iterative improvement
                        if (checkForSolution)
                        {
                            for (size_t i = 0; i < goalMotions_.size(); ++i)
                            {
                                if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                                {
                                    bestCost = goalMotions_[i]->cost;
                                    bestCost_ = bestCost;
                                    updateCostPID(goalMotions_[i]);
                                }

                                sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
                                if (sufficientlyShort)
                                {
                                    solution = goalMotions_[i];
                                    break;
                                }
                                else if (!solution ||
                                         opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
                                    solution = goalMotions_[i];
                            }
                            //JAN
                            //if (solution) cout<<"Solution with cost = "<<solution->cost.v<<endl;
                        }

                        // Checking for approximate solution (closest state found to the goal)
                        if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
                        {
                            approximation = motion;
                            approximatedist = distanceFromGoal;
                        }
                    }

                    // terminate if a sufficient solution is found
                    if (solution && sufficientlyShort)
                        break;
                }

                bool approximate = (solution == 0);
                bool addedSolution = false;
                if (approximate)
                    solution = approximation;
                else
#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
                    bestGoalMotion_ = solution;
#else
                    lastGoalMotion_ = solution;
#endif

                if (solution != 0)
                {

                    cout << "addedSolution with cost = " << solution->cost.value() << endl;
                    // construct the solution path
                    std::vector<Motion*> mpath;
                    while (solution != 0)
                    {
                        mpath.push_back(solution);
                        solution = solution->parent;
                    }

                    // set the solution path
                    og::PathGeometric *geoPath = new og::PathGeometric(si_);
                    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
                        geoPath->append(mpath[i]->state);

                    ob::PathPtr path(geoPath);
                    // Add the solution path.
                    ob::PlannerSolution psol(path);
                    psol.setPlannerName(getName());
                    if (approximate)
                        psol.setApproximate(approximatedist);
                    // Does the solution satisfy the optimization objective?
                    psol.setOptimized(opt_, bestCost, sufficientlyShort);
                    pdef_->addSolutionPath(psol);


                    //JAN
                    //cout<<"path with " << geoPath->getStateCount()<<" states"<<endl;
                    //cout<<"addedSolution with cost = "<<mpath[0]->cost.v<<endl;
                    //print incremental cost
                    //for(int i=mpath.size()-1; i>=0;i--)
                    //    cout<<" "<<mpath[i]->cost.v<<endl;
                    //cout<<endl;

                    ob::Cost pathcost = (pdef_->getSolutionPath())->cost(opt_);
                    og::PathGeometric *gP = (og::PathGeometric *)pdef_->getSolutionPath().get();
                    cout << "path with  = " << gP->getStateCount() << " states" << endl;
                    cout << "Path cost = " << pathcost.value() << endl;



                    addedSolution = true;
                }

                si_->freeState(xstate);
                if (rmotion->state)
                    si_->freeState(rmotion->state);
                delete rmotion;

                OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

                //JAN
                if (approximate) cout<<"approximate solution"<<endl;
                else cout<<"exact  solution"<<endl;




                return ob::PlannerStatus(addedSolution, approximate);
            }

        };




        /*!
   * createOptimizationObjectivePMD creates a PMD alignment Optimization Objective.
   * If there is only a single robot, it is basicly corresponds to the PMDs of the hand synergies.
   * If there are more than one robot it corresponds to the coupling in the x,y,z coordinates of the base.
   * \return the pointer to the OptimizationObjective
   */
        ob::OptimizationObjectivePtr omplRRTStarPlanner::createOptimizationObjectivePMD()
        {
            if (wkSpace()->getNumRobots() == 1)
            {
                //////////////////////////////////////////////////////////////////////////////
                // 3) single robot handPMD alignment optimization criteria
                int robotindex = 0;
                int numPMD = 0;

                string listcontrolsname = wkSpace()->getRobControlsName();
                vector<string*> controlname;
                string *newcontrol = new string;
                for(unsigned i=0; i<listcontrolsname.length();i++)
                {
                    if (listcontrolsname[i]=='|')
                    {
                        controlname.push_back(newcontrol);
                        newcontrol = new string;
                    }
                    else
                        newcontrol->push_back(listcontrolsname[i]);
                }
                //add last control (since listcontrolsname does not end with a |)
                controlname.push_back(newcontrol);

                vector<int> handpmdcolumns;
                for(unsigned i=0;i<controlname.size();i++)
                {
                    if (controlname[i]->find("PMD") != string::npos)
                    {
                        //store the columns of the matrix of controls that will contain the handpmd control
                        handpmdcolumns.push_back(i);
                    }
                }
                numPMD = handpmdcolumns.size();

                //be careful, if no PMD control is provides return the distance optimization function
                if (numPMD==0)
                {
                    cout<<"Error configuring the PMD matrix. No PMD controls provided. Returning distance optimization objective)"<<endl;
                    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
                }


                int numDOF = wkSpace()->getRobot(robotindex)->getNumJoints();
                ob::ProjectionMatrix PMD;
                PMD.mat = ob::ProjectionMatrix::Matrix(numDOF,numPMD);
                KthReal **mm2 = wkSpace()->getRobot(robotindex)->getMapMatrix();
                for(int j=0;j<numPMD;j++)//columns
                {
                    for(int i=0;i<numDOF;i++)//rows
                    {
                        PMD.mat(i,j) = mm2[6+i][handpmdcolumns[j]];
                    }
                }

                ob::OptimizationObjectivePtr _handpmdalignmentopti;
                _handpmdalignmentopti = ob::OptimizationObjectivePtr(new singleRobotPMDalignmentOptimizationObjective(robotindex, ss->getSpaceInformation(),PMD));
                _handpmdalignmentopti->setCostThreshold(ob::Cost(0.0));
                return _handpmdalignmentopti;
            }

            else
            {
                //////////////////////////////////////////////////////////////////////////////
                // 4) multi robot se3PMD alignment optimization criteria
                int numPMD = 0;
                int numDOF = 3*wkSpace()->getNumRobots(); //the coupling is allowed with the x,y,z of each robot

                std::map<string,vector<pair<int,int> > > pmdMap;
                std::map<string,vector<pair<int,int> > >::iterator itpmdMap;


                for(unsigned k=0;k<wkSpace()->getNumRobots(); k++)
                {
                    //find the controls that are coupled, i.e. those that have the PMD letters in their name
                    string listcontrolsname = wkSpace()->getRobControlsName();
                    vector<string*> controlname;
                    string *newcontrol = new string;
                    //split the list of controls to obtain the control names
                    for(unsigned i=0; i<listcontrolsname.length();i++)
                    {
                        if (listcontrolsname[i]=='|')
                        {
                            controlname.push_back(newcontrol);
                            newcontrol = new string;
                        }
                        else
                            newcontrol->push_back(listcontrolsname[i]);
                    }
                    //add last control (since listcontrolsname does not end with a |)
                    controlname.push_back(newcontrol);

                    //verify those control names that have the leters PMD in their name
                    for(unsigned i=0;i<controlname.size();i++)
                    {
                        if (controlname[i]->find("PMD") != string::npos)
                        {
                            //if the pmd has not yet been found (the Key does not exists), create it
                            itpmdMap=pmdMap.find(*controlname[i]);
                            if (itpmdMap == pmdMap.end())//not found
                            {
                                pair< string, vector< pair<int,int> > > pmdfound;
                                pmdfound.first = *controlname[i]; //string

                                vector< pair<int,int> > *pmdcolumns = new vector< pair<int,int> >;
                                pmdfound.second = *pmdcolumns; //indices

                                pmdMap.insert(pmdfound);
                            }

                            //add the pair indices to the map
                            pair<int,int> pmdfoundindex;
                            pmdfoundindex.first = k; //robot index
                            pmdfoundindex.second = i; //pmd index
                            pmdMap[*controlname[i]].push_back( pmdfoundindex );
                        }
                    }
                }
                numPMD = pmdMap.size();
                //check correctness
                for (itpmdMap=pmdMap.begin(); itpmdMap!=pmdMap.end(); ++itpmdMap)
                {
                    if (itpmdMap->second.size() != wkSpace()->getNumRobots())
                    {
                        cout<<"Error configuring the PMD matrix. The same number of coupled controls (PMDs) is required per robot (and with the same name!)"<<endl;
                        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
                    }
                }
                //be careful, if no PMD control is provides return the distance optimization function
                if (numPMD==0)
                {
                    cout<<"Error configuring the PMD matrix. No PMD controls provided. Returning distance optimization objective)"<<endl;
                    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
                }

                //Now load the PMD matrix
                ob::ProjectionMatrix PMD;
                PMD.mat = ob::ProjectionMatrix::Matrix(numDOF,numPMD);

                int ic=0; //column counter
                for (itpmdMap=pmdMap.begin(); itpmdMap!=pmdMap.end(); ++itpmdMap)//columns
                {
                    int ir=0; //row counter
                    for(unsigned j=0; j< itpmdMap->second.size(); j++) //this loops as many times as robots
                    {
                        int robotindex = itpmdMap->second[j].first; //the robot
                        int pmdcolumn = itpmdMap->second[j].second; //the columns in the mapMatrix of robot robotindex
                        KthReal **mm2 = wkSpace()->getRobot(robotindex)->getMapMatrix(); //the mapMatrix of robot robotindex

                        //copy the first three rows of the mapMatrix of robot robotindex, that correspond to the x,y and z coordinates
                        for(int ii=0; ii< 3; ii++)
                        {
                            PMD.mat(ir,ic) = mm2[ii][pmdcolumn];
                            ir++;//next row in PMD matrix
                        }
                    }
                    ic++;
                }

                ob::OptimizationObjectivePtr _multise3pmdalignmentopti;
                _multise3pmdalignmentopti = ob::OptimizationObjectivePtr(new multiRobotSE3PMDalignmentOptimizationObjective(ss->getSpaceInformation(),PMD));
                _multise3pmdalignmentopti->setCostThreshold(ob::Cost(0.0));
                return _multise3pmdalignmentopti;
            }
        }



        //! Constructor
        omplRRTStarPlanner::omplRRTStarPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
            omplPlanner(stype, init, goal, samples, ws, ssptr)
        {
            _guiName = "ompl RRT Star Planner";
            _idName = "omplRRTStar";

            //create planner
            ob::PlannerPtr planner(new myRRTstar(si));
            //ob::PlannerPtr planner(new og::RRTstar(si));

            //set planner parameters: range, goalbias, delay collision checking and optimization option
            _Range = 0.05;
            _GoalBias = (planner->as<myRRTstar>())->getGoalBias();
            _PathBias = 0.05;//_GoalBias;
            _PathSamplingRangeFactor = 10.0;
            _NodeRejection = 0.0;//0.8;
            _DelayCC = (planner->as<myRRTstar>())->getDelayCC();
            _opti = 1; //optimize path lenght by default

            addParameter("Range", _Range);
            addParameter("Goal Bias", _GoalBias);
            addParameter("Path Bias", _PathBias);
            addParameter("Path Sampling Range Factor", _PathSamplingRangeFactor);
            addParameter("Node Rejection", _NodeRejection);
            addParameter("DelayCC (0/1)", _DelayCC);
            addParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)", _opti);
            addParameter("Path Length Weight",0.00001);
            addParameter("KP",1.);
            addParameter("KI",1.);
            addParameter("KD",1.);

            planner->as<myRRTstar>()->setRange(_Range);
            if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
                space->setup();
            }
            planner->as<myRRTstar>()->setGoalBias(_GoalBias);
            planner->as<myRRTstar>()->setPathBias(_PathBias);
            planner->as<myRRTstar>()->setPathSamplingRangeFactor(_PathSamplingRangeFactor);
            planner->as<myRRTstar>()->setNodeRejection(_NodeRejection);
            planner->as<myRRTstar>()->setDelayCC(_DelayCC);


            //set the parameter that modifies the number K of near neighbors to search in the RRT* solve
            _KneighFactor = 1.0;
            addParameter("K-Neigh Factor", _KneighFactor);
            planner->as<myRRTstar>()->setNeighFactor(_KneighFactor);

            //////////////////////////////////////////////////////////////////////////////
            //START optimization criteria:

            ob::ProblemDefinitionPtr pdefPtr = ss->getProblemDefinition();

            //////////////////////////////////////////////////////////////////////////////
            // 1) Length optimization criteria
            _lengthopti = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));

            //////////////////////////////////////////////////////////////////////////////
            // 2) clearance optimization criteria
            //_clearanceopti = ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(ss->getSpaceInformation()));


            _clearanceopti = ob::OptimizationObjectivePtr(new myICOptimizationObjective(ss->getSpaceInformation(),this));

            //////////////////////////////////////////////////////////////////////////////
            // 3) pmd alignment optimization criteria
            _pmdalignmentopti = createOptimizationObjectivePMD();

            //////////////////////////////////////////////////////////////////////////////
            _lengthweight = 0.1;
            addParameter("lengthweight(0..1)", _lengthweight);
            _penalizationweight = 1.0;
            addParameter("penalizationweight", _penalizationweight);
            _orientationweight = 1.0;
            addParameter("orientationweight", _orientationweight);


            //_multiopti = ob::OptimizationObjectivePtr(new ob::MultiOptimizationObjective(ss->getSpaceInformation()));
            //((ob::MultiOptimizationObjective*)_multiopti.get())->addObjective(_lengthopti,_lengthweight);
            //((ob::MultiOptimizationObjective*)_multiopti.get())->addObjective(_pcaalignmentopti,1.0-_lengthweight);
            //_pcaalignmentopti2 = ob::OptimizationObjectivePtr(new PCAalignmentOptimizationObjective2(ss->getSpaceInformation(),dimpca));
            //_pcaalignmentopti3 = ob::OptimizationObjectivePtr(new PCAalignmentOptimizationObjective3(ss->getSpaceInformation(),dimpca));

            switch (_opti) {
                case 0:
                    _optiselected = _lengthopti; //dummy
                    planner->as<myRRTstar>()->setOptimize(false); //Disable optimization
                break;
                case 1:
                    _optiselected = _lengthopti; //length optimization
                    planner->as<myRRTstar>()->setOptimize(true);
                break;
                case 2:
                    _optiselected = _clearanceopti;//clearnace optimization
                    planner->as<myRRTstar>()->setOptimize(true);
                break;
                case 3:
                    _optiselected = _pmdalignmentopti; //pmd alignment optimization
                    planner->as<myRRTstar>()->setOptimize(true);
                break;
                default:
                    _opti = 1;
                    _optiselected = _lengthopti; //length optimization
                    setParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)",_opti);
                    planner->as<myRRTstar>()->setOptimize(true);
                break;
            }
            pdefPtr->setOptimizationObjective(_optiselected);
            planner->setProblemDefinition(pdefPtr);
            planner->setup();

            //set the planner
            ss->setPlanner(planner);

            //for the RRT type of planners we do not want to constrain the samppling in the PMD subspace, then disable these controls
            _disablePMDControlsFromSampling=1;
            addParameter("disablePMDControlsFromSampling", _disablePMDControlsFromSampling);
            disablePMDControlsFromSampling();
        }

        //! void destructor
        omplRRTStarPlanner::~omplRRTStarPlanner(){

        }

        bool omplRRTStarPlanner::setPotentialCost(string filename) {
            return ((myICOptimizationObjective*) _clearanceopti.get())->setPotentialCost(filename);
        }



        //! function to find a solution path
        bool omplRRTStarPlanner::trySolve()
        {
            bool ret;
            //solve
            ret = omplPlanner::trySolve();
            if (ret)
            {
                //evaluate path
                cout << "path with " << ((og::PathGeometric)ss->getSolutionPath()).getStateCount() << " states" << endl;
                ob::Cost pathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
                cout << "Path cost = " << pathcost.value() << endl;
                if (_opti==3)
                {
                    //store the values
                    double ow = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getOrientationWeight();
                    double dw = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getDistanceWeight();
                    double pw = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getOrientationPenalization();
                    //eval only the alignment with the PMDs
                    _orientationweight = 1.0;
                    _lengthweight = 0.0;
                    _penalizationweight = 0.0;
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);

                    ob::Cost alignmentpathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
                    cout << "Path alignment cost = " << alignmentpathcost.value() << endl;

                    //eval only the distance
                    _orientationweight = 0.0;
                    _lengthweight = 1.0;
                    _penalizationweight = 0.0;
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);

                    ob::Cost distancepathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
                    cout << "Path distance cost = " << distancepathcost.value() << endl;

                    cout << "Alignment cost per unitary distance traveled = " <<
                            (alignmentpathcost.value()/distancepathcost.value()) << endl;

                    //restore the values
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(ow);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(dw);
                    ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(pw);
                }
            }
            else{
                cout<<"No solution found"<<endl;
            }

            return ret;

        }

        //! setParameters sets the parameters of the planner
        bool omplRRTStarPlanner::setParameters() {
            if (!omplPlanner::setParameters()) return false;

            try {
                HASH_S_K::iterator it;


                it = _parameters.find("Optimize none(0)/dist(1)/clear(2)/PMD(3)");
                if (it == _parameters.end()) return false;
                _opti = it->second;
                switch (_opti) {
                    case 0:
                        _optiselected = _lengthopti; //dummy
                    break;
                    case 1:
                        _optiselected = _lengthopti; //length optimization
                    break;
                    case 2:
                        _optiselected = _clearanceopti;//clearance optimization
                    break;
                    case 3:
                        _optiselected = _pmdalignmentopti; //pmd alignment optimization
                    break;
                    default:
                        _opti = 1;
                        _optiselected = _lengthopti; //length optimization
                        setParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)", _opti);
                    break;
                }
                ss->getProblemDefinition()->setOptimizationObjective(_optiselected);
                ss->getPlanner()->as<myRRTstar>()->setOptimize(_opti != 0);
                ss->getPlanner()->setup();


                it = _parameters.find("disablePMDControlsFromSampling");
                if (it == _parameters.end()) return false;
                _disablePMDControlsFromSampling = it->second;
                disablePMDControlsFromSampling(_disablePMDControlsFromSampling == 0);


                it = _parameters.find("Goal Bias");
                if (it == _parameters.end()) return false;
                _GoalBias = it->second;
                ss->getPlanner()->as<og::RRTstar>()->setGoalBias(_GoalBias);


                it = _parameters.find("Path Bias");
                if (it == _parameters.end()) return false;
                _PathBias = it->second;
                ss->getPlanner()->as<myRRTstar>()->setPathBias(_PathBias);


                it = _parameters.find("Path Sampling Range Factor");
                if (it == _parameters.end()) return false;
                _PathSamplingRangeFactor = it->second;
                ss->getPlanner()->as<myRRTstar>()->setPathSamplingRangeFactor(_PathSamplingRangeFactor);


                it = _parameters.find("Node Rejection");
                if (it == _parameters.end()) return false;
                _NodeRejection = it->second;
                ss->getPlanner()->as<myRRTstar>()->setNodeRejection(_NodeRejection);


                it = _parameters.find("K-Neigh Factor");
                if (it == _parameters.end()) return false;
                _KneighFactor = it->second;
                ss->getPlanner()->as<myRRTstar>()->setNeighFactor(_KneighFactor);


                it = _parameters.find("DelayCC (0/1)");
                if (it == _parameters.end()) return false;
                _DelayCC = (it->second != 0);
                ss->getPlanner()->as<og::RRTstar>()->setDelayCC(_DelayCC);


                it = _parameters.find("KP");
                if (it == _parameters.end()) return false;
                if (_opti == 2) ((myICOptimizationObjective*) _clearanceopti.get())->setKP(it->second);

                it = _parameters.find("KI");
                if (it == _parameters.end()) return false;
                if (_opti == 2) ((myICOptimizationObjective*) _clearanceopti.get())->setKI(it->second);

                it = _parameters.find("KD");
                if (it == _parameters.end()) return false;
                if (_opti == 2) ((myICOptimizationObjective*) _clearanceopti.get())->setKD(it->second);


                it = _parameters.find("Range");
                if (it == _parameters.end()) return false;
                _Range = it->second;
                ss->getPlanner()->as<og::RRTstar>()->setRange(_Range);
                if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                    space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
                    space->setup();
                }
                if (_opti == 3) ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setEpsilon(_Range);


                it = _parameters.find("lengthweight(0..1)");
                if (it == _parameters.end()) return false;
                _lengthweight = it->second;
                if (_lengthweight < 0. || _lengthweight > 1.) _lengthweight = 0.5;
                if (_opti == 3) ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);


                it = _parameters.find("penalizationweight");
                if (it == _parameters.end()) return false;
                _penalizationweight = it->second;
                if (_opti == 3) ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);


                it = _parameters.find("orientationweight");
                if (it == _parameters.end()) return false;
                _orientationweight = it->second;
                if (_opti == 3) ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
            } catch(...) {
                return false;
            }
            return true;
        }
    }
}


#endif // KAUTHAM_USE_OMPL
