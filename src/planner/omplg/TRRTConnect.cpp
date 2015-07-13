/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#include "TRRTConnect.h"

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>


ompl::geometric::TRRTConnect::TRRTConnect(const base::SpaceInformationPtr &si) :
    base::Planner(si,"TRRTConnect"),
    maxDistance_(0.0),//Set in setup()
    kConstant_(0.0),//Set in setup()
    frontierThreshold_(0.0),//Set in setup()
    frontierNodeRatio_(0.1),//1/10, or 1 nonfrontier for every 10 frontier
    tempChangeFactor_(2.0),//How much to decrease or increase the temp each time
    minTemperature_(10e-10),//Lower limit of the temperature change
    initTemperature_(10e-6),//Where the temperature starts out
    maxStatesSucceed_(2),//Threshold for when to start decreasing the temperature
    maxStatesFailed_(10),//Threshold for when to start increasing the temperature
    connectionPoint_(std::make_pair<base::State*,base::State*>(NULL,NULL)) {

    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range",this,&TRRTConnect::setRange,
                                  &TRRTConnect::getRange,"0.:1.:10000.");
    Planner::declareParam<unsigned int>("max_states_succeed",this,
                                        &TRRTConnect::setMaxStatesSucceed,
                                        &TRRTConnect::getMaxStatesSucceed,"0:1000");
    Planner::declareParam<unsigned int>("max_states_failed",this,
                                        &TRRTConnect::setMaxStatesFailed,
                                        &TRRTConnect::getMaxStatesFailed,"0:1000");
    Planner::declareParam<double>("temp_change_factor",this,
                                  &TRRTConnect::setTempChangeFactor,
                                  &TRRTConnect::getTempChangeFactor,"0.:.1:10.");
    Planner::declareParam<double>("min_temperature",this,
                                  &TRRTConnect::setMinTemperature,
                                  &TRRTConnect::getMinTemperature);
    Planner::declareParam<double>("init_temperature",this,
                                  &TRRTConnect::setInitTemperature,
                                  &TRRTConnect::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold",this,
                                  &TRRTConnect::setFrontierThreshold,
                                  &TRRTConnect::getFrontierThreshold);
    Planner::declareParam<double>("frontierNodeRatio",this,
                                  &TRRTConnect::setFrontierNodeRatio,
                                  &TRRTConnect::getFrontierNodeRatio);
    Planner::declareParam<double>("k_constant",this,&TRRTConnect::setKConstant,
                                  &TRRTConnect::getKConstant);
}


ompl::geometric::TRRTConnect::~TRRTConnect() {
    freeMemory();
}


void ompl::geometric::TRRTConnect::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_,getName());

    //Set optimization objective
    if (pdef_->hasOptimizationObjective()) {
        opt_ = pdef_->getOptimizationObjective();
    } else {
        opt_.reset(new base::MechanicalWorkOptimizationObjective(si_));
        OMPL_INFORM("%s: No optimization objective specified.",getName().c_str());
        OMPL_INFORM("%s: Defaulting to optimizing path length.",getName().c_str());
    }

    //Set maximum distance a new node can be from its nearest neighbor
    if (maxDistance_ < std::numeric_limits<double>::epsilon()) {
        sc.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    //Set the threshold to decide if a new node is a frontier node
    if (frontierThreshold_ < std::numeric_limits<double>::epsilon()) {
        frontierThreshold_ = 0.01*si_->getMaximumExtent();
        OMPL_DEBUG("%s: Frontier threshold detected to be %lf",
                   getName().c_str(),frontierThreshold_);
    }

    //Autoconfigure the K constant
    if (kConstant_ < std::numeric_limits<double>::epsilon()) {
        //Find the average cost of states by sampling
        kConstant_ = opt_->averageStateCost(magic::TEST_STATE_COUNT).v;
    }

    if (!tStart_) tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>
                                (si_->getStateSpace()));
    tStart_->setDistanceFunction(boost::bind(&TRRTConnect::distanceFunction,this,_1,_2));
    clearTree(tStart_);

    if (!tGoal_) tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>
                              (si_->getStateSpace()));
    tGoal_->setDistanceFunction(boost::bind(&TRRTConnect::distanceFunction,this,_1,_2));
    clearTree(tGoal_);
}


void ompl::geometric::TRRTConnect::freeTreeMemory(TreeData tree) {
    if (tree) {
        std::vector<Motion*> motions;
        tree->list(motions);
        for (unsigned int i(0); i < motions.size(); ++i) {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}


void ompl::geometric::TRRTConnect::freeMemory() {
    freeTreeMemory(tStart_);
    freeTreeMemory(tGoal_);
}


void ompl::geometric::TRRTConnect::clearTree(TreeData tree) {
    if (tree) tree->clear();
    tree.numStatesFailed = 0;
    tree.temp = initTemperature_;
    tree.nonfrontierCount = 1;
    tree.frontierCount = 1;//Init to 1 to prevent division by zero error
}


void ompl::geometric::TRRTConnect::clear() {
    Planner::clear();

    sampler_.reset();

    freeMemory();

    clearTree(tStart_);
    clearTree(tGoal_);

    connectionPoint_ = std::make_pair<base::State*,base::State*>(NULL,NULL);
}


ompl::geometric::TRRTConnect::GrowState
ompl::geometric::TRRTConnect::growTree(TreeData &tree,
                                       TreeGrowingInfo &tgi, Motion *rmotion) {
    //Find closest state in the tree
    Motion *nmotion(tree->nearest(rmotion));

    //Assume we can reach the state we go towards
    bool reach(true);

    //Find state to add
    base::State *dstate(rmotion->state);
    double d(si_->distance(nmotion->state,dstate));
    if (d > maxDistance_) {
        si_->getStateSpace()->interpolate(nmotion->state,rmotion->state,
                                          maxDistance_/d,tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }

    //Minimum Expansion Control
    //A possible side effect may appear when the tree expansion towards
    //unexplored regions remains slow, and the new nodes contribute
    //only to refine already explored regions.
    if (!minExpansionControl(d,tree)) {
        return TRAPPED;//Give up on this one and try a new sample
    }

    base::Cost cost(opt_->motionCost(tgi.start ? nmotion->state : dstate,
                                     tgi.start ? dstate : nmotion->state));

    //Only add this motion to the tree if the transition test accepts it
    if (!transitionTest(cost,d,tree)) {
        return TRAPPED;//Give up on this one and try a new sample
    }

    //If we are in the start tree, we just check the motion like we normally do.
    //If we are in the goal tree, we need to check the motion in reverse,
    //but checkMotion() assumes the first state it receives as argument is valid,
    //so we check that one first.
    bool validMotion(tgi.start ? si_->checkMotion(nmotion->state,dstate) :
                                 (si_->getStateValidityChecker()->isValid(dstate) &&
                                  si_->checkMotion(dstate,nmotion->state)));
    if (validMotion) {
        //Create a motion
        Motion *motion(new Motion(si_));
        si_->copyState(motion->state,dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);

        return reach ? REACHED : ADVANCED;
    } else {
        return TRAPPED;
    }
}


ompl::base::PlannerStatus
ompl::geometric::TRRTConnect::solve(const base::PlannerTerminationCondition &ptc) {
    checkValidity();
    base::GoalSampleableRegion *goal(dynamic_cast<base::GoalSampleableRegion*>
                                     (pdef_->getGoal().get()));

    if (!goal) {
        OMPL_ERROR("%s: Unknown type of goal",getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart()) {
        Motion *motion(new Motion(si_));
        si_->copyState(motion->state,st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0) {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!",
                   getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region",
                   getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure",
                getName().c_str(),int(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *rmotion(new Motion(si_));
    base::State *rstate(rmotion->state);
    bool startTree(true);
    bool solved(false);

    double temp(0.0);
    unsigned int n(0);

    while (!ptc) {
        TreeData &tree(startTree ? tStart_ : tGoal_);
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree(startTree ? tStart_ : tGoal_);

        if (((2*pis_.getSampledGoalsCount()) < (tGoal_->size())) ||
                (tGoal_->size() == 0)) {
            const base::State *st((tGoal_->size() == 0) ?
                                      pis_.nextGoal(ptc) : pis_.nextGoal());
            if (st) {
                Motion *motion(new Motion(si_));
                si_->copyState(motion->state,st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0) {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree",
                           getName().c_str());
                break;
            }
        }

        //Sample random state
        sampler_->sampleUniform(rstate);

        GrowState gs(growTree(tree,tgi,rmotion));
        temp += tree.temp;
        n++;

        if (gs != TRAPPED) {
            //Remember which motion was just added
            Motion *addedMotion(tgi.xmotion);

            //Attempt to connect trees

            //If reached, it means we used rstate directly, no need to copy again
            if (gs != REACHED) si_->copyState(rstate,tgi.xstate);

            GrowState gsc(ADVANCED);
            tgi.start = startTree;
            while (gsc == ADVANCED) {
                gsc = growTree(otherTree,tgi,rmotion);
                temp += otherTree.temp;
                n++;
            }

            Motion *startMotion(startTree ? tgi.xmotion : addedMotion);
            Motion *goalMotion(startTree ? addedMotion : tgi.xmotion);

            //If we connected the trees in a valid way (start and goal pair is valid)
            if ((gsc == REACHED) && goal->isStartGoalPairValid(startMotion->root,
                                                               goalMotion->root)) {
                //It must be the case that either the start tree or the goal tree
                //has made some progress so one of the parents is not NULL.
                //We go one step 'back' to avoid having a duplicate state
                //on the solution path
                if (startMotion->parent) {
                    startMotion = startMotion->parent;
                } else {
                    goalMotion = goalMotion->parent;
                }

                connectionPoint_ = std::make_pair(startMotion->state,goalMotion->state);

                //Construct the solution path
                Motion *solution(startMotion);
                std::vector<Motion*> mpath1;
                while (solution) {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion*> mpath2;
                while (solution) {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                PathGeometric *path(new PathGeometric(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i(mpath1.size() - 1); i >= 0; --i) {
                    path->append(mpath1[i]->state);
                }
                for (unsigned int i(0); i < mpath2.size(); ++i) {
                    path->append(mpath2[i]->state);
                }

                pdef_->addSolutionPath(base::PathPtr(path),false,0.0);
                solved = true;
                break;
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)",getName().c_str(),
                tStart_->size()+tGoal_->size(),tStart_->size(),tGoal_->size());

    OMPL_INFORM("%s: Average temperature %f ",getName().c_str(),temp/double(n));

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}


void ompl::geometric::TRRTConnect::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_) tStart_->list(motions);

    for (unsigned int i(0); i < motions.size(); ++i) {
        if (!motions[i]->parent) {
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state,1));
        } else {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state,1),
                         base::PlannerDataVertex(motions[i]->state,1));
        }
    }

    motions.clear();
    if (tGoal_) tGoal_->list(motions);

    for (unsigned int i(0); i < motions.size(); ++i) {
        if (!motions[i]->parent) {
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state,2));
        } else {
            //The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state,2),
                         base::PlannerDataVertex(motions[i]->parent->state,2));
        }
    }

    //Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first),
                 data.vertexIndex(connectionPoint_.second));
}


bool ompl::geometric::TRRTConnect::transitionTest(base::Cost cost,
                                                  double distance, TreeData &tree) {
    //Always accept if motionCost has a negative cost
    if (cost.v < 0.0) return true;

    //Difference in cost
    double slope(cost.v / std::min(distance,maxDistance_));

    //The probability of acceptance of a new motion is defined by its cost.
    //Based on the Metropolis criterion.
    double transitionProbability(exp(-slope/(kConstant_*tree.temp)));

    //Check if we can accept it
    if (rng_.uniform01() <= transitionProbability) {
        //State has succeed
        ++tree.numStatesSucceed;

        //Update temperature
        if (tree.numStatesSucceed > maxStatesSucceed_) {
            tree.temp /= tempChangeFactor_;
            //Prevent temperature from getting too small
            if (tree.temp < minTemperature_) tree.temp = minTemperature_;

            tree.numStatesSucceed = 0;
        }

        return true;
    } else {
        //State has failed
        ++tree.numStatesFailed;

        //Update temperature
        if (tree.numStatesFailed > maxStatesFailed_) {
            tree.temp *= tempChangeFactor_;

            tree.numStatesFailed = 0;
        }

        return false;
    }
}


bool ompl::geometric::TRRTConnect::minExpansionControl(double distance, TreeData &tree) {
    //Decide to accept or not
    if (distance > frontierThreshold_) {
        //Participates in the tree expansion
        ++tree.frontierCount;
    } else {
        //Participates in the tree refinement

        //Check our ratio first before accepting it
        if ((double(tree.frontierCount)*frontierNodeRatio_) <
                double(tree.nonfrontierCount)) {

            //Increment so that the temperature rises faster
            ++tree.numStatesFailed;

            //Reject this node as being too much refinement
            return false;
        } else {
            ++tree.nonfrontierCount;
        }
    }

    return true;
}
