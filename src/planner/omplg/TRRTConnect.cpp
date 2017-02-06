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


#include <kautham/planner/omplg/TRRTConnect.h>
#include <kautham/planner/omplg/FOSOptimizationObjective.h>

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>

#include <boost/math/constants/constants.hpp>

unsigned int extendCount, connectCount;


ompl::geometric::TRRTConnect::TRRTConnect(const base::SpaceInformationPtr &si) :
    base::Planner(si,"TRRTConnect"),
    delayCC_(true),
    maxDistance_(0.0),//Set in setup()
    kConstant_(0.0),//Set in setup()
    frontierThreshold_(0.0),//Set in setup()
    frontierNodeRatio_(0.1),//1/10, or 1 nonFrontier for every 10 frontier
    tempChangeFactor_(2.0),//How much to decrease or increase the temp each time
    minTemperature_(10e-10),//Lower limit of the temperature change
    initTemperature_(10e-6),//Where the temperature starts out
    maxStatesSucceed_(2),//Threshold for when to start decreasing the temperature
    maxStatesFailed_(10),//Threshold for when to start increasing the temperature
    connectionPoint_(std::make_pair<base::State*,base::State*>(NULL,NULL)) {

    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    tStart_.start_ = true;
    tGoal_.start_ = false;

    Planner::declareParam<bool>("delay_collision_checking",this,&TRRTConnect::setDelayCC,
                                &TRRTConnect::getDelayCC,"0,1");
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
    
    tStart_.root = si_->allocState();
    tGoal_.root = si_->allocState();
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
        kConstant_ = opt_->averageStateCost(magic::TEST_STATE_COUNT).value();
    }

    //Create the nearest neighbor function the first time setup is run
#if OMPL_VERSION_VALUE < 1001000 // 1.1.0
    if (!tStart_) tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>
                                   (si_->getStateSpace()));
    if (!tGoal_) tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>
                                   (si_->getStateSpace()));
#else
    if (!tStart_) tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    if (!tGoal_) tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
#endif

    //Set the distance function
    tStart_->setDistanceFunction(std::bind(&TRRTConnect::distanceFunction,this,std::placeholders::_1,std::placeholders::_2));
    tGoal_->setDistanceFunction(std::bind(&TRRTConnect::distanceFunction,this,std::placeholders::_1,std::placeholders::_2));

    //Setup TRRTConnect specific variables
    clearTree(tStart_);
    clearTree(tGoal_);

    if (tStart_.compareFn_) delete tStart_.compareFn_;
    tStart_.compareFn_ = new CostIndexCompare(tStart_.costs_,*opt_);

    if (tGoal_.compareFn_) delete tGoal_.compareFn_;
    tGoal_.compareFn_ = new CostIndexCompare(tGoal_.costs_,*opt_);

    k_rrg_ = boost::math::constants::e<double>()*
            double(si_->getStateSpace()->getDimension()+1)/
            double(si_->getStateSpace()->getDimension());//e+e/d
}


void ompl::geometric::TRRTConnect::freeTreeMemory(TreeData &tree) {
    //Delete all motions, states and the nearest neighbors data structure
    if (tree) {
        std::vector<Motion*> motions;
        tree->list(motions);
        for (std::size_t i(0); i < motions.size(); ++i) {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

}


void ompl::geometric::TRRTConnect::freeMemory() {
    freeTreeMemory(tStart_);
    freeTreeMemory(tGoal_);
}


void ompl::geometric::TRRTConnect::clearTree(TreeData &tree) {
    if (tree) tree->clear();
    //Clear TRRTConnect specific variables
    tree.numStatesFailed_ = 0;
    tree.temp_ = initTemperature_;
    tree.nonFrontierCount_ = 0;
    tree.frontierCount_ = 0;
    tree.costs_.clear();
    tree.sortedCostIndices_.clear();
    //If opt_ is of type FOSOptimizationObjective, there is a global zero-order box and
    //then this parameter will be used and initially set to false
    //If opt is of another type, this parameter is set to true to avoid its use
    tree.stateInBoxZos_ = !dynamic_cast<ompl::base::FOSOptimizationObjective*>(opt_.get());
}


void ompl::geometric::TRRTConnect::clear() {
    Planner::clear();

    sampler_.reset();

    freeMemory();

    clearTree(tStart_);
    clearTree(tGoal_);

    connectionPoint_ = std::make_pair<base::State*,base::State*>(NULL,NULL);
}


ompl::geometric::TRRTConnect::ExtendResult
ompl::geometric::TRRTConnect::extend(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion) {
    ++extendCount;

    Motion *nmotion;
    /*if (tree.stateInBoxZos_) {
        //Among the nearest K nodes in the tree, find state with lowest cost to go
        //The returned nmotion is collision-free
        //If no collision-free motion is found, NULL is returned
        nmotion = minCostNeighbor(tree,tgi,rmotion);
        if (!nmotion) return TRAPPED;
    } else {*/
        //Find nearest node in the tree
        //The returned motion has not been checked for collisions
        //It never returns NULL
        nmotion = tree->nearest(rmotion);
    //}

    //State to add
    base::State *dstate;

    //Distance from near state to random state
    double d(si_->distance(nmotion->state,rmotion->state));

    //Check if random state is too far away
    if (d > maxDistance_) {
        si_->getStateSpace()->interpolate(nmotion->state,rmotion->state,
                                          maxDistance_/d,tgi.xstate);

        //Use the interpolated state as the new state
        dstate = tgi.xstate;
    } else {
        //Random state is close enough
        dstate = rmotion->state;
    }

    //Check for collisions
    //if (!tree.stateInBoxZos_) {
        //If we are in the start tree, we just check the motion like we normally do.
        //If we are in the goal tree, we need to check the motion in reverse,
        //but checkMotion() assumes the first state it receives as argument is valid,
        //so we check that one first.
        bool validMotion(tree.start_ ? si_->checkMotion(nmotion->state,dstate) :
                                     (si_->getStateValidityChecker()->isValid(dstate) &&
                                      si_->checkMotion(dstate,nmotion->state)));
        if (!validMotion) return TRAPPED;
    //}

    //Minimum Expansion Control
    //A possible side effect may appear when the tree expansion towards
    //unexplored regions remains slow, and the new nodes contribute
    //only to refine already explored regions.
    if (!minExpansionControl(d,tree)) {
        return TRAPPED;//Give up on this one and try a new sample
    }

    //Compute motion cost
    base::Cost cost;
    if (tree.stateInBoxZos_) {
        if (tree.start_) {
            cost = opt_->motionCost(nmotion->state,dstate);
        } else {
            cost = opt_->motionCost(dstate,nmotion->state);
        }
    } else {
        //opt_ is of type FOSOptimizationObjective,
        //there is no need to check the conversion
        cost = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                preSolveMotionCost(nmotion->state,dstate);
    }

    //Only add this motion to the tree if the transition test accepts it
    //Temperature must be updated
    if (!transitionTest(cost,d,tree,true)) {
        return TRAPPED;//Give up on this one and try a new sample
    }

    //Create a motion
    Motion *motion(new Motion(si_));
    si_->copyState(motion->state,dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    tgi.xmotion = motion;

    //Add motion to the tree
    tree->add(motion);

    //Check if now the tree has a motion inside BoxZos
    if (!tree.stateInBoxZos_) {
        tree.stateInBoxZos_ = cost.value() < DBL_EPSILON*std::min(d,maxDistance_);
        if (tree.stateInBoxZos_) tree.temp_ = initTemperature_;
    }

    //Update frontier nodes and non frontier nodes count
    if (d > frontierThreshold_) {//Participates in the tree expansion
        ++tree.frontierCount_;
    } else {//Participates in the tree refinement
        ++tree.nonFrontierCount_;
    }

    return (d > maxDistance_) ? ADVANCED : REACHED;
}


bool ompl::geometric::TRRTConnect::connect(TreeData &tree, TreeGrowingInfo &tgi,
                                           Motion *rmotion) {
    ++connectCount;
unsigned int steps = 0;
    Motion *nmotion;
    if (tree.stateInBoxZos_) {
        //Among the nearest K nodes in the tree, find state with lowest cost to go
        //The returned nmotion is collision-free
        //If no collision-free motion is found, NULL is returned
        nmotion = minCostNeighbor(tree,tgi,rmotion);
        if (!nmotion) return false;
    } else {
        //Find nearest node in the tree
        //The returned motion has not checked for collisions
        //It never returns NULL
        nmotion = tree->nearest(rmotion);
    }

    //State to add
    base::State *dstate;

    //Distance from near state to random state
    double d(si_->distance(nmotion->state,rmotion->state));

    //Check if random state is too far away
    if (d > maxDistance_) {
        si_->getStateSpace()->interpolate(nmotion->state,rmotion->state,
                                          maxDistance_/d,tgi.xstate);

        //Use the interpolated state as the new state
        dstate = tgi.xstate;
    } else {
        //Random state is close enough
        dstate = rmotion->state;
    }

    //Check for collisions
    if (!tree.stateInBoxZos_) {
        //If we are in the start tree, we just check the motion like we normally do.
        //If we are in the goal tree, we need to check the motion in reverse,
        //but checkMotion() assumes the first state it receives as argument is valid,
        //so we check that one first.
        bool validMotion(tree.start_? si_->checkMotion(nmotion->state,dstate) :
                                     (si_->getStateValidityChecker()->isValid(dstate) &&
                                      si_->checkMotion(dstate,nmotion->state)));
        if (!validMotion) return false;
    }

    //Compute motion cost
    base::Cost cost;
    if (tree.stateInBoxZos_) {
        if (tree.start_) {
            cost = opt_->motionCost(nmotion->state,dstate);
        } else {
            cost = opt_->motionCost(dstate,nmotion->state);
        }
    } else {
        //opt_ is of type FOSOptimizationObjective,
        //there is no need to check the conversion
        cost = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                preSolveMotionCost(nmotion->state,dstate);
    }

    //Only add this motion to the tree if the transition test accepts it
    //Temperature must not be updated
    while (transitionTest(cost,d,tree,false)) {
        //Create a motion
        Motion *motion(new Motion(si_));
        si_->copyState(motion->state,dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        //Add motion to the tree
        tree->add(motion);
        steps++;

        //Check if now the tree has a motion inside BoxZos
        if (!tree.stateInBoxZos_) {
            //tree.stateInBoxZos_ = cost.v < DBL_EPSILON*std::min(d,maxDistance_);
            tree.stateInBoxZos_ = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                    boxZosDistance(motion->state) < DBL_EPSILON;
            if (tree.stateInBoxZos_) tree.temp_ = initTemperature_;
        }

        //Update frontier nodes and non frontier nodes count
        ++tree.frontierCount_;//Participates in the tree expansion

        //If reached
        if (dstate == rmotion->state) {
       //     std::cout << steps << " steps" << std::endl;
            return true;
        }

        //Current near motion is the motion just added
        nmotion = motion;

        //Distance from near state to random state
        d = si_->distance(nmotion->state,rmotion->state);

        //Check if random state is too far away
        if (d > maxDistance_) {
            si_->getStateSpace()->interpolate(nmotion->state,rmotion->state,
                                              maxDistance_/d,tgi.xstate);

            //Use the interpolated state as the new state
            dstate = tgi.xstate;
        } else {
            //Random state is close enough
            dstate = rmotion->state;
        }

        //Check for collisions
        //If we are in the start tree, we just check the motion like we normally do.
        //If we are in the goal tree, we need to check the motion in reverse,
        //but checkMotion() assumes the first state it receives as argument is valid,
        //so we check that one first.
        bool validMotion(tree.start_ ? si_->checkMotion(nmotion->state,dstate) :
                                     (si_->getStateValidityChecker()->isValid(dstate) &&
                                      si_->checkMotion(dstate,nmotion->state)));
        if (!validMotion) return false;

        //Compute motion cost
        if (tree.stateInBoxZos_) {
            if (tree.start_) {
                cost = opt_->motionCost(nmotion->state,dstate);
            } else {
                cost = opt_->motionCost(dstate,nmotion->state);
            }
        } else {
            //opt_ is of type FOSOptimizationObjective,
            //there is no need to check the conversion
            cost = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                    preSolveMotionCost(nmotion->state,dstate);
        }
    }

  //  std::cout << steps << " steps" << std::endl;
    return false;
}


ompl::base::PlannerStatus
ompl::geometric::TRRTConnect::solve(const base::PlannerTerminationCondition &ptc) {
    //Basic error checking
    checkValidity();

    //Goal information
    base::GoalSampleableRegion *goal(dynamic_cast<base::GoalSampleableRegion*>
                                     (pdef_->getGoal().get()));
    if (!goal) {
        OMPL_ERROR("%s: Unknown type of goal",getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    //Input States
    //Loop through valid input states and add to tree
    while (const base::State *st = pis_.nextStart()) {
        //Create a motion
        Motion *motion(new Motion(si_));
        si_->copyState(motion->state,st);
        motion->root = motion->state;

        //Add motion to the tree
        tStart_->add(motion);
        si_->copyState(tStart_.root,motion->root);

        //Check if now the tree has a motion inside BoxZos
        if (!tStart_.stateInBoxZos_) {
            tStart_.stateInBoxZos_ = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                    boxZosDistance(motion->state) < DBL_EPSILON;
            if (tStart_.stateInBoxZos_) tStart_.temp_ = initTemperature_;
        }

        //Update frontier nodes and non frontier nodes count
        ++tStart_.frontierCount_;//Participates in the tree expansion
    }

    //Check that input states exist
    if (tStart_->size() == 0) {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!",
                   getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    //Check that goal states exist
    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region",
                   getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    //Create state sampler if the first run
    if (!sampler_) sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure",
                getName().c_str(),int(tStart_->size() + tGoal_->size()));

    Motion *rmotion(new Motion(si_));
    base::State *rstate(rmotion->state);

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    bool startTree(true);
    bool solved(false);

    double temp(0.0);
    unsigned int iter(0);
    extendCount = 0;
    connectCount = 0;

    //Begin sampling
    while (!ptc) {
        //Choose tree to extend
        TreeData &tree(startTree ? tStart_ : tGoal_);
        startTree = !startTree;
        TreeData &otherTree(startTree ? tStart_ : tGoal_);

        if (((2*pis_.getSampledGoalsCount()) < (tGoal_->size())) ||
                (tGoal_->size() == 0)) {
            const base::State *st((tGoal_->size() == 0) ?
                                      pis_.nextGoal(ptc) : pis_.nextGoal());
            if (st) {
                //Create a motion
                Motion *motion(new Motion(si_));
                si_->copyState(motion->state,st);
                motion->root = motion->state;

                //Add motion to the tree
                tGoal_->add(motion);
                si_->copyState(tGoal_.root,motion->root);

                //Check if now the tree has a motion inside BoxZos
                if (!tGoal_.stateInBoxZos_) {
                    tGoal_.stateInBoxZos_ = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                            boxZosDistance(motion->state) < DBL_EPSILON;
                    if (tGoal_.stateInBoxZos_) tGoal_.temp_ = initTemperature_;
                }

                //Update frontier nodes and non frontier nodes count
                ++tGoal_.frontierCount_;//Participates in the tree expansion
            }

            if (tGoal_->size() == 0) {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree",
                           getName().c_str());
                break;
            }
        }

        //Sample random state
        if (rng_.uniform01() < 0.05) {
            si_->copyState(rstate,otherTree.root);
        } else {
            sampler_->sampleUniform(rstate);
        }

        //Extend tree
        ExtendResult extendResult(extend(tree,tgi,rmotion));
        temp += tree.temp_;
        iter++;

        if (extendResult != TRAPPED) {
            //Remember which motion was just added
            Motion *addedMotion(tgi.xmotion);

            //If reached, it means we used rstate directly, no need to copy again
            if (extendResult == ADVANCED) si_->copyState(rstate,tgi.xstate);

            //Attempt to connect trees
            otherTree.temp_ *= tempChangeFactor_;
            bool connected(connect(otherTree,tgi,rmotion));
            otherTree.temp_ /= tempChangeFactor_;

            Motion *startMotion(startTree ? tgi.xmotion : addedMotion);
            Motion *goalMotion(startTree ? addedMotion : tgi.xmotion);

            //If we connected the trees in a valid way (start and goal pair is valid)
            if (connected && goal->isStartGoalPairValid(startMotion->root,
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

                //Set the solution path
                PathGeometric *path(new PathGeometric(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i(mpath1.size() - 1); i >= 0; --i) {
                    path->append(mpath1[i]->state);
                }
                for (std::size_t i(0); i < mpath2.size(); ++i) {
                    path->append(mpath2[i]->state);
                }

                pdef_->addSolutionPath(base::PathPtr(path),false,0.0);
                solved = true;
                break;
            }
        }
    }

    //Clean up
    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)",getName().c_str(),
                tStart_->size()+tGoal_->size(),tStart_->size(),tGoal_->size());

    OMPL_INFORM("%s: Average temperature %f ",getName().c_str(),temp/double(iter));

    //std::cout << extendCount << " " << connectCount << " " << iter << std::endl;

    double costs(0);
    std::vector<Motion*> start;
    tStart_->list(start);
    for (unsigned int i = 0; i < start.size(); ++i) {
        if (start.at(i)->parent) {
            costs += opt_->motionCost(start.at(i)->parent->state,start.at(i)->state).value()
                    /si_->distance(start.at(i)->parent->state,start.at(i)->state);
        }
    }
    std::vector<Motion*> data;
    tGoal_->list(data);
    for (unsigned int i = 0; i < data.size(); ++i) {
        if (data.at(i)->parent) {
            costs += opt_->motionCost(data.at(i)->state,data.at(i)->parent->state).value()
                    /si_->distance(data.at(i)->parent->state,data.at(i)->state);
        }
    }
    //std::cout << "avg cost " << costs/double(start.size()+data.size()-2) << std::endl;

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}


void ompl::geometric::TRRTConnect::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_) tStart_->list(motions);

    for (std::size_t i(0); i < motions.size(); ++i) {
        if (!motions[i]->parent) {
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state,1));
        } else {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state,1),
                         base::PlannerDataVertex(motions[i]->state,1));
        }
    }

    motions.clear();
    if (tGoal_) tGoal_->list(motions);

    for (std::size_t i(0); i < motions.size(); ++i) {
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


bool ompl::geometric::TRRTConnect::transitionTest(base::Cost cost, double distance,
                                                  TreeData &tree, bool updateTemp) {
    //Difference in cost
    double slope(cost.value() / std::min(distance,maxDistance_));

    //The probability of acceptance of a new motion is defined by its cost.
    //Based on the Metropolis criterion.
    double transitionProbability(exp(-slope/(kConstant_*tree.temp_)));

    //Check if we can accept it
    if (rng_.uniform01() <= transitionProbability) {//State has succeed
        if (updateTemp) {
            ++tree.numStatesSucceed_;

            //Update temperature
            if (tree.numStatesSucceed_ > maxStatesSucceed_) {
                tree.temp_ /= tempChangeFactor_;
                //Prevent temperature from getting too small
                if (tree.temp_ < minTemperature_) tree.temp_ = minTemperature_;

                tree.numStatesSucceed_ = 0;
            }
        }
        return true;
    } else {//State has failed
        if (updateTemp) {
            ++tree.numStatesFailed_;

            //Update temperature
            if (tree.numStatesFailed_ > maxStatesFailed_) {
                tree.temp_ *= tempChangeFactor_;

                tree.numStatesFailed_ = 0;
            }
        }
        return false;
    }
}


bool ompl::geometric::TRRTConnect::minExpansionControl(double distance, TreeData &tree) {
    //Reject the node if it participates in the tree refinement
    //and the ratio between frontier nodes and non-frontier becomes small
    if (double(tree.frontierCount_)*frontierNodeRatio_ < double(tree.nonFrontierCount_)
            && distance <= frontierThreshold_) {
        //Reject this node as being too much refinement
        return false;
    }

    return true;
}


ompl::geometric::TRRTConnect::Motion *ompl::geometric::TRRTConnect::minCostNeighbor
(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion) {
    //Find nearby neighbors of the random motion - k-nearest TRRTConenect
    unsigned int k(std::ceil(k_rrg_*log(double(tree->size()+1))));
    std::vector<Motion*> nbh;
    tree->nearestK(rmotion,k,nbh);

    //Cache for distance computations
    //Our cost caches only increase in size, so they're only
    //resized if they can't fit the current neighborhood
    if (tree.costs_.size() < nbh.size()) {
        tree.costs_.resize(nbh.size());
        tree.sortedCostIndices_.resize(nbh.size());
    }

    //Finding the nearest valid neighbor with the lowest cost
    //By default, neighborhood states are sorted by cost, and collision checking
    //is performed in increasing order of cost
    if (delayCC_) {
        //Calculate all costs and distances
        for (std::size_t i(0); i < nbh.size(); ++i) {
            if (tree.start_) {
                tree.costs_[i] = opt_->motionCost(nbh[i]->state,rmotion->state);
            } else {
                tree.costs_[i] = opt_->motionCost(rmotion->state,nbh[i]->state);
            }
        }

        //Sort the nodes
        //We're using index-value pairs so that we can get at
        //original, unsorted indices
        for (std::size_t i(0); i < nbh.size(); ++i) {
            tree.sortedCostIndices_[i] = i;
        }
        std::sort(tree.sortedCostIndices_.begin(),tree.sortedCostIndices_.begin()+nbh.size(),
                  *tree.compareFn_);

        //Collision check until a valid motion is found
        bool validMotion;
        double d;
        base::State *dstate;
        for (std::vector<std::size_t>::const_iterator i(tree.sortedCostIndices_.begin());
             i != (tree.sortedCostIndices_.begin()+nbh.size()); ++i) {
            //Distance from near state to random state
            d = si_->distance(nbh[*i]->state,rmotion->state);

            //Check if random state is too far away
            if (d > maxDistance_) {
                si_->getStateSpace()->interpolate(nbh[*i]->state,rmotion->state,
                        maxDistance_/d,tgi.xstate);

                //Use the interpolated state as the new state
                dstate = tgi.xstate;
            } else {
                //Random state is close enough
                dstate = rmotion->state;
            }

            //If we are in the start tree, we just check the motion like we normally do.
            //If we are not in the goal tree, we need to check the motion in reverse,
            //but checkMotion() assumes the first state it receives as argument is valid,
            //so we check that one first.
            validMotion = (tree.start_ ? si_->checkMotion(nbh[*i]->state,dstate) :
                    (si_->getStateValidityChecker()->isValid(dstate) &&
                     si_->checkMotion(dstate,nbh[*i]->state)));
            if (validMotion) {
                //Found the collision-free motion with the lowest cost
                return nbh[*i];
            }
        }
    } else {//If not delayCC
        std::size_t nbhBest(nbh.size());
        //Find which one we connect the new state to
        bool validMotion;
        double d;
        base::State *dstate;
        for (std::size_t i(0); i < nbh.size(); ++i) {
            if (tree.start_) {
                tree.costs_[i] = opt_->motionCost(nbh[i]->state,rmotion->state);
            } else {
                tree.costs_[i] = opt_->motionCost(rmotion->state,nbh[i]->state);
            }
            if (nbhBest == nbh.size() || opt_->isCostBetterThan(tree.costs_[i],tree.costs_[nbhBest])) {
                //Distance from near state to random state
                d = si_->distance(nbh[i]->state,rmotion->state);

                //Check if random state is too far away
                if (d > maxDistance_) {
                    si_->getStateSpace()->interpolate(nbh[i]->state,rmotion->state,
                                                      maxDistance_/d,tgi.xstate);

                    //Use the interpolated state as the new state
                    dstate = tgi.xstate;
                } else {
                    //Random state is close enough
                    dstate = rmotion->state;
                }

                //If we are in the start tree, we just check the motion like we normally do.
                //If we are not in the goal tree, we need to check the motion in reverse,
                //but checkMotion() assumes the first state it receives as argument is valid,
                //so we check that one first.
                validMotion = (tree.start_ ? si_->checkMotion(nbh[i]->state,dstate) :
                                           (si_->getStateValidityChecker()->isValid(dstate) &&
                                            si_->checkMotion(dstate,nbh[i]->state)));
                if (validMotion) {
                    nbhBest = i;
                }
            }
        }
        //Found the collision-free motion with the lowest cost
        if (nbhBest < nbh.size()) return nbh[nbhBest];
    }

    //The motions with the neighbors are all in collision
    return NULL;
}
