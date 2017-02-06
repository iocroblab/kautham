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


#include <kautham/planner/omplg/lazyTRRT.h>

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>


ompl::geometric::LazyTRRT::LazyTRRT(const base::SpaceInformationPtr &si) : base::Planner(si,"LazyTRRT") {
    // Standard RRT Variables
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0; // set in setup()
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range",this,&LazyTRRT::setRange,&LazyTRRT::getRange,"0.:1.:10000.");
    Planner::declareParam<double>("goal_bias",this,&LazyTRRT::setGoalBias,&LazyTRRT::getGoalBias,"0.:.05:1.");

    // TRRT Specific Variables
    frontierThreshold_ = 0.0; // set in setup()
    kConstant_ = 0.0; // set in setup()
    maxStatesFailed_ = 10; // threshold for when to start increasing the temperatuer
    tempChangeFactor_ = 2.0; // how much to decrease or increase the temp each time
    minTemperature_ = 10e-10; // lower limit of the temperature change
    initTemperature_ = 10e-6; // where the temperature starts out
    frontierNodeRatio_ = 0.1; // 1/10, or 1 nonfrontier for every 10 frontier

    //Complete lazy collision checking by default
    minCollThr_ = DBL_MAX-DBL_EPSILON;
    maxCollThr_ = DBL_MAX;

    Planner::declareParam<unsigned int>("max_states_failed",this,&LazyTRRT::setMaxStatesFailed,&LazyTRRT::getMaxStatesFailed,"1:1000");
    Planner::declareParam<double>("temp_change_factor",this,&LazyTRRT::setTempChangeFactor,&LazyTRRT::getTempChangeFactor,"0.:.1:10.");
    Planner::declareParam<double>("min_temperature",this,&LazyTRRT::setMinTemperature,&LazyTRRT::getMinTemperature);
    Planner::declareParam<double>("init_temperature",this,&LazyTRRT::setInitTemperature,&LazyTRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold",this,&LazyTRRT::setFrontierThreshold,&LazyTRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontier_nodes_ratio",this,&LazyTRRT::setFrontierNodeRatio,&LazyTRRT::getFrontierNodeRatio);
    Planner::declareParam<double>("k_constant",this,&LazyTRRT::setKConstant,&LazyTRRT::getKConstant);
    Planner::declareParam<double>("min_coll_thr",this,&LazyTRRT::setMinCollisionThreshold,&LazyTRRT::getMinCollisionThreshold);
    Planner::declareParam<double>("max_coll_thr",this,&LazyTRRT::setMaxCollisionThreshold,&LazyTRRT::getMaxCollisionThreshold);
}

ompl::geometric::LazyTRRT::~LazyTRRT() {
    freeMemory();
}


void ompl::geometric::LazyTRRT::clear() {
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nearestNeighbors_)
        nearestNeighbors_->clear();
    lastGoalMotion_ = NULL;

    // Clear TRRT specific variables ---------------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; // init to 1 to prevent division by zero error
}


void ompl::geometric::LazyTRRT::setup() {
    Planner::setup();
    tools::SelfConfig selfConfig(si_,getName());

    // Set optimization objective
    if (pdef_->hasOptimizationObjective()) {
        opt_ = pdef_->getOptimizationObjective();
    } else {        
        opt_.reset(new base::MechanicalWorkOptimizationObjective(si_));
        OMPL_INFORM("%s: No optimization objective specified.",getName().c_str());
        OMPL_INFORM("%s: Defaulting to optimizing path length.",getName().c_str());
    }

    // Set maximum distance a new node can be from its nearest neighbor
    if (maxDistance_ < std::numeric_limits<double>::epsilon()) {
        selfConfig.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    // Set the threshold that decides if a new node is a frontier node or non-frontier node
    if (frontierThreshold_ < std::numeric_limits<double>::epsilon()) {
        frontierThreshold_ = si_->getMaximumExtent() * 0.01;
        OMPL_DEBUG("%s: Frontier threshold detected to be %lf",getName().c_str(),frontierThreshold_);
    }

    // Autoconfigure the K constant
    if (kConstant_ < std::numeric_limits<double>::epsilon()) {
        // Find the average cost of states by sampling
        double averageCost = opt_->averageStateCost(magic::TEST_STATE_COUNT).value();
        kConstant_ = averageCost;
        OMPL_DEBUG("%s: K constant detected to be %lf",getName().c_str(),kConstant_);
    }

    // Create the nearest neighbor function the first time setup is run
    if (!nearestNeighbors_) {
#if OMPL_VERSION_VALUE < 1001000 // 1.1.0
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
#else
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
#endif
    }

    // Set the distance function
    nearestNeighbors_->setDistanceFunction(std::bind(&LazyTRRT::distanceFunction,this,std::placeholders::_1,std::placeholders::_2));

    // Setup TRRT specific variables ---------------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; // init to 1 to prevent division by zero error
}


void ompl::geometric::LazyTRRT::freeMemory() {
    // Delete all motions, states and the nearest neighbors data structure
    if (nearestNeighbors_) {
        std::vector<Motion*> motions;
        nearestNeighbors_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i) {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}


ompl::base::PlannerStatus ompl::geometric::LazyTRRT::solve(const base::PlannerTerminationCondition &ptc) {
    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goalRegion = dynamic_cast<base::GoalSampleableRegion*>(goal);

    // Input States ---------------------------------------------------------------------------------

    // Loop through valid input states and add to tree
    while (const base::State *state = pis_.nextStart()) {
        // Allocate memory for a new start state motion based on the "space-information"-size
        Motion *motion = new Motion(si_);

        // Copy destination <= source
        si_->copyState(motion->state,state);

        // Set cost for this start state
        motion->cost = opt_->stateCost(motion->state);

        //!
        motion->valid = true;

        // Add start motion to the tree
        nearestNeighbors_->add(motion);
    }

    // Check that input states exist
    if (nearestNeighbors_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!",getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Create state sampler if this is TRRT's first run
    if (!sampler_) sampler_ = si_->allocStateSampler();

    // Debug
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
                getName().c_str(),nearestNeighbors_->size());


    // Solver variables ------------------------------------------------------------------------------------

    // the final solution
    Motion *solution = NULL;

    //!
    double distsol = -1.;
    unsigned int pathChecks = 0;
    unsigned int validMotions = 0;
    unsigned int invalidMotions = 0;
    /*
    // the approximate solution, returned if no final solution found
    Motion *approxSolution = NULL;
    // track the distance from goal to closest solution yet found
    double approxDifference = std::numeric_limits<double>::infinity();
    */

    // distance between states - the intial state and the interpolated state (may be the same)
    double randMotionDistance;
    double motionDistance;

    // Create random motion and a pointer (for optimization) to its state
    Motion *randMotion = new Motion(si_);
    Motion *nearMotion;

    // STATES
    // The random state
    base::State *randState = randMotion->state;
    // The new state that is generated between states *to* and *from*
    base::State *interpolatedState = si_->allocState(); // Allocates "space information"-sized memory for a state
    // The chosen state btw rand_state and interpolated_state
    base::State *newState;

    // Begin sampling --------------------------------------------------------------------------------------
    //!
    bool solutionFound = false;
    while (!ptc() && !solutionFound) {
        //while (!ptc()) {
        // I.

        // Sample random state (with goal biasing probability)
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample()) {
            // Bias sample towards goal
            goalRegion->sampleGoal(randState);
        } else {
            // Uniformly Sample
            sampler_->sampleUniform(randState);
        }

        // II.

        // Find closest state in the tree
        nearMotion = nearestNeighbors_->nearest(randMotion);

        //!
        assert(nearMotion != randMotion);

        // III.

        // Distance from near state q_n to a random state
        randMotionDistance = si_->distance(nearMotion->state,randState);

        // Check if the rand_state is too far away
        if (randMotionDistance > maxDistance_) {
            // Computes the state that lies at time t in [0, 1] on the segment that connects *from* state to *to* state.
            // The memory location of *state* is not required to be different from the memory of either *from* or *to*.
            si_->getStateSpace()->interpolate(nearMotion->state,randState,
                                              maxDistance_ / randMotionDistance,interpolatedState);

            // Update the distance between near and new with the interpolated_state
            motionDistance = si_->distance(nearMotion->state,interpolatedState);

            // Use the interpolated state as the new state
            newState = interpolatedState;
        } else {
            // Random state is close enough
            newState = randState;

            // Copy the distance
            motionDistance = randMotionDistance;
        }

        // IV.
        // this stage integrates collision detections in the presence of ompl::basestacles and checks for collisions

        //!
        if (rng_.uniform01() < (nearMotion->cost.value()-minCollThr_)/(maxCollThr_-minCollThr_)) {
            if (!si_->checkMotion(nearMotion->state,newState)) {
                invalidMotions++;
                continue; // try a new sample
            } else {
                validMotions++;
                nearMotion->valid = true;
            }
        }
        //if (!si_->checkMotion(nearMotion->state,newState)) continue; // try a new sample

        // Minimum Expansion Control
        // A possible side effect may appear when the tree expansion toward unexplored regions remains slow, and the
        // new nodes contribute only to refine already explored regions.
        if (!minExpansionControl(randMotionDistance)) {
            continue; // give up on this one and try a new sample
        }

        base::Cost childCost = opt_->stateCost(newState);

        // Only add this motion to the tree if the tranistion test accepts it
        if (!transitionTest(childCost.value(),nearMotion->cost.value(),motionDistance)) {
            continue; // give up on this one and try a new sample
        }

        // V.

        // Create a motion
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,newState);
        motion->parent = nearMotion; // link q_new to q_near as an edge
        motion->cost = childCost;

        //!
        nearMotion->children.push_back(motion);

        // Add motion to data structure
        nearestNeighbors_->add(motion);

        // VI.

        // Check if this motion is the goal
        double distToGoal = 0.;
        if (goal->isSatisfied(motion->state,&distToGoal)) {
            //!
            pathChecks++;
            distsol = distToGoal;
            solution = motion;
            solutionFound = true;
            lastGoalMotion_ = solution;

            // Check that the solution is valid:
            // construct the solution path
            std::vector<Motion*> mpath;
            while (solution) {
                mpath.push_back(solution);
                solution = solution->parent;
            }

            // Check each segment along the path for validity
            for (int i = mpath.size() - 1 ; i >= 0 && solutionFound; --i) {
                if (!mpath[i]->valid) {
                    if (si_->checkMotion(mpath[i]->parent->state,mpath[i]->state)) {
                        mpath[i]->valid = true;
                    } else {
                        removeMotion(mpath[i]);
                        solutionFound = false;
                        lastGoalMotion_ = NULL;
                    }
                }
            }

            if (solutionFound) {
                // Set the solution path
                PathGeometric *path = new PathGeometric(si_);
                for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
                    path->append(mpath[i]->state);
                }

                pdef_->addSolutionPath(base::PathPtr(path),false,distsol);
            }
        }
    }

    /*approxDifference = distTompl::geometricoal; // the tolerated error distance btw state and goal
            solution = motion; // set the final solution
            break;
        }

        // Is this the closest solution we've found so far
        if (distTompl::geometricoal < approxDifference) {
            approxDifference = distTompl::geometricoal;
            approxSolution = motion;
        }

    } // end of solver sampling loop


    // Finish solution processing --------------------------------------------------------------------

    bool solved = false;
    bool approximate = false;

    // Substitute an empty solution with the best approximation
    if (!solution) {
        solution = approxSolution;
        approximate = true;
    }

    // Generate solution path for real/approx solution
    if (solution) {
        lastGoalMotion_ = solution;

        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
            path->append(mpath[i]->state);
        }

        pdef_->addSolutionPath(base::PathPtr(path),approximate,approxDifference);
        solved = true;
    }*/

    // Clean up ---------------------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (randMotion->state) si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: Created %u states",getName().c_str(),nearestNeighbors_->size());

    //!
    OMPL_INFORM("%s: Path checked %u times",getName().c_str(),pathChecks);
    OMPL_INFORM("%s: %u invalid motions of %u checked motions (%f)",getName().c_str(),invalidMotions,
                validMotions+invalidMotions,double(invalidMotions)/double(std::max(1u,validMotions+invalidMotions)));

    return solutionFound ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
    //return base::PlannerStatus(solved,approximate);
}


void ompl::geometric::LazyTRRT::removeMotion(Motion *motion) {
    nearestNeighbors_->remove(motion);

    // Remove self from parent list
    if (motion->parent) {
        for (unsigned int i = 0; i < motion->parent->children.size(); ++i) {
            if (motion->parent->children[i] == motion) {
                motion->parent->children.erase(motion->parent->children.begin() + i);
                break;
            }
        }
    }

    // Remove children
    for (unsigned int i = 0; i < motion->children.size(); ++i) {
        motion->children[i]->parent = NULL;
        removeMotion(motion->children[i]);
    }

    if (motion->state) si_->freeState(motion->state);
    delete motion;
}


void ompl::geometric::LazyTRRT::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nearestNeighbors_) nearestNeighbors_->list(motions);

    if (lastGoalMotion_) data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0; i < motions.size(); ++i) {
        if (!motions[i]->parent) {
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        } else {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
        }
    }
}


bool ompl::geometric::LazyTRRT::transitionTest(double childCost, double parentCost, double distance) {
    // Always accept if new state has same or lower cost than old state
    if (childCost <= parentCost) return true;

    // Difference in cost
    double costSlope = (childCost - parentCost) / distance;

    // The probability of acceptance of a new configuration is defined by comparing its cost c_j
    // relatively to the cost c_i of its parent in the tree. Based on the Metropolis criterion.
    double transitionProbability = 1.; // if cost_slope is <= 0, prompl::baseabilty is 1

    // Only return at end
    bool result = false;

    // Calculate tranision prompl::baseabilty
    if (costSlope > 0) {
        transitionProbability = exp(-costSlope / (kConstant_ * temp_));
    }

    // Check if we can accept it
    if (rng_.uniform01() <= transitionProbability) {
        if (temp_ > minTemperature_) {
            temp_ /= tempChangeFactor_;

            // Prevent temp_ from getting too small
            if (temp_ < minTemperature_) {
                temp_ = minTemperature_;
            }
        }

        numStatesFailed_ = 0;

        result = true;
    } else {
        // State has failed
        if (numStatesFailed_ >= maxStatesFailed_) {
            temp_ *= tempChangeFactor_;
            numStatesFailed_ = 0;
        } else {
            ++numStatesFailed_;
        }

    }

    return result;
}


bool ompl::geometric::LazyTRRT::minExpansionControl(double randMotionDistance) {
    // Decide to accept or not
    if (randMotionDistance > frontierThreshold_) {
        // participates in the tree expansion
        ++frontierCount_;

        return true;
    } else {
        // participates in the tree refinement

        // check our ratio first before accepting it
        if (double(nonfrontierCount_) > double(frontierCount_)*frontierNodeRatio_) {
            // Increment so that the temperature rises faster
            ++numStatesFailed_;

            // reject this node as being too much refinement
            return false;
        } else {
            ++nonfrontierCount_;
            return true;
        }
    }
}
