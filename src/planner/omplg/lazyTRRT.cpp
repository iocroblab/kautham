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


#include "lazyTRRT.h"

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

og::LazyTRRT::LazyTRRT(const base::SpaceInformationPtr &si) : TRRT(si) {
    setName("LazyTRRT");
}

ob::PlannerStatus og::LazyTRRT::solve(const base::PlannerTerminationCondition &ptc) {
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
    // the approximate solution, returned if no final solution found
    Motion *approxSolution = NULL;
    // track the distance from goal to closest solution yet found
    double approxDifference = std::numeric_limits<double>::infinity();

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
    while (!ptc()) {
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
        // this stage integrates collision detections in the presence of obstacles and checks for collisions

        if (!si_->checkMotion(nearMotion->state,newState)) {
            continue; // try a new sample
        }


        // Minimum Expansion Control
        // A possible side effect may appear when the tree expansion toward unexplored regions remains slow, and the
        // new nodes contribute only to refine already explored regions.
        if (!minExpansionControl(randMotionDistance)) {
            continue; // give up on this one and try a new sample
        }

        base::Cost childCost = opt_->stateCost(newState);

        // Only add this motion to the tree if the tranistion test accepts it
        if (!transitionTest(childCost.v,nearMotion->cost.v,motionDistance)) {
            continue; // give up on this one and try a new sample
        }

        // V.

        // Create a motion
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,newState);
        motion->parent = nearMotion; // link q_new to q_near as an edge
        motion->cost = childCost;

        // Add motion to data structure
        nearestNeighbors_->add(motion);

        // VI.

        // Check if this motion is the goal
        double distToGoal = 0.;
        if (goal->isSatisfied(motion->state,&distToGoal)) {
            approxDifference = distToGoal; // the tolerated error distance btw state and goal
            solution = motion; // set the final solution
            break;
        }

        // Is this the closest solution we've found so far
        if (distToGoal < approxDifference) {
            approxDifference = distToGoal;
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
    }

    // Clean up ---------------------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (randMotion->state) si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: Created %u states",getName().c_str(),nearestNeighbors_->size());

    return base::PlannerStatus(solved,approximate);
}
