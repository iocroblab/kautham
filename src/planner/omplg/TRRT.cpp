#include <kautham/planner/omplg/TRRT.h>
#include <kautham/planner/omplg/FOSOptimizationObjective.h>

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>

#include <limits>
#include <cfloat>


ompl::geometric::TRRT::TRRT(const base::SpaceInformationPtr &si) :
    base::Planner(si,"TRRT") {
    //Standard RRT Variables
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0; //set in setup()
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range",this,&TRRT::setRange,&TRRT::getRange,
                                  "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias",this,&TRRT::setGoalBias,&TRRT::getGoalBias,
                                  "0.:.05:1.");

    //TRRT Specific Variables
    frontierThreshold_ = 0.0;//Set in setup()
    kConstant_ = 0.0;//Set in setup()
    maxStatesFailed_ = 10;//Treshold for when to start increasing the temperatuer
    tempChangeFactor_ = 2.0;//How much to decrease or increase the temp each time
    minTemperature_ = 10e-10;//Lower limit of the temperature change
    initTemperature_ = 10e-6;//Where the temperature starts out
    frontierNodeRatio_ = 0.1;//1/10, or 1 nonfrontier for every 10 frontier

    Planner::declareParam<unsigned int>("max_states_failed",this,&TRRT::setMaxStatesFailed,
                                        &TRRT::getMaxStatesFailed,"1:1000");
    Planner::declareParam<double>("temp_change_factor",this,&TRRT::setTempChangeFactor,
                                  &TRRT::getTempChangeFactor,"0.:.1:10.");
    Planner::declareParam<double>("min_temperature",this,&TRRT::setMinTemperature,
                                  &TRRT::getMinTemperature);
    Planner::declareParam<double>("init_temperature",this,&TRRT::setInitTemperature,
                                  &TRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold",this,&TRRT::setFrontierThreshold,
                                  &TRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontierNodeRatio",this,&TRRT::setFrontierNodeRatio,
                                  &TRRT::getFrontierNodeRatio);
    Planner::declareParam<double>("k_constant",this,&TRRT::setKConstant,
                                  &TRRT::getKConstant);
}


ompl::geometric::TRRT::~TRRT() {
    freeMemory();
}


void ompl::geometric::TRRT::clear() {
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nearestNeighbors_) nearestNeighbors_->clear();
    lastGoalMotion_ = NULL;

    //Clear TRRT specific variables ----------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1;//Init to 1 to prevent division by zero error

    //If opt_ is of type FOSOptimizationObjective, there is a global zer-order box and
    //then this parameter will be used and initially set to false
    //If opt is of another type, this parameter is set to true to avoid its use
    stateInBoxZos_ = !dynamic_cast<ompl::base::FOSOptimizationObjective*>(opt_.get());
}


void ompl::geometric::TRRT::setup() {
    Planner::setup();
    tools::SelfConfig selfConfig(si_,getName());

    bool usingDefaultObjective;
    if (!pdef_->hasOptimizationObjective()) {
        OMPL_INFORM("%s: No optimization objective specified.",getName().c_str());
        usingDefaultObjective = true;
    } else {
        usingDefaultObjective = false;
    }

    if (usingDefaultObjective) {
        opt_.reset(new base::MechanicalWorkOptimizationObjective(si_));
        OMPL_INFORM("%s: Defaulting to optimizing path length.",getName().c_str());
    } else {
        opt_ = pdef_->getOptimizationObjective();
    }

    //Set maximum distance a new node can be from its nearest neighbor
    if (maxDistance_ < std::numeric_limits<double>::epsilon()) {
        selfConfig.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    //Set the threshold deciding if a new node is a frontier node or non-frontier node
    if (frontierThreshold_ < std::numeric_limits<double>::epsilon()) {
        frontierThreshold_ = si_->getMaximumExtent() * 0.01;
        OMPL_DEBUG("%s: Frontier threshold detected to be %lf",
                   getName().c_str(),frontierThreshold_);
    }

    //Autoconfigure the K constant
    if (kConstant_ < std::numeric_limits<double>::epsilon()) {
        //Find the average cost of states by sampling
        kConstant_ = opt_->averageStateCost(magic::TEST_STATE_COUNT).value();
        OMPL_DEBUG("%s: K constant detected to be %lf",getName().c_str(),kConstant_);
    }

    //Create the nearest neighbor function the first time setup is run
    if (!nearestNeighbors_) {
#if OMPL_VERSION_VALUE < 1001000 // 1.1.0
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>
                                         (si_->getStateSpace()));
#else
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
#endif
    }

    //Set the distance function
    nearestNeighbors_->setDistanceFunction(std::bind(&TRRT::distanceFunction,this,std::placeholders::_1,std::placeholders::_2));

    //Setup TRRT specific variables --------------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; //Init to 1 to prevent division by zero error

    //If opt_ is of type FOSOptimizationObjective, there is a global zer-order box and
    //then this parameter will be used and initially set to false
    //If opt is of another type, this parameter is set to true to avoid its use
    stateInBoxZos_ = !dynamic_cast<ompl::base::FOSOptimizationObjective*>(opt_.get());
}


void ompl::geometric::TRRT::freeMemory() {
    //Delete all motions, states and the nearest neighbors data structure
    if (nearestNeighbors_) {
        std::vector<Motion*> motions;
        nearestNeighbors_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i) {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}


ompl::base::PlannerStatus ompl::geometric::TRRT::solve
(const base::PlannerTerminationCondition &plannerTerminationCondition) {
    //Basic error checking
    checkValidity();

    //Goal information
    base::Goal *goal(pdef_->getGoal().get());
    base::GoalSampleableRegion *goalRegion(dynamic_cast<base::GoalSampleableRegion*>(goal));

    //Input States ---------------------------------------------------------------------

    //Loop through valid input states and add to tree
    while (const base::State *state = pis_.nextStart()) {
        //Allocate memory for a new start state motion based on the "space-information"-size
        Motion *motion(new Motion(si_));

        //Copy destination <= source
        si_->copyState(motion->state,state);

        //Add start motion to the tree
        nearestNeighbors_->add(motion);

        //Check if now the tree has a motion inside BoxZos
        if (!stateInBoxZos_) {
            stateInBoxZos_ = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                    boxZosDistance(motion->state) < DBL_EPSILON;
            if (stateInBoxZos_) temp_ = initTemperature_;
        }
    }

    //Check that input states exist
    if (nearestNeighbors_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!",getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    //Create state sampler if this is TRRT's first run
    if (!sampler_) sampler_ = si_->allocStateSampler();

    //Debug
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
                getName().c_str(),nearestNeighbors_->size());


    //Solver variables -----------------------------------------------------------------

    //The final solution
    Motion *solution(NULL);
    //The approximate solution, returned if no final solution found
    Motion *approxSolution(NULL);
    //Track the distance from goal to closest solution yet found
    double approxDifference(std::numeric_limits<double>::infinity());

    //Distance between states - the intial state and the interpolated state
    double randMotionDistance;
    double motionDistance;

    //Create random motion and a pointer (for optimization) to its state
    Motion *randMotion(new Motion(si_));
    Motion *nearMotion;

    //STATES
    //The random state
    base::State *randState = randMotion->state;
    //The new state that is generated between states *to* and *from*
    base::State *interpolatedState = si_->allocState();
    //The chosen state btw rand_state and interpolated_state
    base::State *newState;

    //Begin sampling -------------------------------------------------------------------
    while (!plannerTerminationCondition()) {
        //I.

        //Sample random state (with goal biasing probability)
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample()) {
            //Bias sample towards goal
            goalRegion->sampleGoal(randState);
        } else {
            //Uniformly Sample
            sampler_->sampleUniform(randState);
        }

        //II.

        //Find closest state in the tree
        nearMotion = nearestNeighbors_->nearest(randMotion);

        //III.

        //Distance from near state q_n to a random state
        randMotionDistance = si_->distance(nearMotion->state,randState);

        //Check if the rand_state is too far away
        if (randMotionDistance > maxDistance_) {
            //Computes the state that lies at time t in [0, 1] on
            //the segment that connects *from* state to *to* state.
            //The memory location of *state* is not required to be
            //different from the memory of either *from* or *to*.
            si_->getStateSpace()->interpolate(nearMotion->state,randState,maxDistance_ /
                                              randMotionDistance,interpolatedState);

            //Update the distance between near and new with the interpolated_state
            motionDistance = si_->distance(nearMotion->state,interpolatedState);

            //Use the interpolated state as the new state
            newState = interpolatedState;
        } else {
            //Random state is close enough
            newState = randState;

            //Copy the distance
            motionDistance = randMotionDistance;
        }

        //IV.
        //This stage integrates collision detections in the presence of obstacles
        if (!si_->checkMotion(nearMotion->state,newState)) continue; //Try a new sample


        //Minimum Expansion Control
        //A possible side effect may appear when the tree expansion toward
        //unexplored regions remains slow, and the new nodes contribute only
        //to refine already explored regions.
        if (!minExpansionControl(randMotionDistance)) continue; //Try a new sample

        base::Cost motionCost;
        if (stateInBoxZos_) {
            motionCost = opt_->motionCost(nearMotion->state,newState);
        } else {
            //opt_ is of type FOSOptimizationObjective,
            //there is no need to check the conversion
            motionCost = ((ompl::base::FOSOptimizationObjective*)opt_.get())->
                    preSolveMotionCost(nearMotion->state,newState);
        }


        //Only add this motion to the tree if the tranistion test accepts it
        if (!transitionTest(motionCost,motionDistance)) {
            continue; //give up on this one and try a new sample
        }

        //V.

        //Create a motion
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,newState);
        motion->parent = nearMotion; //link q_new to q_near as an edge

        //Add motion to data structure
        nearestNeighbors_->add(motion);

        //Check if now the tree has a motion inside BoxZos
        if (!stateInBoxZos_) {
            stateInBoxZos_ = motionCost.value() < DBL_EPSILON*motionDistance;
            if (stateInBoxZos_) temp_ = initTemperature_;
        }

        //VI.

        //Check if this motion is the goal
        double distToGoal = 0.0;
        bool isSatisfied = goal->isSatisfied(motion->state,&distToGoal);
        if (isSatisfied) {
            approxDifference = distToGoal; //the tolerated error distance btw state and goal
            solution = motion; //set the final solution
            break;
        }

        //Is this the closest solution we've found so far
        if (distToGoal < approxDifference) {
            approxDifference = distToGoal;
            approxSolution = motion;
        }

    } //end of solver sampling loop


    //Finish solution processing -------------------------------------------------------

    bool solved = false;
    bool approximate = false;

    //Substitute an empty solution with the best approximation
    if (!solution) {
        solution = approxSolution;
        approximate = true;
    }

    //Generate solution path for real/approx solution
    if (solution) {
        lastGoalMotion_ = solution;

        //construct the solution path
        std::vector<Motion*> mpath;
        while (solution) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        //set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1; i >= 0; --i) {
            path->append(mpath[i]->state);
        }

        pdef_->addSolutionPath(base::PathPtr(path),approximate,approxDifference);
        solved = true;
    }

    //Clean up -------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (randMotion->state) si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: Created %u states",getName().c_str(),nearestNeighbors_->size());

    return base::PlannerStatus(solved,approximate);
}


void ompl::geometric::TRRT::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nearestNeighbors_) nearestNeighbors_->list(motions);

    if (lastGoalMotion_) {
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    }

    for (unsigned int i = 0; i < motions.size(); ++i) {
        if (!motions[i]->parent) {
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        } else {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
        }
    }
}


bool ompl::geometric::TRRT::transitionTest(base::Cost cost, double distance) {
    //Difference in cost
    double costSlope = cost.value() / distance;

    //The probability of acceptance of a new configuration is defined by comparing
    //its motion cost. Based on the Metropolis criterion.
    double transitionProbability = 1.; //if cost_slope is <= 0, probabilty is 1

    //Only return at end
    bool result = false;

    //Calculate tranision probabilty
    if (costSlope > 0) transitionProbability = exp(-costSlope / (kConstant_ * temp_));

    //Check if we can accept it
    if (rng_.uniform01() <= transitionProbability) {
        if (temp_ > minTemperature_) {
            temp_ /= tempChangeFactor_;

            //Prevent temp_ from getting too small
            if (temp_ < minTemperature_) {
                temp_ = minTemperature_;
            }
        }

        numStatesFailed_ = 0;

        result = true;
    } else {
        //State has failed
        if (numStatesFailed_ >= maxStatesFailed_) {
            temp_ *= tempChangeFactor_;
            numStatesFailed_ = 0;
        } else {
            ++numStatesFailed_;
        }
    }

    return result;
}


bool ompl::geometric::TRRT::minExpansionControl(double randMotionDistance) {
    //Decide to accept or not
    if (randMotionDistance > frontierThreshold_) {
        //Participates in the tree expansion
        ++frontierCount_;

        return true;
    } else {
        //Participates in the tree refinement

        //Check our ratio first before accepting it
        if ((double)nonfrontierCount_ / (double)frontierCount_ > frontierNodeRatio_) {
            //Increment so that the temperature rises faster
            ++numStatesFailed_;

            //Reject this node as being too much refinement
            return false;
        } else {
            ++nonfrontierCount_;

            return true;
        }
    }
}
