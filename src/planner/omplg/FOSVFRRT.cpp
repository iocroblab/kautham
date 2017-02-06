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

/* Authors: Nestor Garcia Hidalgo */


#include <kautham/planner/omplg/FOSVFRRT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


namespace ompl
{
    namespace magic
    {
        /// Number of sampler to determine mean vector field norm in \ref gVFRRT
        static const unsigned int FOSVFRRT_MEAN_NORM_SAMPLES = 1000;
    }
}

ompl::geometric::FOSVFRRT::FOSVFRRT(const base::SpaceInformationPtr &si,
                                    omplplanner::omplPlanner *pl,
                                    SynergyTree *st, double exploration,
                                    double initial_lambda, unsigned int update_freq)
    : RRT(si), st_(st), efficientCount_(0), inefficientCount_(0), explorationInefficiency_(0.),
      explorationSetting_(exploration), initialLambda_(initial_lambda),
      nth_step_(update_freq), step_(0), meanNorm_(0.), vfdim_(0), originalExtend_(true), inefficiencyThreshold_(1.) {
    setName("FOSVFRRT");
    maxDistance_ = si->getStateValidityCheckingResolution();

    Planner::declareParam<double>("exploration", this, &FOSVFRRT::setExploration,
                                  &FOSVFRRT::getExploration, "0.:1.");
    Planner::declareParam<double>("initial_lambda", this, &FOSVFRRT::setInitialLambda,
                                  &FOSVFRRT::getInitialLambda, "0.:1.e100");
    Planner::declareParam<double>("update_freq", this, &FOSVFRRT::setUpdateFrequency,
                                  &FOSVFRRT::getUpdateFrequency, "0.:1.e100");
    Planner::declareParam<bool>("use_original_extend", this, &FOSVFRRT::setOriginalExtend,
                                &FOSVFRRT::getOriginalExtend);


    ///dofs of base orientation are supposed to be disabled for all the robots!

    //Get the total number of joints in the workspace
    unsigned int totalNumJoints = 0;
    for (unsigned int r = 0; r < pl->wkSpace()->getNumRobots(); ++r) {
        if (pl->wkSpace()->getRobot(r)->isSE3Enabled()) totalNumJoints += 3;
        totalNumJoints += pl->wkSpace()->getRobot(r)->getNumJoints();
    }

    //For each robot
    for (unsigned int r = 0; r < pl->wkSpace()->getNumRobots(); ++r) {
        //Get robot
        Robot *robot = pl->wkSpace()->getRobot(r);

        //SE3
        if (robot->isSE3Enabled()) {
            for (unsigned int i = 0; i < 3; ++i) {
                //Check if the joint is actuated, i.e. there exists a component different from 0
                //in the i-th row of the robot map matrix
                bool actuated = false;
                for (unsigned int j = 0; j < pl->wkSpace()->getNumRobControls() && !actuated; ++j) {
                    if (fabs(robot->getMapMatrix()[i][j]) > FLT_EPSILON) {//Different from 0
                        actuated = true;
                        //Add joint
                        robotJoint[r].first.insert(i);
                        vfdim_++;
                    }
                }
            }
        }

        //Rn
        for (unsigned int i = 0; i < robot->getNumJoints(); ++i) {
            //Check if the joint is actuated, i.e. there exists a component different from 0
            //in the (i+6)-th row of the robot map matrix (first 6 rows are related with the SE3, not used)
            bool actuated = false;
            for (unsigned int j = 0; j < pl->wkSpace()->getNumRobControls() && !actuated; ++j) {
                if (fabs(robot->getMapMatrix()[i+6][j]) > FLT_EPSILON) {//Different from 0
                    actuated = true;
                    //Add joint
                    robotJoint[r].second.insert(i);
                    vfdim_++;
                }
            }
        }
    }

    assert(vfdim_ > 0);
}


void ompl::geometric::FOSVFRRT::omplState2armaVec(const ompl::base::State *s,
                                                  arma::vec &q) const {
    q.resize(vfdim_);
    unsigned int k = 0;

    //Convert state to scoped state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> ss(si_->getStateSpace());
    ss = *s;

    //Extract from the ss scoped state the values related to the robot
    ///dofs of base orientation are supposed to be disabled for all the robots!
    for (auto it = robotJoint.begin(); it != robotJoint.end(); ++it) {
        //SE3
        if (!it->second.first.empty()) {
            ompl::base::ScopedState<ompl::base::SE3StateSpace> ssRobot
                    (si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->first)->as<ompl::base::CompoundStateSpace>()->getSubspace(0));
            ss >> ssRobot;

            for (auto j = it->second.first.begin(); j != it->second.first.end(); ++j) {
                if (*j == 0) {
                    q[k] = ssRobot->getX();
                } else if (*j == 1) {
                    q[k] = ssRobot->getY();
                } else {
                    q[k] = ssRobot->getZ();
                }
                k++;
            }
        }

        //Rn
        if (!it->second.second.empty()) {
            ompl::base::ScopedState<Kautham::omplplanner::weigthedRealVectorStateSpace> ssRobot
                    (si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->first)->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->second.first.empty()? 0 : 1));
            ss >> ssRobot;

            for (auto j = it->second.second.begin(); j != it->second.second.end(); ++j) {
                q[k] = ssRobot->values[*j];
                k++;
            }
        }
    }
}


void ompl::geometric::FOSVFRRT::armaVec2omplState(const arma::vec &q,
                                                  ompl::base::State *s) const {
    assert(q.n_elem == vfdim_);
    unsigned int k = 0;

    //Convert state to scoped state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> ss(si_->getStateSpace());
    ss = *s;

    //Extract from the ss scoped state the values related to the robot
    ///dofs of base orientation are supposed to be disabled for all the robots!
    for (auto it = robotJoint.begin(); it != robotJoint.end(); ++it) {
        //SE3
        if (!it->second.first.empty()) {
            ompl::base::ScopedState<ompl::base::SE3StateSpace> ssRobot
                    (si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->first)->as<ompl::base::CompoundStateSpace>()->getSubspace(0));
            ss >> ssRobot;

            for (auto j = it->second.first.begin(); j != it->second.first.end(); ++j) {
                if (*j == 0) {
                    ssRobot->setX(q[k]);
                } else if (*j == 1) {
                    ssRobot->setY(q[k]);
                } else {
                    ssRobot->setZ(q[k]);
                }
                k++;
            }

            ss << ssRobot;
        }

        //Rn
        if (!it->second.second.empty()) {
            ompl::base::ScopedState<Kautham::omplplanner::weigthedRealVectorStateSpace> ssRobot
                    (si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->first)->as<ompl::base::CompoundStateSpace>()->
                     getSubspace(it->second.first.empty()? 0 : 1));
            ss >> ssRobot;

            for (auto j = it->second.second.begin(); j != it->second.second.end(); ++j) {
                ssRobot->values[*j] = q[k];
                k++;
            }

            ss << ssRobot;
        }
    }

    si_->getStateSpace()->copyState(s,ss.get());
}


ompl::geometric::FOSVFRRT::~FOSVFRRT()
{
}


void ompl::geometric::FOSVFRRT::clear()
{
    RRT::clear();
    efficientCount_ = 0;
    inefficientCount_ = 0;
    explorationInefficiency_ = 0.;
    step_ = 0;
}


void ompl::geometric::FOSVFRRT::setup()
{
    RRT::setup();
    inefficiencyThreshold_ = si_->getStateValidityCheckingResolution()/maxDistance_;
}


double ompl::geometric::FOSVFRRT::determineMeanNorm()
{

    ompl::base::State *rstate = si_->allocState();
    double sum = 0.;
    for (unsigned int i = 0; i < magic::FOSVFRRT_MEAN_NORM_SAMPLES; i++)
    {
        sampler_->sampleUniform(rstate);
        arma::vec q;
        omplState2armaVec(rstate,q);

        sum += arma::norm(st_->vectorField(q));
    }
    si_->freeState(rstate);
    return sum / magic::FOSVFRRT_MEAN_NORM_SAMPLES;
}


arma::vec ompl::geometric::FOSVFRRT::getNewDirection(const base::State *snear, const base::State *srand)
{
    // Set vrand to be the normalized vector from qnear to qrand
    arma::vec qrand;
    omplState2armaVec(srand,qrand);
    arma::vec qnear;
    omplState2armaVec(snear,qnear);
    arma::vec vrand = arma::normalise(qrand-qnear);

    // Get the vector at qnear, and normalize
    arma::vec vfield = st_->vectorField(qnear);
    const double lambdaScale = arma::norm(vfield);

    // In the case where there is no vector field present, vfield.norm() == 0,
    // return the direction of the random state.
    if (lambdaScale < std::numeric_limits<float>::epsilon()) return vrand;
    vfield /= lambdaScale;
    // Sample a weight from the distribution
    const double omega = biasedSampling(vrand, vfield, lambdaScale);
    // Determine updated direction
    return computeAlphaBeta(omega, vrand, vfield);
}

double ompl::geometric::FOSVFRRT::biasedSampling(const arma::vec &vrand,
                                                 const arma::vec &vfield, double lambdaScale)
{
    double sigma = .25 * pow(arma::norm(vrand-vfield), 2.);
    updateGain();
    double scaledLambda = lambda_ * lambdaScale / meanNorm_;
    double phi = scaledLambda / (1. - std::exp(-2. * scaledLambda));
    double z = - std::log(1. - sigma * scaledLambda / phi) / scaledLambda;
    return std::sqrt(2. * z);
}

void ompl::geometric::FOSVFRRT::updateGain()
{
    if (step_ == nth_step_)
    {
        lambda_ = lambda_ * (1 - explorationInefficiency_ + explorationSetting_);
        efficientCount_ = inefficientCount_ = 0;
        explorationInefficiency_ = 0;
        step_ = 0;
    }
    else
        step_++;
}

arma::vec ompl::geometric::FOSVFRRT::computeAlphaBeta(
        double omega, const arma::vec &vrand, const arma::vec &vfield)
{
    double w2 = omega * omega;
    double c = arma::dot(vfield,vrand);
    double cc_1 = c * c - 1.;
    double root = std::sqrt(cc_1 * w2 * (w2 - 4.));
    double beta = -root / (2. * cc_1);
    double sign = (beta < 0.) ? -1. : 1.;
    beta *= sign;
    double alpha = (sign * c * root + cc_1 * (2. - w2)) / (2. * cc_1);
    return alpha * vfield + beta * vrand;
}

ompl::geometric::FOSVFRRT::Motion *ompl::geometric::FOSVFRRT::extendTree
(Motion *m, base::State* rstate, bool biased, const arma::vec &v) {
    double d = si_->distance(m->state,rstate);

    base::State* newState = si_->allocState();
    if (!originalExtend_ && biased && d < maxDistance_) {
        si_->copyState(newState,rstate);
    } else {
        d = maxDistance_;
        si_->copyState(newState,m->state);
        arma::vec qnew;
        omplState2armaVec(newState,qnew);
        armaVec2omplState(qnew + d*v,newState);
    }

    if (si_->checkMotion(m->state,newState)) {
        Motion *motion = new Motion(si_);
        motion->state = newState;
        motion->parent = m;
        updateExplorationEfficiency(motion);
        nn_->add(motion);
        return motion;
    } else {
        si_->freeState(newState);
        inefficientCount_++;
        return nullptr;
    }
}

void ompl::geometric::FOSVFRRT::updateExplorationEfficiency(Motion *m)
{
    Motion *near = nn_->nearest(m);
    if (distanceFunction(m, near) < maxDistance_*inefficiencyThreshold_)
        inefficientCount_++;
    else
        efficientCount_++;
    explorationInefficiency_ = inefficientCount_ / (double)(efficientCount_ + inefficientCount_);
}

ompl::base::PlannerStatus ompl::geometric::FOSVFRRT::solve(const base::PlannerTerminationCondition &ptc)
{
//    initialLambda_ = rng_.uniformReal(1e-3,1e5);
//    explorationSetting_ = rng_.uniform01();
//    inefficiencyThreshold_ = rng_.uniform01();
//    nth_step_ = 1000-rng_.halfNormalInt(0,999);
    OMPL_INFORM("%s: Initial lambda %f, Exploration inefficiency %f, Inefficiency threshold %f, Update frequency %i",
                getName().c_str(),initialLambda_,explorationSetting_,inefficiencyThreshold_,nth_step_);

    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (!sampler_) sampler_ = si_->allocStateSampler();

    meanNorm_ = determineMeanNorm();
        OMPL_INFORM("%s: Mean norm %f", getName().c_str(), meanNorm_);
    lambda_ = initialLambda_;

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = nullptr;
    Motion *approxsol = nullptr;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        // Sample random state (with goal biasing)
        bool biased;
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
            goal_s->sampleGoal(rstate);
            biased = true;
        } else {
            sampler_->sampleUniform(rstate);
            biased = false;
        }

        // Find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        // Modify direction based on vector field before extending
        Motion *motion = extendTree(nmotion, rstate, biased, getNewDirection(nmotion->state,rstate));
        if (!motion) continue;

        // Check if we can connect to the goal
        double dist = 0;
        bool sat = goal->isSatisfied(motion->state, &dist);
        if (sat) {
            approxdif = dist;
            solution = motion;
            break;
        }
        if (dist < approxdif) {
            approxdif = dist;
            approxsol = motion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr) {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr) {
        lastGoalMotion_ = solution;

        // Construct the solution path
        std::vector<Motion*> mpath;
        while (solution != nullptr) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
            path->append(mpath[i]->state);
        }
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, name_);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state) si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    OMPL_INFORM("%s: Final lambda %f", getName().c_str(),lambda_);


    return base::PlannerStatus(solved, approximate);
}
