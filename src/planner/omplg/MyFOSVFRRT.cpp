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


#include <kautham/planner/omplg/MyFOSVFRRT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


#define LAMBDA_MIN 1.e-3
#define LAMBDA_MAX 1.e+5

#define th 0.3
#define p 0.3

namespace ompl
{
    namespace magic
    {
        /// Number of sampler to determine mean vector field norm in \ref gVFRRT
        static const unsigned int MyFOSVFRRT_MEAN_NORM_SAMPLES = 1000;
    }
}

ompl::geometric::MyFOSVFRRT::MyFOSVFRRT(const base::SpaceInformationPtr &si,
                                    omplplanner::omplPlanner *pl, SynergyTree *st)
    : RRT(si), st_(st), meanNorm_(0.), vfdim_(0) {
    setName("MyFOSVFRRT");
    maxDistance_ = si->getStateValidityCheckingResolution();

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


void ompl::geometric::MyFOSVFRRT::omplState2armaVec(const ompl::base::State *s,
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


void ompl::geometric::MyFOSVFRRT::armaVec2omplState(const arma::vec &q,
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


ompl::geometric::MyFOSVFRRT::~MyFOSVFRRT()
{
}


void ompl::geometric::MyFOSVFRRT::clear()
{
    RRT::clear();
}


void ompl::geometric::MyFOSVFRRT::setup()
{
    RRT::setup();
}


double ompl::geometric::MyFOSVFRRT::determineMeanNorm()
{

    ompl::base::State *rstate = si_->allocState();
    double sum = 0.;
    for (unsigned int i = 0; i < magic::MyFOSVFRRT_MEAN_NORM_SAMPLES; i++)
    {
        sampler_->sampleUniform(rstate);
        arma::vec q;
        omplState2armaVec(rstate,q);

        sum += arma::norm(st_->vectorField(q));
    }
    si_->freeState(rstate);
    return sum / magic::MyFOSVFRRT_MEAN_NORM_SAMPLES;
}


arma::vec ompl::geometric::MyFOSVFRRT::getNewDirection(const base::State *snear, const base::State *srand)
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
    if (lambdaScale < std::numeric_limits<double>::epsilon()) return vrand;
    vfield /= lambdaScale;
    // Determine updated direction
    double c = arma::dot(vfield,vrand);
    if (1.-fabs(c) <= 2.*std::numeric_limits<double>::epsilon()) return vrand;

    double scaledLambda = lambda_ * lambdaScale / meanNorm_;
    double w2 = -2./scaledLambda*log(1.-0.5*(1.-c)*(1.-exp(-2.*scaledLambda)));
    double wfield = (1.-0.5*w2)-0.5*c*sqrt(w2*(4.-w2)/(1.-c*c));
    double wrand = wfield-1.;
    wrand = sqrt(wrand*wrand+wfield*w2);
    return wrand*vrand + wfield*vfield;
}


ompl::geometric::MyFOSVFRRT::Motion *ompl::geometric::MyFOSVFRRT::extendTree
(Motion *m, base::State* rstate, bool biased, const arma::vec &v) {
    double d = si_->distance(m->state,rstate);

    base::State* newState = si_->allocState();
    if (biased && d < maxDistance_) {
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
        double d = distanceFunction(motion,nn_->nearest(motion))/maxDistance_;
        lambda_ = std::min(std::max(lambda_*exp(1.-2.*pow(fabs(1.-d),p)),
                                    LAMBDA_MIN),LAMBDA_MAX);

        if (std::isnan(lambda_)) {
            std::cout << d << " " << (1.-d)/std::numeric_limits<double>::epsilon() << " "
                      << pow(fabs(1.-d),p) << " " << exp(1.-2.*pow(fabs(1.-d),p)) << std::endl;
            assert(0);
        }
//        if (d > th) {
//            numEff++;
//        } else {
//            numIneff++;
//        }

        nn_->add(motion);
        return motion;
    } else {
        //numIneff++;
        lambda_ = std::min(std::max(lambda_/M_E,LAMBDA_MIN),LAMBDA_MAX);
        return nullptr;
    }
}


ompl::base::PlannerStatus ompl::geometric::MyFOSVFRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (!sampler_) sampler_ = si_->allocStateSampler();

    meanNorm_ = determineMeanNorm();
    OMPL_INFORM("%s: Mean norm %f", getName().c_str(), meanNorm_);
    lambda_ = LAMBDA_MAX;
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

    //numEff = 0;
    //numIneff = 0;
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
    //OMPL_INFORM("%s: Inefficiency rate %f", getName().c_str(),double(numIneff)/double(numEff+numIneff));


    return base::PlannerStatus(solved, approximate);
}
