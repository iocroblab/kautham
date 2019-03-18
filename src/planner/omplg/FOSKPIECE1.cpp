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

#include <kautham/planner/omplg/FOSKPIECE1.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>
#include <cassert>
#include <cfloat>

#define LAMBDA_MIN 1.e-3
#define LAMBDA_MAX 1.e+5


ompl::geometric::FOSKPIECE1::FOSKPIECE1(const base::SpaceInformationPtr &si,
                                        omplplanner::omplPlanner *pl,
                                        SynergyTree *st, double exploration,
                                        double initial_lambda, unsigned int update_freq) :
    base::Planner(si,"FOSKPIECE1"),
    disc_(std::bind(&FOSKPIECE1::freeMotion,this,std::placeholders::_1),
          std::bind(&FOSKPIECE1::distanceFunction,this,std::placeholders::_1,std::placeholders::_2),
          std::bind(&FOSKPIECE1::getDefaultNearestNeighbors,this)),
    st_(st), failedExpansionScoreFactor_(0.5), goalBias_(0.05), minValidPathFraction_(0.5),
    maxDistance_(0.0), lastGoalMotion_(nullptr), efficientCount_(0), inefficientCount_(0),
    explorationInefficiency_(0.), explorationSetting_(exploration), initialLambda_(initial_lambda),
    efficiencyThreshold_(si_->getStateValidityCheckingResolution()), nth_step_(update_freq), step_(0), vfdim_(0) {
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range",this,&FOSKPIECE1::setRange,
                                  &FOSKPIECE1::getRange,"0.:1.:10000.");
    Planner::declareParam<double>("goal_bias",this,&FOSKPIECE1::setGoalBias,
                                  &FOSKPIECE1::getGoalBias,"0.:.05:1.");
    Planner::declareParam<double>("border_fraction",this,&FOSKPIECE1::setBorderFraction,
                                  &FOSKPIECE1::getBorderFraction,"0.:0.05:1.");
    Planner::declareParam<double>("failed_expansion_score_factor",this,
                                  &FOSKPIECE1::setFailedExpansionCellScoreFactor,
                                  &FOSKPIECE1::getFailedExpansionCellScoreFactor);
    Planner::declareParam<double>("min_valid_path_fraction",this,
                                  &FOSKPIECE1::setMinValidPathFraction,
                                  &FOSKPIECE1::getMinValidPathFraction);
    Planner::declareParam<double>("initial_lambda",this,
                                  &FOSKPIECE1::setInitialLambda,
                                  &FOSKPIECE1::getInitialLambda);
    Planner::declareParam<double>("efficiency_threshold",this,
                                  &FOSKPIECE1::setEfficiencyThreshold,
                                  &FOSKPIECE1::getEfficiencyThreshold);
    Planner::declareParam<double>("exploration",this,&FOSKPIECE1::setExploration,
                                  &FOSKPIECE1::getExploration,"0.:1.");
    Planner::declareParam<double>("update_freq",this,&FOSKPIECE1::setUpdateFrequency,
                                  &FOSKPIECE1::getUpdateFrequency,"0.:1.e100");

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


void ompl::geometric::FOSKPIECE1::omplState2armaVec(const ompl::base::State *s,
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


void ompl::geometric::FOSKPIECE1::armaVec2omplState(const arma::vec &q,
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


ompl::geometric::FOSKPIECE1::~FOSKPIECE1() {}


void ompl::geometric::FOSKPIECE1::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_,getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    if (failedExpansionScoreFactor_ < std::numeric_limits<double>::epsilon() || failedExpansionScoreFactor_ > 1.0) {
        throw Exception("Failed expansion cell score factor must be in the range (0,1]");
    }
    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() || minValidPathFraction_ > 1.0) {
        throw Exception("The minimum valid path fraction must be in the range (0,1]");
    }

    disc_.setDimension(projectionEvaluator_->getDimension());
    if (projectionEvaluator_->hasBounds()) {
        Discretization<Motion>::Coord low, high;
        Kautham::Vector tmp(projectionEvaluator_->getDimension());

#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
        std::copy(projectionEvaluator_->getBounds().low.begin(),
                  projectionEvaluator_->getBounds().low.end(),tmp.data());
#else
        std::copy(projectionEvaluator_->getBounds().low.begin(),
                  projectionEvaluator_->getBounds().low.end(),tmp.begin());
#endif
        projectionEvaluator_->computeCoordinates(tmp,low);

#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
        std::copy(projectionEvaluator_->getBounds().high.begin(),
                  projectionEvaluator_->getBounds().high.end(),tmp.data());
#else
        std::copy(projectionEvaluator_->getBounds().high.begin(),
                  projectionEvaluator_->getBounds().high.end(),tmp.begin());
#endif
        projectionEvaluator_->computeCoordinates(tmp,high);

        disc_.setBounds(low,high);
    }
}


void ompl::geometric::FOSKPIECE1::clear() {
    Planner::clear();

    efficientCount_ = 0;
    inefficientCount_ = 0;
    explorationInefficiency_ = 0.;
    step_ = 0;
    sampler_.reset();
    disc_.clear();
    lastGoalMotion_ = nullptr;
}


void ompl::geometric::FOSKPIECE1::freeMotion(Motion *motion) {
    if (motion->state) si_->freeState(motion->state);
    delete motion;
}


arma::vec ompl::geometric::FOSKPIECE1::getNewDirection(const arma::vec &vrand,
                                                       const arma::vec &vfield) {
    updateGain();
    double c = arma::dot(vfield,vrand);
    if (1.-fabs(c) <= 2.*std::numeric_limits<double>::epsilon()) return vrand;

    double w2 = -2./lambda_*log(1.-0.5*(1.-c)*(1.-exp(-2.*lambda_)));
    double wfield = (1.-0.5*w2)-0.5*c*sqrt(w2*(4.-w2)/(1.-c*c));
    double wrand = wfield-1.;
    wrand = sqrt(wrand*wrand+wfield*w2);

    return wrand*vrand + wfield*vfield;
}


void ompl::geometric::FOSKPIECE1::updateGain() {
    if (step_ == nth_step_) {
        lambda_ = std::min(std::max(lambda_*(1.-explorationInefficiency_+explorationSetting_),LAMBDA_MIN),LAMBDA_MAX);
        efficientCount_ = 0;
        inefficientCount_ = 0;
        explorationInefficiency_ = 0;
        step_ = 0;
    } else {
        step_++;
    }
}


ompl::geometric::FOSKPIECE1::Motion *ompl::geometric::FOSKPIECE1::extendTree
(Motion *m, Discretization<Motion>::Cell *cell, base::GoalSampleableRegion *goal, base::State *tmp) {
    if (goal->distanceGoal(m->state) <= maxDistance_) {
        goal->sampleGoal(tmp);
    } else {
        arma::vec q;
        omplState2armaVec(m->state,q);

        // Get the vector at q and normalize
        arma::vec vfield = st_->vectorField(q);

        arma::vec vrand;
        if (rng_.uniform01() < goalBias_) {
            goal->sampleGoal(tmp);
            arma::vec qgoal;
            omplState2armaVec(tmp,qgoal);
            vrand = arma::normalise(qgoal-q);
        } else {
            if (m->parent == nullptr) {
                vrand = arma::normalise(arma::randn(vfield.n_elem));
            } else {
                arma::vec qparent;
                omplState2armaVec(m->parent->state,qparent);
                vrand = q-qparent;
                arma::vec v = arma::randn(vfield.n_elem);
                double c = arma::dot(v,vrand);
                if (c < 0.) v = v - 2.*c*vrand/arma::dot(vrand,vrand);
                vrand = arma::normalise(v);
            }
        }

        arma::vec vnew = getNewDirection(vrand,vfield);
        si_->copyState(tmp,m->state);
        armaVec2omplState(q + maxDistance_*vnew,tmp);
    }

    std::pair<base::State*,double> fail(tmp,0.);
    bool keep = si_->checkMotion(m->state,tmp,fail);
    if (!keep && fail.second > minValidPathFraction_) keep = true;
    if (keep) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,tmp);
        motion->parent = m;
        updateExplorationEfficiency(motion,cell);

        return motion;
    } else {
        inefficientCount_++;
        explorationInefficiency_ = double(inefficientCount_)/double(efficientCount_+inefficientCount_);
        return nullptr;
    }
}


void ompl::geometric::FOSKPIECE1::updateExplorationEfficiency(Motion *m,
                                                              Discretization<Motion>::Cell *cell) {
    if (distanceFunction(m,cell->data->motions->nearest(m)) < efficiencyThreshold_) {
        inefficientCount_++;
        explorationInefficiency_ = double(inefficientCount_)/double(efficientCount_+inefficientCount_);
        return;
    }
    for (auto it = cell->data->neighbors.begin();
         it != cell->data->neighbors.end(); ++it) {
        if (distanceFunction(m,(*it)->data->motions->nearest(m)) < efficiencyThreshold_) {
            inefficientCount_++;
            explorationInefficiency_ = double(inefficientCount_)/double(efficientCount_+inefficientCount_);
            return;
        }
    }
    efficientCount_++;
    explorationInefficiency_ = double(inefficientCount_)/double(efficientCount_+inefficientCount_);
}


ompl::base::PlannerStatus ompl::geometric::FOSKPIECE1::solve(const base::PlannerTerminationCondition &ptc) {
    checkValidity();

    base::GoalSampleableRegion *goal =
            dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    if (!goal) {
        OMPL_ERROR("%s: Unknown type of goal",getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region",getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    lambda_ = initialLambda_;
    step_ = 0;

    Discretization<Motion>::Coord xcoord;

    while (const base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,st);
        projectionEvaluator_->computeCoordinates(motion->state,xcoord);
        disc_.addMotion(motion,xcoord,goal->distanceGoal(st));
    }

    if (disc_.getMotionCount() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!",getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
                getName().c_str(),disc_.getMotionCount());

    base::State *tmpState = si_->allocState();
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double  approxdif = std::numeric_limits<double>::infinity();

    while (!ptc) {
        disc_.countIteration();

        /* Decide on a state to expand from */
        Motion *existing = nullptr;
        Discretization<Motion>::Cell *ecell = nullptr;
        disc_.selectMotion(existing,ecell);
        assert(existing);

        Motion *motion = extendTree(existing,ecell,goal,tmpState);

        if (motion) {
            double dist;
            bool solv = goal->isSatisfied(motion->state,&dist);
            projectionEvaluator_->computeCoordinates(motion->state,xcoord);
            disc_.addMotion(motion,xcoord,dist);//this will also update the discretization heaps as needed, so no call to updateCell() is needed

            if (solv) {
                approxdif = dist;
                solution = motion;
                break;
            }

            if (dist < approxdif){
                approxdif = dist;
                approxsol = motion;
            }
        } else {
            ecell->data->score *= failedExpansionScoreFactor_;
            ecell->data->addCoef *= failedExpansionScoreFactor_;
            ecell->data->multCoef *= failedExpansionScoreFactor_;
        }
        disc_.updateCell(ecell);
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr) {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr) {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
        solved = true;
    }
    si_->freeState(tmpState);

    OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)",
                getName().c_str(),
                disc_.getMotionCount(),disc_.getCellCount(),
                disc_.getGrid().countInternal(),disc_.getGrid().countExternal());
    OMPL_INFORM("%s: Final lambda %f",getName().c_str(),lambda_);
    OMPL_INFORM("%s: Iterations %u",getName().c_str(),disc_.iterations());

    //    std::vector<Discretization<Motion>::CellData*> cdata;
    //    disc_.getGrid().getContent(cdata);
    //    arma::vec motions(cdata.size());
    //    for (unsigned int i = 0; i < motions.n_elem; ++i) {
    //        motions[i] = cdata[i]->orderedMotions.size();
    //    }
    //    std::cout << "Min: " << motions.min() << " Max: " << motions.max() << " Mean: "
    //              << arma::sum(motions)/double(motions.n_elem) << std::endl;

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::FOSKPIECE1::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);
    disc_.getPlannerData(data, 0, true, lastGoalMotion_);
}
