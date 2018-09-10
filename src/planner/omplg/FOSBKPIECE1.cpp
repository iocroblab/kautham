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

#include <kautham/planner/omplg/FOSBKPIECE1.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <cassert>


#define LAMBDA_MIN 1.e-3
#define LAMBDA_MAX 1.e+5


ompl::geometric::FOSBKPIECE1::FOSBKPIECE1(const base::SpaceInformationPtr &si,
                                          omplplanner::omplPlanner *pl,
                                          SynergyTree *st, double exploration,
                                          double initial_lambda, unsigned int update_freq) :
    base::Planner(si,"FOSBKPIECE1"),
    dStart_(std::bind(&FOSBKPIECE1::freeMotion,this,std::placeholders::_1),
            std::bind(&FOSBKPIECE1::distanceFunction,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&FOSBKPIECE1::getDefaultNearestNeighbors,this)),
    dGoal_(std::bind(&FOSBKPIECE1::freeMotion,this,std::placeholders::_1),
           std::bind(&FOSBKPIECE1::distanceFunction,this,std::placeholders::_1,std::placeholders::_2),
           std::bind(&FOSBKPIECE1::getDefaultNearestNeighbors,this)),
    st_(st), failedExpansionScoreFactor_(0.5), treeBias_(0.05), minValidPathFraction_(0.5),
    maxDistance_(0.), connectionPoint_(nullptr,nullptr), efficientCount_(0), inefficientCount_(0),
    explorationInefficiency_(0.), explorationSetting_(exploration), initialLambda_(initial_lambda),
    efficiencyThreshold_(si_->getStateValidityCheckingResolution()), nth_step_(update_freq), step_(0), vfdim_(0) {
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range",this,&FOSBKPIECE1::setRange,
                                  &FOSBKPIECE1::getRange,"0.:1.:10000.");
    Planner::declareParam<double>("tree_bias",this,&FOSBKPIECE1::setTreeBias,
                                  &FOSBKPIECE1::getTreeBias,"0.:1.");
    Planner::declareParam<double>("border_fraction",this,&FOSBKPIECE1::setBorderFraction,
                                  &FOSBKPIECE1::getBorderFraction,"0.:.05:1.");
    Planner::declareParam<double>("failed_expansion_score_factor",this,
                                  &FOSBKPIECE1::setFailedExpansionCellScoreFactor,
                                  &FOSBKPIECE1::getFailedExpansionCellScoreFactor);
    Planner::declareParam<double>("min_valid_path_fraction",this,
                                  &FOSBKPIECE1::setMinValidPathFraction,
                                  &FOSBKPIECE1::getMinValidPathFraction);
    Planner::declareParam<double>("initial_lambda",this,
                                  &FOSBKPIECE1::setInitialLambda,
                                  &FOSBKPIECE1::getInitialLambda);
    Planner::declareParam<double>("efficiency_threshold",this,
                                  &FOSBKPIECE1::setEfficiencyThreshold,
                                  &FOSBKPIECE1::getEfficiencyThreshold);
    Planner::declareParam<double>("exploration",this,&FOSBKPIECE1::setExploration,
                                  &FOSBKPIECE1::getExploration,"0.:1.");
    Planner::declareParam<double>("update_freq",this,&FOSBKPIECE1::setUpdateFrequency,
                                  &FOSBKPIECE1::getUpdateFrequency,"0.:1.e100");

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


void ompl::geometric::FOSBKPIECE1::omplState2armaVec(const ompl::base::State *s,
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


void ompl::geometric::FOSBKPIECE1::armaVec2omplState(const arma::vec &q,
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


ompl::geometric::FOSBKPIECE1::~FOSBKPIECE1() {}


void ompl::geometric::FOSBKPIECE1::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_,getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    if (failedExpansionScoreFactor_ < std::numeric_limits<double>::epsilon() ||
            failedExpansionScoreFactor_ > 1.) {
        throw Exception("Failed expansion cell score factor must be in the range (0,1]");
    }
    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() ||
            minValidPathFraction_ > 1.) {
        throw Exception("The minimum valid path fraction must be in the range (0,1]");
    }

    dStart_.setDimension(projectionEvaluator_->getDimension());
    dGoal_.setDimension(projectionEvaluator_->getDimension());
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

        dStart_.setBounds(low,high);
        dGoal_.setBounds(low,high);
    }
}


void ompl::geometric::FOSBKPIECE1::clear() {
    Planner::clear();

    efficientCount_ = 0;
    inefficientCount_ = 0;
    explorationInefficiency_ = 0.;
    step_ = 0;
    sampler_.reset();
    dStart_.clear();
    dGoal_.clear();
    connectionPoint_ = std::make_pair<base::State*,base::State*>(nullptr,nullptr);
}


void ompl::geometric::FOSBKPIECE1::freeMotion(Motion *motion) {
    if (motion->state) si_->freeState(motion->state);
    delete motion;
}


arma::vec ompl::geometric::FOSBKPIECE1::getNewDirection(const arma::vec &vrand,
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


void ompl::geometric::FOSBKPIECE1::updateGain() {
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


ompl::geometric::FOSBKPIECE1::Motion *ompl::geometric::FOSBKPIECE1::extendTree
(Motion *m, Discretization<Motion>::Cell *cell, const base::State *biased, bool reverseVF, base::State *tmp) {
    if (si_->distance(m->state,biased) <= maxDistance_) {
        si_->copyState(tmp,biased);
    } else {
        arma::vec q;
        omplState2armaVec(m->state,q);

        // Get the vector at q and normalize
        arma::vec vfield = (reverseVF? -st_->vectorField(q) : st_->vectorField(q));

        arma::vec vrand;
        if (rng_.uniform01() < treeBias_) {
            arma::vec qbias;
            omplState2armaVec(biased,qbias);
            vrand = arma::normalise(qbias-q);
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
        motion->root = m->root;
        updateExplorationEfficiency(motion,cell);

        return motion;
    } else {
        inefficientCount_++;
        explorationInefficiency_ = double(inefficientCount_)/double(efficientCount_+inefficientCount_);
        return nullptr;
    }
}


void ompl::geometric::FOSBKPIECE1::updateExplorationEfficiency(Motion *m,
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


ompl::base::PlannerStatus ompl::geometric::FOSBKPIECE1::solve
(const base::PlannerTerminationCondition &ptc) {
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

    base::State *startState = nullptr;
    Discretization<Motion>::Coord xcoord;
    while (const base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,st);
        motion->root = motion->state;
        projectionEvaluator_->computeCoordinates(motion->state,xcoord);
        dStart_.addMotion(motion,xcoord,goal->distanceGoal(st));
        if (startState == nullptr) startState = motion->state;
    }
    if (dStart_.getMotionCount() == 0) {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!",
                   getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure",
                getName().c_str(),(int)(dStart_.getMotionCount() + dGoal_.getMotionCount()));

    base::State *tmpState = si_->allocState();
    bool startTree = true;
    bool solved = false;

    while (!ptc) {
        Discretization<Motion> &disc = startTree ? dStart_ : dGoal_;
        startTree = !startTree;
        Discretization<Motion> &otherDisc = startTree ? dStart_ : dGoal_;
        disc.countIteration();

        // if we have not sampled too many goals already
        if (dGoal_.getMotionCount() == 0 ||
                pis_.getSampledGoalsCount() < dGoal_.getMotionCount()/2) {
            const base::State *st = dGoal_.getMotionCount() == 0 ?
                        pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st) {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state,st);
                motion->root = motion->state;
                projectionEvaluator_->computeCoordinates(motion->state,xcoord);
                dGoal_.addMotion(motion,xcoord,si_->distance(st,startState));
            }
            if (dGoal_.getMotionCount() == 0) {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree",
                           getName().c_str());
                break;
            }
        }

        Discretization<Motion>::Cell *ecell = nullptr;
        Motion *existing = nullptr;
        disc.selectMotion(existing,ecell);
        assert(existing);

        Discretization<Motion>::Cell *ocell = nullptr;
        Motion *otherExisting = nullptr;
        otherDisc.selectTopExternalMotion(otherExisting,ocell);
        assert(otherExisting);

        Motion *motion = extendTree(existing,ecell,otherExisting->state,startTree,tmpState);
        if (motion) {
            projectionEvaluator_->computeCoordinates(motion->state,xcoord);
            disc.addMotion(motion,xcoord,distanceFunction(motion,otherExisting));

            ocell = otherDisc.getGrid().getCell(xcoord);
            if (ocell && ocell->data->motions->size() > 0) {
                otherExisting = ocell->data->motions->nearest(motion);
                if (goal->isStartGoalPairValid(startTree ? otherExisting->root : motion->root,
                                               startTree ? motion->root : otherExisting->root) &&
                        si_->checkMotion(motion->state,otherExisting->state)) {
                    if (startTree) {
                        connectionPoint_ = std::make_pair(otherExisting->state,motion->state);
                    } else {
                        connectionPoint_ = std::make_pair(motion->state,otherExisting->state);
                    }

                    // Construct the solution path
                    std::vector<Motion*> mpath1;
                    while (motion != nullptr) {
                        mpath1.push_back(motion);
                        motion = motion->parent;
                    }
                    std::vector<Motion*> mpath2;
                    while (otherExisting != nullptr) {
                        mpath2.push_back(otherExisting);
                        otherExisting = otherExisting->parent;
                    }
                    if (startTree) mpath1.swap(mpath2);

                    // Set the solution path
                    PathGeometric *path = new PathGeometric(si_);
                    path->getStates().reserve(mpath1.size() + mpath2.size());
                    for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
                        path->append(mpath1[i]->state);
                    }
                    for (unsigned int i = 0 ; i < mpath2.size() ; ++i) {
                        path->append(mpath2[i]->state);
                    }
                    pdef_->addSolutionPath(base::PathPtr(path),false,0.0,getName());
                    solved = true;
                    break;
                }
            }
        } else {
            ecell->data->score *= failedExpansionScoreFactor_;
            ecell->data->addCoef *= failedExpansionScoreFactor_;
            ecell->data->multCoef *= failedExpansionScoreFactor_;
        }

        disc.updateCell(ecell);
    }
    si_->freeState(tmpState);

    OMPL_INFORM("%s: Created %u (%u start + %u goal) states in %u cells (%u start (%u on boundary) + %u goal (%u on boundary))",
                getName().c_str(),
                dStart_.getMotionCount() + dGoal_.getMotionCount(),dStart_.getMotionCount(),dGoal_.getMotionCount(),
                dStart_.getCellCount() + dGoal_.getCellCount(),dStart_.getCellCount(),dStart_.getGrid().countExternal(),
                dGoal_.getCellCount(),dGoal_.getGrid().countExternal());
    OMPL_INFORM("%s: Final lambda %f",getName().c_str(),lambda_);
    OMPL_INFORM("%s: Iterations %u",getName().c_str(),dStart_.iterations()+dGoal_.iterations());

    //        std::vector<Discretization<Motion>::CellData*> cdataS;
    //        dStart_.getGrid().getContent(cdataS);
    //        std::vector<Discretization<Motion>::CellData*> cdataG;
    //        dGoal_.getGrid().getContent(cdataG);
    //        arma::vec motions(cdataS.size()+cdataG.size());
    //        for (unsigned int i = 0; i < cdataS.size(); ++i) {
    //            motions[i] = cdataS[i]->orderedMotions.size();
    //        }
    //        for (unsigned int i = 0; i < cdataG.size(); ++i) {
    //            motions[cdataS.size()+i] = cdataG[i]->orderedMotions.size();
    //        }
    //        std::cout << "Min: " << motions.min() << " Max: " << motions.max() << " Mean: "
    //                  << arma::sum(motions)/double(motions.n_elem) << std::endl;

    return (solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT);
}

void ompl::geometric::FOSBKPIECE1::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);
    dStart_.getPlannerData(data,1,true,nullptr);
    dGoal_.getPlannerData(data,2,false,nullptr);

    // Insert the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first),
                 data.vertexIndex(connectionPoint_.second));
}
