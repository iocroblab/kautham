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

#include <kautham/planner/omplg/FOSRRT.h>
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <armadillo> // for compute PCA
#include "ompl/geometric/PathGeometric.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <algorithm>
#include <cfloat>

using namespace arma;


ompl::base::PlannerStatus ompl::geometric::FOSRRT::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    checkValidity();
    ompl::base::Goal *goal = pdef_->getGoal().get();
    ompl::base::GoalSampleableRegion *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

    while (const ompl::base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,st);
        nn_->add(motion);
    }

    if (nn_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!",getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",getName().c_str(),nn_->size());

    Motion *solution = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion = new Motion(si_);
    ompl::base::State *rstate = rmotion->state;
    ompl::base::State *xstate = si_->allocState();

    while (!ptc) {
        Motion *nmotion;
        ompl::base::State *dstate;
        if (rng_.uniform01() < pmdBias_) {
            // sample random state (with goal biasing)
            bool isGoal;
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
                goal_s->sampleGoal(rstate);
                isGoal = true;
            } else {
                isGoal = false;
                sampler_->sampleUniform(rstate);
            }

            // find closest state in the tree
            nmotion = nn_->nearest(rmotion);
            dstate = rstate;

            if (!isGoal || si_->distance(nmotion->state,rstate) > maxDistance_) {
                //Get qRand and qNear
                vec qRand, qNear;
                omplState2armaVec(rstate,qRand);
                omplState2armaVec(nmotion->state,qNear);

                //cout << "qNear" << endl << qNear << endl;
                //cout << "qRand" << endl << qRand << endl;

                //Compute new qRand
                qRand = new_qRand(qRand,qNear);

                //Save new qRand
                armaVec2omplState(qRand,rstate);

                omplState2armaVec(rstate,qRand);
                //cout << "qRand*" << endl << qRand << endl;
                //cout << endl;

            }
        } else {
            // sample random state (with goal biasing)
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
                goal_s->sampleGoal(rstate);
            } else {
                sampler_->sampleUniform(rstate);
            }

            // find closest state in the tree
            nmotion = nn_->nearest(rmotion);
            dstate = rstate;

            // find state to add
            double d = si_->distance(nmotion->state,rstate);
            if (d > maxDistance_) {
                si_->getStateSpace()->interpolate(nmotion->state,rstate,maxDistance_/d,xstate);
                dstate = xstate;
            }
        }

        if (si_->checkMotion(nmotion->state,dstate)) {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state,dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.;
            if (goal->isSatisfied(motion->state,&dist)) {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif) {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (!solution) {
        solution = approxsol;
        approximate = true;
    }

    if (solution) {
        lastGoalMotion_ = solution;

        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
            path->append(mpath[i]->state);
        }
        pdef_->addSolutionPath(ompl::base::PathPtr(path),approximate,approxdif);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state) si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states",getName().c_str(),nn_->size());

    return ompl::base::PlannerStatus(solved,approximate);
}

void ompl::geometric::FOSRRT::omplState2armaVec(const ompl::base::State *state, arma::vec &vector) {
    /*It is suposed that no coupled DOF exist.
     *Each acted DOF has a mapMatrix row with all the element equal to 0 and
     *only one element equal to 1. The offMatrix element must be equal to 0.5*/

    unsigned int nDOF = planner_->wkSpace()->getNumRobControls();
    vector.resize(nDOF);
    //Convert from ompl state to kautham sample
    Sample smp(nDOF);
    smp.setMappedConf(planner_->initSamp()->getMappedConf());
    planner_->omplState2smp(state,&smp);

    //Convert from kautham sample to armadillo vec
    std::vector<RobConf> conf(smp.getMappedConf());
    RobConf robConf;
    Robot *robot;
    bool found;
    unsigned int k(0),j;
    KthReal value;

    for (unsigned int u = 0; u < conf.size(); ++u) {
        robot = planner_->wkSpace()->getRobot(u);
        robConf = conf.at(u);
        for (unsigned int i = 0; i < robot->getNumJoints()+6; ++i) {
            //Check if all values in i-th mapping matrix row are zero
            j = 0;
            found = false;
            while (j < nDOF && !found) {
                if (fabs(robot->getMapMatrix()[i][j]) < FLT_EPSILON) {
                    j++;
                } else {
                    found = true;
                }
            }

            if (found) {//If an element not null has been found
                if (i < 3) {//Base translation DOF
                    value = robConf.getSE3().getPos().at(i);
                } else if (i < 6) {//Base rotation DOF
                    value = robConf.getSE3().getParams().at(i-3);
                } else {//Joint DOF
                    value = robConf.getRn().getCoordinate(i-6);
                }

                vector[k] = value;
                k++;

                if (k >= nDOF) break;
            }
        }

        if (k >= nDOF) break;
    }
}


void ompl::geometric::FOSRRT::armaVec2omplState(const arma::vec vector, ompl::base::State *state) {
    /*It is suposed that no coupled DOF exist.
     *Each acted DOF has a mapMatrix row with all the element equal to 0 and
     *only one element equal to 1. The offMatrix element must be equal to 0.5*/

    unsigned int nDOF = planner_->wkSpace()->getNumRobControls();
    Sample smp(nDOF);
    smp.setMappedConf(planner_->initSamp()->getMappedConf());
    planner_->omplState2smp(state,&smp);

    //Convert from armadillo vec to kautham sample

    /*for (unsigned int i = 0; i < smp.getMappedConf().size(); ++i) {
        cout << "[ ";
        for (unsigned int j = 0; j < 3; ++j) {
            cout << smp.getMappedConf().at(i).getSE3().getPos().at(j) << " ";
        }
        for (unsigned int j = 0; j < 3; ++j) {
            cout << smp.getMappedConf().at(i).getSE3().getParams().at(j) << " ";
        }
        for (unsigned int j = 0; j < smp.getMappedConf().at(i).getRn().getCoordinates().size(); ++j) {
            cout << smp.getMappedConf().at(i).getRn().getCoordinate(j) << " ";
        }
        cout << "]" << endl;
    }*/

    std::vector<RobConf> conf(smp.getMappedConf());
    RobConf robConf;
    Robot *robot;
    bool found;
    unsigned int k(0),j;
    std::vector<KthReal> coords;
    for (unsigned int u = 0; u < conf.size(); ++u) {
        robot = planner_->wkSpace()->getRobot(u);
        robConf = conf.at(u);
        for (unsigned int i = 0; i < robot->getNumJoints()+6; ++i) {
            //Check if all values in i-th mapping matrix row are zero
            j = 0;
            found = false;
            while (j < nDOF && !found) {
                if (fabs(robot->getMapMatrix()[i][j]) < FLT_EPSILON) {
                    j++;
                } else {
                    found = true;
                }
            }

            if (found) {//If an element not null has been found
                if (i < 3) {//Base translation DOF
                    robConf.getSE3().getCoordinates().at(i) = vector[k];
                } else if (i < 6) {//Base rotation DOF
                    coords.resize(6);

                    coords.at(0) = robConf.getSE3().getPos().at(0);
                    coords.at(1) = robConf.getSE3().getPos().at(1);
                    coords.at(2) = robConf.getSE3().getPos().at(2);
                    coords.at(3) = robConf.getSE3().getParams().at(0);
                    coords.at(4) = robConf.getSE3().getParams().at(1);
                    coords.at(5) = robConf.getSE3().getParams().at(2);

                    coords.at(i-3) = vector[k];

                    robConf.getSE3().setCoordinates(coords);
                } else {//Joint DOF
                    robConf.getRn().getCoordinates().at(i-6) = vector[k];
                }

                k++;

                if (k >= nDOF) break;
            }
        }
        conf.at(u) = robConf;

        if (k >= nDOF) break;
    }
    smp.setMappedConf(conf);

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> sstate(getSpaceInformation()->getStateSpace());
    planner_->smp2omplScopedState(&smp,&sstate);
    getSpaceInformation()->getStateSpace()->copyState(state,sstate.get());

    /*for (unsigned int i = 0; i < smp.getMappedConf().size(); ++i) {
        cout << "[ ";
        for (unsigned int j = 0; j < 3; ++j) {
            cout << smp.getMappedConf().at(i).getSE3().getPos().at(j) << " ";
        }
        for (unsigned int j = 0; j < 3; ++j) {
            cout << smp.getMappedConf().at(i).getSE3().getParams().at(j) << " ";
        }
        for (unsigned int j = 0; j < smp.getMappedConf().at(i).getRn().getCoordinates().size(); ++j) {
            cout << smp.getMappedConf().at(i).getRn().getCoordinate(j) << " ";
        }
        cout << "]" << endl;
    }*/
}


arma::vec ompl::geometric::FOSRRT::new_qRand(arma::vec qr, arma::vec qn) {
    //Cell PMD set
    const Synergy *synergy = tree_->getSynergy(qn);

    //Scaled velocity
    arma::vec dq(qr-qn);
    arma::vec Lv(tree_->getVelocityLimits());
    arma::vec v(Lv.n_elem);
    for (unsigned int i = 0; i < Lv.n_elem; ++i) {
        v[i] = dq[i]/Lv[i]/timeStep_;
    }
    v /= std::max(1.,arma::max(arma::abs(v)));

    //
    if (synergy && dot(v,synergy->b) >= 0.) {
        //vEps
        arma::vec vEps(v*(maxDistance_/timeStep_/norm(v)));

        //vFos
        arma::vec x = normalise(v-synergy->b);
        arma::vec vFos(synergy->b);
        for (unsigned int i = 0; i < synergy->dim; ++i) {
            vFos += synergy->a[i]*dot(x,synergy->U.col(i))*synergy->U.col(i);
        }

        //vC
        double c = sqrt(rng_.uniform01());
        arma::vec vC = c*vFos + (1-c)*vEps;

        //Real velocity
        arma::vec vReal(synergy->dim);
        for (unsigned int i = 0; i < synergy->dim; ++i) {
            vReal[i] = vC[i]*Lv[i];
        }

        //New configuration
        return qn + timeStep_*vReal;
    } else {
        return qn + std::min(maxDistance_/norm(dq),1.)*dq;
    }

    /*//double timeStep = maxDistance_/synergy_->avgVel;
    double timeStep = 1.;
    cout << "qr:[" << qr[0] << " " << qr[1] << "] ";
    cout << "qn:[" << qn[0] << " " << qn[1] << "] ";
    cout << "Dt:" << timeStep << " ";

    arma::vec x((qr-qn)/timeStep);
    cout << "v:[" << x[0] << " " << x[1] << "] ";
    cout << "|v|:" << norm(x) << " max:" << synergy_->at(index)->maxVel << " ";
    x *= std::min(synergy_->at(index)->maxVel/norm(x),1.);
    cout << "v:[" << x[0] << " " << x[1] << "] ";
    x = alfa_*(x-synergy_->at(index)->b)/synergy_->at(index)->a[0];

    arma::vec v(synergy_->at(index)->b);

    for (unsigned int i = 0; i < synergy_->at(index)->dim; i++) {
        v += synergy_->at(index)->a[i]*dot(x,synergy_->at(index)->U.col(i))*synergy_->at(index)->U.col(i);
    }
    cout << "v':[" << v[0] << " " << v[1] << "] ";
    cout << "Dq:" << norm(v)*timeStep << " ";
    x = qn + timeStep*v;
    cout << "qr':[" << x[0] << " " << x[1] << "]" << endl << endl;
    arma::vec dq(v*timeStep);
    dq *= std::min(maxDistance_/norm(dq),1.);
    return qn + timeStep*v;*/
}
