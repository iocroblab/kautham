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

#include "myPCARRT.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <armadillo> // for compute PCA
#include "ompl/geometric/PathGeometric.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace arma;

namespace Kautham {
namespace omplplanner {
ob::PlannerStatus myPCARRT::solve(const ob::PlannerTerminationCondition &ptc) {
    checkValidity();
    ob::Goal *goal = pdef_->getGoal().get();
    ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

    while (const ob::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state,st);
        nn_->add(motion);
    }

    if (nn_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!",getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",getName().c_str(),nn_->size());

    Motion *solution = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    ob::State *xstate = si_->allocState();

    while (!ptc) {
        Motion *nmotion;
        ob::State *dstate;
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
                //Calculate the new q_rand.
                // Definitions for change from planer variables (Motion,rstate) to armadillo variable (vec)
                ob::StateSpacePtr space = getSpaceInformation()->getStateSpace(); // obtain the state space
                ob::StateSpacePtr ssRobot0 = ((ob::StateSpacePtr)space->as<ob::CompoundStateSpace>()->getSubspace(0)); //paso 1)se extrae el subespacio correspondiente al robot 0
                ob::StateSpacePtr ssRobot0se3 = ((ob::StateSpacePtr)ssRobot0->as<ob::CompoundStateSpace>()->getSubspace(0)); //paso 2) se extrae el componente se3
                ob::ScopedState<ob::CompoundStateSpace> sstate_qrand(space);      // definition of scoped state (encapsulation of a state)
                ob::ScopedState<ob::SE3StateSpace> qrandse3(ssRobot0se3); //se define un scoped state para la parte se3 del qrand
                ob::ScopedState<ob::CompoundStateSpace> sstate_qnear(space);      //se define un scoped state para el qnear (encapsulameinto de un state)
                ob::ScopedState<ob::SE3StateSpace> qnearse3(ssRobot0se3); //se define un scoped state para la parte se3 del qnear

                //change of rstate and nmotion->state to armadillo vectors (vec)
                // q_rand
                vec old_q_rand(2);
                sstate_qrand = *rstate;          //obtain the qrand with rstate
                sstate_qrand >> qrandse3;       //fill the scoped state qrandse3 with the content of se3 of the scoped state sstate_qrand
                old_q_rand[0] = qrandse3->getX(); // obtain the x of q_rand state
                old_q_rand[1] = qrandse3->getY(); // obtain the y of q_rand state

                // q_near
                vec new_nmotion(2);
                sstate_qnear = *(nmotion->state); //obtain the qnear with nmotion->state
                sstate_qnear >> qnearse3;        //fill the scoped state qnearse3 with the content of se3 del scoped state sstate_qnear
                new_nmotion[0] = qnearse3->getX(); //obtain the x of q_near state
                new_nmotion[1] = qnearse3->getY(); //obtain the y of q_near state

                //calculate the new rstate
                vec q_rand_new = new_qRand(old_q_rand,new_nmotion);

                // variable change from armadillo vector (vec) to rstate
                qrandse3->setX(q_rand_new[0]); //change the X of qrand
                qrandse3->setY(q_rand_new[1]); //change the Y of qrand

                sstate_qrand << qrandse3;
                space->copyState(rstate,sstate_qrand.get());
                //rstate = sstate_qrand.get();
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
        og::PathGeometric *path = new og::PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
            path->append(mpath[i]->state);
        }
        pdef_->addSolutionPath(ob::PathPtr(path),approximate,approxdif);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state) si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states",getName().c_str(),nn_->size());

    return ob::PlannerStatus(solved,approximate);
}


arma::vec myPCARRT::new_qRand(arma::vec qr, arma::vec qn) {
    arma::vec dq(qr-qn);
    dq /= norm(dq);
    PCAResult *pmdSet = tree_->getPMD(qn);
    if (pmdSet && dot(dq,pmdSet->b) >= 0.) {
        //Compute normalized velocity
        arma::vec Lv(tree_->getVelocityLimits());
        arma::vec dv(pmdSet->dim);
        for (unsigned int i = 0; i < pmdSet->dim; ++i) {
            dv[i] = (qr[i]-qn[i])/Lv[i];
        }
        dv /= norm(dv);
        arma::vec v(pmdSet->b);
        for (unsigned int i = 0; i < pmdSet->dim; ++i) {
            v += pmdSet->a[i]*dot(dv,pmdSet->U.col(i))*pmdSet->U.col(i);
        }

        //Compute real velocity
        for (unsigned int i = 0; i < pmdSet->dim; ++i) {
            v[i] = Lv[i]*v[i];
        }

        //Compute
        double c = 1-sqrt(rng_.uniform01());
        v = c*dq*maxDistance_ + (1-c)*alfa_*v;

        return qn + v;

    } else {
        return qn + dq*maxDistance_;
    }

    /*//double timeStep = maxDistance_/pmdSet_->avgVel;
    double timeStep = 1.;
    cout << "qr:[" << qr[0] << " " << qr[1] << "] ";
    cout << "qn:[" << qn[0] << " " << qn[1] << "] ";
    cout << "Dt:" << timeStep << " ";

    arma::vec x((qr-qn)/timeStep);
    cout << "v:[" << x[0] << " " << x[1] << "] ";
    cout << "|v|:" << norm(x) << " max:" << pmdSet_->at(index)->maxVel << " ";
    x *= std::min(pmdSet_->at(index)->maxVel/norm(x),1.);
    cout << "v:[" << x[0] << " " << x[1] << "] ";
    x = alfa_*(x-pmdSet_->at(index)->b)/pmdSet_->at(index)->a[0];

    arma::vec v(pmdSet_->at(index)->b);

    for (unsigned int i = 0; i < pmdSet_->at(index)->dim; i++) {
        v += pmdSet_->at(index)->a[i]*dot(x,pmdSet_->at(index)->U.col(i))*pmdSet_->at(index)->U.col(i);
    }
    cout << "v':[" << v[0] << " " << v[1] << "] ";
    cout << "Dq:" << norm(v)*timeStep << " ";
    x = qn + timeStep*v;
    cout << "qr':[" << x[0] << " " << x[1] << "]" << endl << endl;
    arma::vec dq(v*timeStep);
    dq *= std::min(maxDistance_/norm(dq),1.);
    return qn + timeStep*v;*/
}

}
}
