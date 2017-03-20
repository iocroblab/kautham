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

#ifdef KAUTHAM_USE_ARMADILLO

#include <kautham/planner/omplg/FOSOptimizationObjective.h>


#define MAX_COST 1e6
#define SIGN(x) ((x>0.0)-(x<0.0))


ompl::base::FOSOptimizationObjective::FOSOptimizationObjective
(Kautham::omplplanner::omplPlanner *planner, const SpaceInformationPtr &si) :
    ompl::base::OptimizationObjective(si), planner_(planner), tree_(NULL) {
    description_ = "First Order Synergy Optimization Objective";
}


ompl::base::Cost ompl::base::FOSOptimizationObjective::stateCost(const State *s) const {
    (void) s;//unused
    return ompl::base::Cost(0.05);
}


ompl::base::Cost ompl::base::FOSOptimizationObjective::motionCost(const State *s1,
                                                                  const State *s2) const {
    //Motion goes from s1 to s2.
    //If the tree is rooted at q_init, s1 is q_near and s2 is q_new
    //If the tree is rooted at q_goal, s1 is q_new and s2 is q_near
    //Motion cost is the alignment multiplied by the motion length

    //Convert states to configurations
    arma::vec q1;
    omplState2armaVec(s1,q1);
    arma::vec q2;
    omplState2armaVec(s2,q2);

    //Get synergy
    const Synergy *synergy = tree_->getSynergy(0.5*(q1+q2));
    if (!synergy) return ompl::base::Cost(MAX_COST*si_->distance(s1,s2));

    //Get motion velocity
    arma::vec v = q2-q1;
    for (unsigned int i = 0; i < synergy->dim; ++i) {
        v(i) /= tree_->getVelocityLimits()(i);
    }

    //Compute cost
    return ompl::base::Cost(std::min(synergy->alignment(v),MAX_COST)*si_->distance(s1,s2));
}


ompl::base::Cost ompl::base::FOSOptimizationObjective::preSolveMotionCost(const State *s1,
                                                                          const State *s2) const {
    //s1 is q_near and s2 is q_new.
    //Motion cost is the distance between s1 and
    //the global zero-order box multiplied by the motion length

    //Compute cost
    return ompl::base::Cost(boxZosDistance(s2)*si_->distance(s1,s2));
}


double ompl::base::FOSOptimizationObjective::boxZosDistance(const State *s) const {
    //Convert state to configuration
    arma::vec q;
    omplState2armaVec(s,q);

    arma::mat L(tree_->getPositionLimits());
    for (unsigned int i (0); i < L.n_rows; ++i) {
        q(i) = (q(i)-L(i,0))/(L(i,1)-L(i,0));
    }

    //Return distance
    return tree_->distance(q);
}


void ompl::base::FOSOptimizationObjective::omplState2armaVec(const State *s, arma::vec &q) const {
    //It is suposed that no coupled DOF exist.
    //Each acted DOF has a mapMatrix row with all the elements equal to 0 and
    //only one element equal to 1. The offMatrix element must be equal to 0.5

    unsigned int nDOF = planner_->wkSpace()->getNumRobControls();
    q.resize(nDOF);

    //Convert from ompl state to kautham sample
    Sample smp(nDOF);
    smp.setMappedConf(planner_->initSamp()->getMappedConf());
    planner_->omplState2smp(s,&smp);

    //Convert from kautham sample to armadillo vec
    std::vector<RobConf> conf(smp.getMappedConf());
    RobConf robConf;
    Robot *robot;
    bool found;
    unsigned int k(0), j;

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

            if (found) {//If an element different from 0 has been found
                if (i < 3) {//Base translation DOF
                    q(k) = robConf.getSE3().getPos().at(i);
                } else if (i < 6) {//Base rotation DOF
                    q(k) = robConf.getSE3().getParams().at(i-3);
                } else {//Joint DOF
                    q(k) = robConf.getRn().getCoordinate(i-6);
                }

                k++;
                if (k >= nDOF) break;
            }
        }

        if (k >= nDOF) break;
    }
}

#endif // KAUTHAM_USE_ARMADILLO
