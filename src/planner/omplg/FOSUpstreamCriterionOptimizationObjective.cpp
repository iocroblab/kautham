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


#include <kautham/planner/omplg/FOSUpstreamCriterionOptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>


namespace ompl {
    namespace base {
        FOSUpstreamCriterionOptimizationObjective::FOSUpstreamCriterionOptimizationObjective
        (const ompl::base::SpaceInformationPtr &si, omplplanner::omplPlanner *pl,
         SynergyTree *st) : ompl::base::OptimizationObjective(si), st_(st), vfdim_(0)
        {
            description_ = "FOS Upstream Criterion";

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


        void FOSUpstreamCriterionOptimizationObjective::omplState2armaVec(const ompl::base::State *s,
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


        void FOSUpstreamCriterionOptimizationObjective::armaVec2omplState(const arma::vec &q,
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


        bool FOSUpstreamCriterionOptimizationObjective::isSatisfied(ompl::base::Cost /*c*/) const
        {
            return false;
        }


        Cost FOSUpstreamCriterionOptimizationObjective::stateCost(const State */*s*/) const
        {
            return Cost(0.);
        }


        ompl::base::Cost FOSUpstreamCriterionOptimizationObjective::motionCost(const State *s1,
                                                                               const State *s2) const
        {
            // Per equation 1 in the paper, Riemann approximation on the left
            arma::vec q1;
            arma::vec q2;
            omplState2armaVec(s1,q1);
            omplState2armaVec(s2,q2);
            arma::vec qprime = arma::normalise(q2-q1);

            unsigned int numSegments = si_->getStateSpace()->validSegmentCount(s1, s2);
            double cost = 0;
            for (unsigned int i = 0; i < numSegments; ++i) {
                arma::vec q = q1 + (q2-q1)*double(i)/double(numSegments);

                arma::vec f = st_->vectorField(q);
                cost += arma::norm(f)-arma::dot(f,qprime);

                //for (unsigned int k = 0; k < 100; ++k) {
                //    cost += (1.-arma::dot(st_->vectorField(q),qprime));// ||f|| must be 1
                //}
            }

            //return ompl::base::Cost(cost/100.*si_->distance(s2,s1));
            return ompl::base::Cost(cost*si_->distance(s2,s1));
        }
    }
}
