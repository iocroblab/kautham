/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#if defined(KAUTHAM_USE_OMPL)

#include <kautham/planner/omplg/omplFOSBKPIECE1planner.h>

#include <kautham/planner/omplg/FOSBKPIECE1.h>
#include <kautham/planner/omplg/omplValidityChecker.h>

#include <ompl/base/ScopedState.h>
#include <kautham/planner/omplg/FOSUpstreamCriterionOptimizationObjective.h>

#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <boost/bind/mem_fn.hpp>
#include <cfloat>


namespace Kautham {
    namespace omplplanner{
        omplFOSBKPIECE1Planner::Projection::Projection(const ompl::base::StateSpacePtr &space,
                                                       Kautham::omplplanner::omplPlanner *pl,
                                                       SynergyTree *st) :
            ompl::base::ProjectionEvaluator(space), pl_(pl), vfdim_(0) {
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

            low = st->getPositionLimits().col(0);
            high = st->getPositionLimits().col(1);
            range = high-low;
            offset = st->getZOS()->b;
            unsigned int n = 0;
            double accum = 0.;
            double total = arma::sum(st->getZOS()->a);
            while (accum < 0.95*total) {
                accum += st->getZOS()->a[n];
                n++;
            }
            n = 2;//!
            arma::mat map = st->getZOS()->U.cols(0,n-1);
            proj = arma::solve(map.t()*map,map.t());// inv(map.t()*map)*map.t()

            std::cout << "FOS BKPIECE Projection" << std::endl;
            proj.print("proj");
            offset.print("offset");
            low.print("low");
            high.print("high");
            range.print("range");
            std::cout << "Reduced number of ZOS synergies " << n << std::endl;

            bounds_.resize(getDimension());
            bounds_.setLow(0.);
            bounds_.setHigh(1.);
        }

        unsigned int omplFOSBKPIECE1Planner::Projection::getDimension() const {
            return proj.n_rows;
        }

        void omplFOSBKPIECE1Planner::Projection::defaultCellSizes() {
            cellSizes_.resize(getDimension());
            for (unsigned int i = 0; i < cellSizes_.size(); ++i) {
                cellSizes_[i] = 0.05+std::numeric_limits<double>::epsilon();
            }
        }

        void omplFOSBKPIECE1Planner::Projection::project(const ompl::base::State *state,
                                                         Kautham::VectorRef projection) const {
            arma::vec q;
            omplState2armaVec(state,q);

            arma::vec u = (q-low)/range;

            arma::vec c = proj*(u-offset)+0.5*arma::ones(proj.n_rows);

            //Clamp c in the range [0,1]
            for (unsigned int i = 0; i < c.n_elem; ++i) {
                c[i] = std::min(std::max(c[i],0.),1.);
            }

            for (unsigned int i = 0; i < getDimension(); ++i) {
                projection(i) = c[i];
            }
        }

        void omplFOSBKPIECE1Planner::Projection::omplState2armaVec(const ompl::base::State *s,
                                                                   arma::vec &q) const {
            q.resize(vfdim_);
            unsigned int k = 0;

            //Convert state to scoped state
            ompl::base::ScopedState<ompl::base::CompoundStateSpace> ss(pl_->getSpace());
            ss = *s;

            //Extract from the ss scoped state the values related to the robot
            ///dofs of base orientation are supposed to be disabled for all the robots!
            for (auto it = robotJoint.begin(); it != robotJoint.end(); ++it) {
                //SE3
                if (!it->second.first.empty()) {
                    ompl::base::ScopedState<ompl::base::SE3StateSpace> ssRobot
                            (pl_->getSpace()->as<ompl::base::CompoundStateSpace>()->
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
                            (pl_->getSpace()->as<ompl::base::CompoundStateSpace>()->
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

        //! Constructor
        omplFOSBKPIECE1Planner::omplFOSBKPIECE1Planner(SPACETYPE stype, Sample *init, Sample *goal,
                                                       SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr,
                                                       const std::string &synergyTreeFilename) :
            omplPlanner(stype, init, goal, samples, ws, ssptr) {
            _guiName = "ompl FOSBKPIECE1 Planner";
            _idName = "omplFOSBKPIECE1";


            //create planner
            st_ = new SynergyTree(synergyTreeFilename);
            ss->getProblemDefinition()->setOptimizationObjective
                    (ob::OptimizationObjectivePtr(new ob::FOSUpstreamCriterionOptimizationObjective(si,this,st_)));

            og::FOSBKPIECE1 *planner = new og::FOSBKPIECE1(si,this,st_);

            ob::ProjectionEvaluatorPtr projEv(new omplFOSBKPIECE1Planner::Projection(space,this,st_));
            projEv->setup();
            space->registerProjection("FOSProjection",projEv);
            planner->setProjectionEvaluator("FOSProjection");
            planner->setup();

            //set planner parameters
            addParameter("Range",planner->getRange());
            addParameter("Tree Bias",planner->getTreeBias());
            addParameter("BorderFraction",planner->getBorderFraction());
            addParameter("FailedExpansionScoreFactor",planner->getFailedExpansionCellScoreFactor());
            addParameter("MinValidPathFraction",planner->getMinValidPathFraction());
            addParameter("Exploration",planner->getExploration());
            addParameter("Initial Lambda",planner->getInitialLambda());
            addParameter("Efficiency Threshold",planner->getEfficiencyThreshold());
            addParameter("Update Frequency",planner->getUpdateFrequency());
            addParameter("Cell Size",projEv->getCellSizes().at(0));
            addParameter("InteriorCellNeighborLimit",planner->getInteriorCellNeighborLimit());

            //set the planner
            ss->setPlanner(ob::PlannerPtr(planner));
        }

        //! void destructor
        omplFOSBKPIECE1Planner::~omplFOSBKPIECE1Planner(){

        }


        //! function to find a solution path
        bool omplFOSBKPIECE1Planner::trySolve()
        {
            if (omplPlanner::trySolve()) {
                //evaluate path
                cout << "path with " << ((og::PathGeometric)ss->getSolutionPath()).getStateCount() << " states" << endl;
                ob::Cost pathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(ss->getProblemDefinition()->getOptimizationObjective());
                cout << "Path cost = " << pathcost.value() << endl;

                return true;
            } else {
                cout<<"No solution found"<<endl;

                return true;
            }
        }


        //! setParameters sets the parameters of the planner
        bool omplFOSBKPIECE1Planner::setParameters(){
            omplPlanner::setParameters();
            try {
                HASH_S_K::iterator it;
                og::FOSBKPIECE1 *planner = ss->getPlanner()->as<og::FOSBKPIECE1>();

                it = _parameters.find("Range");
                if (it == _parameters.end()) return false;
                if (it->second < 0.0) {
                    it->second = planner->getRange();
                } else {
                    planner->setRange(it->second);
                }

                it = _parameters.find("Tree Bias");
                if (it == _parameters.end()) return false;
                if (it->second < 0.0 || it->second > 1.0) {
                    it->second = planner->getTreeBias();
                } else {
                    planner->setTreeBias(it->second);
                }

                it = _parameters.find("BorderFraction");
                if (it == _parameters.end()) return false;
                if (it->second < 0.0 || it->second > 1.0) {
                    it->second = planner->getBorderFraction();
                } else {
                    planner->setBorderFraction(it->second);
                }

                it = _parameters.find("FailedExpansionScoreFactor");
                if (it == _parameters.end()) return false;
                if (it->second < 0) {
                    it->second = planner->getFailedExpansionCellScoreFactor();
                } else {
                    planner->setFailedExpansionCellScoreFactor(it->second);
                }

                it = _parameters.find("MinValidPathFraction");
                if (it == _parameters.end()) return false;
                if (it->second < 0 || it->second > 1.0) {
                    it->second = planner->getMinValidPathFraction();
                } else {
                    planner->setMinValidPathFraction(it->second);
                }

                it = _parameters.find("Initial Lambda");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = planner->getInitialLambda();
                } else {
                    planner->setInitialLambda(it->second);
                }

                it = _parameters.find("Exploration");
                if (it == _parameters.end()) return false;
                if (it->second < 0. || it->second > 1.) {
                    it->second = planner->getExploration();
                } else {
                    planner->setExploration(it->second);
                }

                it = _parameters.find("Update Frequency");
                if (it == _parameters.end()) return false;
                if (it->second < 1) {
                    it->second = planner->getUpdateFrequency();
                } else {
                    planner->setUpdateFrequency(it->second);
                }

                it = _parameters.find("Cell Size");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = planner->getProjectionEvaluator()->getCellSizes().at(0);
                } else {
                    std::vector<double> sizes(planner->getProjectionEvaluator()->getDimension(),it->second);
                    planner->getProjectionEvaluator()->setCellSizes(sizes);
                }

                it = _parameters.find("InteriorCellNeighborLimit");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = planner->getInteriorCellNeighborLimit();
                } else {
                    planner->setInteriorCellNeighborLimit(it->second);
                }
            } catch(...) {
                return false;
            }
            return true;
        }
    }
}


#endif // KAUTHAM_USE_OMPL
