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


#include "ompl/geometric/planners/rrt/RRT.h"
#include <kautham/planner/omplg/synergy_tree.h>
#include <kautham/planner/omplg/omplplanner.h>


namespace ompl {
    namespace geometric {
        class FOSRRT : public ompl::geometric::RRT {
        public:
            FOSRRT(const ompl::base::SpaceInformationPtr &si, Kautham::omplplanner::omplPlanner *planner)
                : RRT(si),planner_(planner),nDOF_(planner->wkSpace()->getNumRobControls()) {
                name_ = "FOSRRT";
                tree_ = NULL;
                timeStep_ = 1.;
                pmdBias_ = 1.;
                declareParam<double>("timeStep",this,&FOSRRT::setTimeStep,&FOSRRT::getTimeStep,"0.:.001:1000.");
                declareParam<double>("pmdBias",this,&FOSRRT::setPMDbias,&FOSRRT::getPMDbias,"0.:.01:1.");
            }

            void setTimeStep(double timeStep) {
                timeStep_ = timeStep;
            }

            double getTimeStep() const {
                return timeStep_;
            }

            void setPMDbias(double pmdBias) {
                pmdBias_ = pmdBias;
            }

            double getPMDbias() const {
                return pmdBias_;
            }

            void setSynergyTree(SynergyTree * tree) {
                tree_ = tree;
            }

            SynergyTree *getSynergyTree() const {
                return tree_;
            }

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

        protected:
            //! New qRand
            arma::vec new_qRand(arma::vec qr, arma::vec qn);

            void omplState2armaVec(const ob::State *state, arma::vec &vector);

            void armaVec2omplState(const arma::vec vector, ob::State *state);

            SynergyTree *tree_;

            double timeStep_;

            double pmdBias_;

            Kautham::omplplanner::omplPlanner *planner_;

            unsigned int nDOF_;
        };
    }
}
