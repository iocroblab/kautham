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


#ifndef FIRST_ORDER_SYNERGY_OPTIMIZATION_OBJECTIVE_H
#define FIRST_ORDER_SYNERGY_OPTIMIZATION_OBJECTIVE_H

#ifdef KAUTHAM_USE_ARMADILLO

#include <ompl/base/OptimizationObjective.h>
#include <kautham/planner/omplg/synergy_tree.h>
#include <kautham/planner/omplg/omplplanner.h>


namespace ompl {
    namespace base {
        class FOSOptimizationObjective : public ompl::base::OptimizationObjective {
        public:
            FOSOptimizationObjective(Kautham::omplplanner::omplPlanner *planner,
                                     const SpaceInformationPtr &si);

            bool setSynergyTree(std::string filename) {
                delete tree_;
                try {
                    tree_ = new SynergyTree(filename);
                } catch(std::exception &excp) {
                    std::cout << excp.what() << std::endl;
                    tree_ = NULL;
                    return false;
                }
                return true;
            }

            void setSynergyTree(SynergyTree *tree) {
                delete tree_;
                tree_ = tree;
            }

            SynergyTree *getSynergyTree() const {
                return tree_;
            }

            virtual Cost stateCost(const State *s) const;

            virtual Cost motionCost(const State *s1, const State *s2) const;

            Cost preSolveMotionCost(const State *s1, const State *s2) const;

            double boxZosDistance(const State *s) const;

        protected:
            void omplState2armaVec(const State *s, arma::vec &q) const;

            Kautham::omplplanner::omplPlanner *planner_;

            SynergyTree *tree_;
        };
    }
}

#endif // KAUTHAM_USE_ARMADILLO

#endif // FIRST_ORDER_SYNERGY_OPTIMIZATION_OBJECTIVE_H
