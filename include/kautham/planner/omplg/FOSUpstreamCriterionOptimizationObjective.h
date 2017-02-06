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

#ifndef OMPL_BASE_OBJECTIVES_FOS_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_FOS_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_

#include <ompl/base/OptimizationObjective.h>
#include <armadillo>
#include <kautham/planner/omplg/synergy_tree.h>
#include "omplplanner.h"

namespace ompl
{
    namespace base
    {

        /**
         * Optimization objective that computes the upstream criterion between two states.
         */
        class FOSUpstreamCriterionOptimizationObjective : public ompl::base::OptimizationObjective
        {

        public:

            /** Constructor. */
            FOSUpstreamCriterionOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                      Kautham::omplplanner::omplPlanner *pl,
                                                      SynergyTree *st);

            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost c) const;

            /** \brief Returns a cost with a value of 0. */
            virtual Cost stateCost(const State *s) const;

            /** Compute upstream criterion between two states. */
            ompl::base::Cost motionCost(const State *s1, const State *s2) const;

            bool isSymmetric(void) const
            {
                return false;
            }

        protected:
            void omplState2armaVec(const ompl::base::State *s, arma::vec &q) const;

            void armaVec2omplState(const arma::vec &q, ompl::base::State *s) const;

            /** SynergyTree associated with the space. */
            SynergyTree *st_;

            unsigned int vfdim_;

            std::map<unsigned int,std::pair<std::set<unsigned  int>,std::set<unsigned  int> > > robotJoint;
        };

    }
}

#endif
