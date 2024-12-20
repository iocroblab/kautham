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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */



#if defined(KAUTHAM_USE_OMPL)
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <kautham/planner/omplconstr/omplconstrValidityChecker.hpp>
#include <kautham/planner/omplconstr/omplconstrplanner.hpp>
#include <cfloat>

namespace Kautham {
    namespace omplconstrplanner{

        // Returns whether the given state's position overlaps the obstacles
        bool ValidityChecker::isValid(const ob::State* state) const
        {
            //verify bounds
            if (thesi->satisfiesBounds(state)==false) {
                return false;
            }
            //create sample
            int d = theplanner->wkSpace()->getNumRobControls();
            Sample *smp = new Sample(d);
            // auto smp = std::make_unique<Sample>(theplanner->wkSpace()->getNumRobControls());
            //copy the conf of the init smp. Needed to capture the home positions.
            smp->setMappedConf(theplanner->initSamp()->getMappedConf());
            //load the RobConf of smp form the values of the ompl::state
            ((omplConstraintPlanner*)theplanner)->omplConstraintPlanner::omplState2smp(state,smp);
            //collision-check
            if (theplanner->wkSpace()->collisionCheck(smp)) {
                return false;
            }

            // If the bool is false, the method is not evaluated (efficient).
            if (((omplConstraintPlanner*)theplanner)->have_constr_space && !isConstrainedPartValid(state, smp)) {
                std::cout << "have_constr_space" << std::endl;
                
                
                // return false;
            }

            // If the bool is false, the method is not evaluated (efficient).
            // if (((omplConstraintPlanner*)theplanner)->have_unconstr_space && !isUnconstrainedPartValid(state, smp)) {
            //     std::cout << "have_unconstr_space" << std::endl;
                // return false;
            // }


            //Discards the sample.
            //By default does nothing, i.e. the default filtersample function in class Planner
            //returns always false
            if (theplanner->filtersample(smp)) {
                return false;
            }
            return true;
        }

        // Returns the distance from the given state's position to the obstacles
        double ValidityChecker::clearance(const ob::State* state) const
        {
            //verify bounds
            if (thesi->satisfiesBounds(state)==false) {
                return false;
            }
            //create sample
            int d = theplanner->wkSpace()->getNumRobControls();
            Sample *smp = new Sample(d);
            // auto smp = std::make_unique<Sample>(theplanner->wkSpace()->getNumRobControls());
            //copy the conf of the init smp. Needed to capture the home positions.
            smp->setMappedConf(theplanner->initSamp()->getMappedConf());
            //load the RobConf of smp form the values of the ompl::state
            ((omplConstraintPlanner*)theplanner)->omplConstraintPlanner::omplState2smp(state,smp);
            //distance-check
            vector<double> *distvect;
            distvect = theplanner->wkSpace()->distanceCheck(smp);
            double dist = FLT_MAX;
            for (unsigned i=0; i<distvect->size(); i++) {
                if (dist>distvect->at(i)) {
                    dist = distvect->at(i);
                }
            }
            return dist;
        }

        bool ValidityChecker::isConstrainedPartValid(const ob::State* state, const Sample* smp) const
        {
            // const auto* compound_state = state->as<ob::CompoundState>();
            // for (unsigned int i = 0; i < compound_state->getSubstateCount(); ++i) {
            //     const auto* subspace = thesi->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(i);
            //     if (subspace->getName().find("RnConstr") != std::string::npos) {
            //         if (!subspace->satisfiesBounds(compound_state->as<ob::CompoundState>()->getSubstate(i))) {
            //             return false;
            //         }
            //     }
            // }

            for (const auto& [name, constraint] : ((omplConstraintPlanner*)theplanner)->constraint_map_) {
                std::cout << "Constraint name: " << name << std::endl;
                if (constraint) {
                    // constraint->printInfo();
                    std::cout << "State: ";
                    ((omplConstraintPlanner*)theplanner)->StateSpace()->as<ob::CompoundStateSpace>()->printState(state, std::cout);
                    std::cout << std::endl;
                    // std::cout << "Is satisfied: " << constraint->isSatisfied(state) << std::endl;
                } else {
                    std::cout << "Null constraint." << std::endl;
                }
            }

            return true;
        }



    }
}


#endif // KAUTHAM_USE_OMPL
