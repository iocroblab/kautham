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

// Include all the constraints:
#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>

namespace Kautham {
    namespace omplconstrplanner{

        // Returns whether the given state's position overlaps the obstacles
        bool ValidityChecker::isValid(const ob::State* state) const
        {
            // Verify bounds
            if (thesi->satisfiesBounds(state)==false) {
                return false;
            }
            // Create sample
            int d = theplanner->wkSpace()->getNumRobControls();
            Sample *smp = new Sample(d);
            // auto smp = std::make_unique<Sample>(theplanner->wkSpace()->getNumRobControls());
            // copy the conf of the init smp. Needed to capture the home positions.
            smp->setMappedConf(theplanner->initSamp()->getMappedConf());
            
            //load the RobConf of smp form the values of the ompl::state
            ((omplConstraintPlanner*)theplanner)->omplConstraintPlanner::omplState2smp(state,smp);
            
            // Collision-check:
            if (theplanner->wkSpace()->collisionCheck(smp)) {
                delete smp;
                return false;
            }

            // If the bool is false, the method is not evaluated (efficient).
            if (((omplConstraintPlanner*)theplanner)->have_constr_space && !isConstrainedPartValid(state)) {
                delete smp;
                return false;
            }

            // Discards the sample.
            // By default does nothing, i.e. the default filtersample function in class Planner returns always false
            if (theplanner->filtersample(smp)) {
                delete smp;
                return false;
            }

            delete smp;
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


        bool ValidityChecker::isConstrainedPartValid(const ob::State* state) const
        {
            for (const auto& [name, constraint] : ((omplConstraintPlanner*)theplanner)->constraint_map_) {
                if (constraint) {
                    try {
                        const auto& stateSpace = ((omplConstraintPlanner*)theplanner)->StateSpace();
                        if (!stateSpace) {
                            std::cerr << "Error: StateSpace is null" << std::endl;
                            continue;
                        }

                        const auto& compoundSpace = stateSpace->as<ob::CompoundStateSpace>();
                        if (!compoundSpace) {
                            std::cerr << "Error: Failed to cast to CompoundStateSpace" << std::endl;
                            continue;
                        }

                        const std::vector<ob::StateSpacePtr>& robotSpaces = compoundSpace->getSubspaces();

                        for (size_t i = 0; i < robotSpaces.size(); ++i) {
                            if (auto compoundSubSpace = std::dynamic_pointer_cast<ob::CompoundStateSpace>(robotSpaces[i])) {
                                const std::vector<ob::StateSpacePtr>& subspaces = compoundSubSpace->getSubspaces();
                                for (size_t j = 0; j < subspaces.size(); ++j) {
                                    std::string subspaceName = subspaces[j]->getName();
                                    if (subspaceName.find("_RnConstr_") != std::string::npos) {
                                        auto rnConstrSpace = subspaces[j]->as<ob::ProjectedStateSpace>();
                                        if (!rnConstrSpace) {
                                            std::cerr << "Error: Failed to cast to ProjectedStateSpace for " << subspaceName << std::endl;
                                            continue;
                                        }

                                        // Access the specific substate for this ProjectedStateSpace
                                        const ob::CompoundState* compoundState = state->as<ob::CompoundState>();
                                        const ob::CompoundState* robotState = compoundState->as<ob::CompoundState>(i);
                                        const ob::State* projectedSubState = robotState->as<ob::State>(j);
                                        
                                        if (!constraint->isSatisfied(projectedSubState)) {
                                            // std::cout << "  " << subspaceName << ": is NOT satisfied.";
                                            return false;
                                        }
                                    }
                                }
                            } else {
                                std::cout << "... (similar error checking for non-compound subspaces)" << std::endl;
                            }
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    }
                } else {
                    std::cout << "Null constraint." << std::endl;
                }
            }
            return true;
        }

    }
}


#endif // KAUTHAM_USE_OMPL
