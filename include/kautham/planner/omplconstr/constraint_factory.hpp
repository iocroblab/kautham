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

/* Author: Pol Ramon Canyameres, Jan Rosell */

#ifndef CONSTRAINT_FACTORY_HPP
#define CONSTRAINT_FACTORY_HPP

#if defined(KAUTHAM_USE_OMPL)

#include <memory>
#include <unordered_map>
#include <string>
#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

namespace Kautham {
// /** \addtogroup ControlPlanners
//  *  @{
//  */

    namespace omplconstrplanner{

        // Factory class
        class ConstraintFactory {
            public:
                using ConstraintBase = Kautham::AbstractOMPLConstraint;
                using ConstraintCreator = std::shared_ptr<ConstraintBase>(*)(std::shared_ptr<Kautham::RobotProblemConstraint>, unsigned int);

                // Function to register all constraints in the factory:
                void registerConstraints();

                // Function to create (get) a constraint by name into the main code:
                std::shared_ptr<ConstraintBase> createConstraint(const std::string& name, std::shared_ptr<Kautham::RobotProblemConstraint> _robot_prob_constraint, unsigned int ambientDim);

                // Print registered constraint names:
                void printRegisteredConstraintNames() const;

            private:
                // Stores all the constraint creators:
                std::unordered_map<std::string, ConstraintCreator> creators_;

                // Function to register a constraint in the factory:
                void registerConstraint(const std::string& name, ConstraintCreator creator);
        };



    }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif // CONSTRAINT_FACTORY_HPP
