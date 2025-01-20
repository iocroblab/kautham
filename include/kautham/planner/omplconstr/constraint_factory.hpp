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
#include <ompl/base/Constraint.h>

// Forward declarations of all constraint classes:
class OrientationConstraint;
// class TiagoOrientationConstraint;


namespace Kautham {
// /** \addtogroup ControlPlanners
//  *  @{
//  */

    namespace omplconstrplanner{

        // Factory class
        class ConstraintFactory {
            public:
                using ConstraintBase = ompl::base::Constraint;
                using ConstraintCreator = std::shared_ptr<ConstraintBase>(*)(unsigned int, unsigned int, double);

                // Function to register all constraints in the factory:
                void registerConstraints();

                // Function to create (get) a contraint by name into the main code:
                std::shared_ptr<ConstraintBase> createConstraint(const std::string& name, unsigned int ambientDim, unsigned int coDim, double tolerance);

                // Print registered constraint names:
                void printRegisteredConstraintNames();

            private:
                // Stores all the constraint creators:
                std::unordered_map<std::string, ConstraintCreator> creators_;

                // The registry stores the function pointers for each constraint type
                std::unordered_map<std::string, std::function<void(std::shared_ptr<OrientationConstraint>)>> methodRegistry;


                // Function to register a constraint in the factory:
                void registerConstraint(const std::string& name, ConstraintCreator creator);
        };



    }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif // CONSTRAINT_FACTORY_HPP
