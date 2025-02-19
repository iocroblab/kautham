#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

// Constructor implementation
AbstractOMPLConstraint::AbstractOMPLConstraint(const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : ompl::base::Constraint(num_dofs, num_constraints, tolerance)
    , num_dofs_(num_dofs), num_constraints_(num_constraints), tolerance_(tolerance)
{

}
