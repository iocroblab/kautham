#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

// Constructor implementation
AbstractOMPLConstraint::AbstractOMPLConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : ompl::base::Constraint(num_dofs, num_constraints, tolerance), 
    robot_prob_constraint_(_robot_prob_constraint),
    num_dofs_(num_dofs), num_constraints_(num_constraints), tolerance_(tolerance)
{

}
