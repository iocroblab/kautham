#include <kautham/planner/omplconstr/constraints/sphere_constr.hpp>

namespace Kautham {

// Constructor implementation
SphereConstraint::SphereConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 1)	// Function have 1 output.
{

}

// Constraint function implementation, f(q)=0
void SphereConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionErrorGeometricConstraint(q);

	double radius = robot_prob_constraint_->getGeometricParamRadius();

	double x_error = t_error.translation().x();
    double y_error = t_error.translation().y();
    double z_error = t_error.translation().z();

	// Euclidean distance from the center of the sphere
    double dist = std::sqrt(x_error*x_error + y_error*y_error + z_error*z_error);

	double delta_radius;
    if (dist <= radius) {
        delta_radius = 0; // Inside or on the sphere
    } else {
        delta_radius = dist - radius; // Outside the sphere
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

    out[0] = delta_radius * constraint_is_used;

}


}	// END: namespace Kautham