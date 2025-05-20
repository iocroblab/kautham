#include <kautham/planner/omplconstr/constraints/cylinder_constr.hpp>

namespace Kautham {

// Constructor implementation
CylinderConstraint::CylinderConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 2)	// Function have 2 outputs.
{

}

// Constraint function implementation, f(q)=0
void CylinderConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionErrorGeometricConstraint(q);

	double radius = robot_prob_constraint_->getGeometricParamRadius();
	double height = robot_prob_constraint_->getGeometricParamHeight();

	double x_error = t_error.translation().x();
    double y_error = t_error.translation().y();
    double z_error = t_error.translation().z();

	// Radius constraint (x-y plane)
    double dist_xy = std::sqrt(x_error*x_error + y_error*y_error);
    double delta_radius;
    if (dist_xy <= radius) {
        delta_radius = 0;	// Radious constraint fulfilled.
    } else {
		delta_radius = dist_xy - radius;
    }

    // Height constraint (z-axis from base)
    double delta_height;
    if (0 <= z_error && z_error <= height) {
        delta_height = 0;	// Height constraint fulfilled.
    } else if (z_error < 0) {
        delta_height = z_error;  // Below base (delta_height is negative)
    } else {	// z < height
        delta_height = z_error - height;  // Above top (delta_height is positive)
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

	out[0] = delta_radius * constraint_is_used;
	out[1] = delta_height * constraint_is_used;

}


}	// END: namespace Kautham