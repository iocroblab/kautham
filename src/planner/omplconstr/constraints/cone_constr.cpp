#include <kautham/planner/omplconstr/constraints/cone_constr.hpp>

namespace Kautham {

// Constructor implementation
ConeConstraint::ConeConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 2)	// Function have 2 outputs.
{

}


// Constraint function implementation, f(q)=0
void ConeConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionErrorGeometricConstraint(q);

	double radius = robot_prob_constraint_->getGeometricParamRadius();
	double height = robot_prob_constraint_->getGeometricParamHeight();

	double x_error = t_error.translation().x();
    double y_error = t_error.translation().y();
    double z_error = t_error.translation().z();

	// Height constraint (vertex at origin, base at z=height)
    double delta_height;
    if (z_error < 0) {
        delta_height = z_error; // Below the tip (vertex)
    } else if (z_error > height) {
        delta_height = z_error - height; // Above the base
    } else {
        delta_height = 0; // Within height range
    }

    // Max allowed radius at this z
    double max_radius_at_z = (z_error / height) * radius;
    double dist_xy = std::sqrt(x_error * x_error + y_error * y_error);

    double delta_radius;
    if (z_error < 0 || height < z_error) {
        delta_radius = 0; // Outside height, ignore radius constraint -> Will be computed only when z fulfills.
    } else if (dist_xy <= max_radius_at_z) {
        delta_radius = 0; // Inside cone at this z
    } else {
        delta_radius = dist_xy - max_radius_at_z; // Outside cone at this z
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

    out[0] = delta_radius * constraint_is_used;
    out[1] = delta_height * constraint_is_used;

}

}	// END: namespace Kautham