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

    // Get the allowed region (inside or outside) from the constraint
    auto allowed_region = robot_prob_constraint_->getAllowedVolumeRegion();

    double dist_xy = std::sqrt(x_error * x_error + y_error * y_error);
    double delta_radius = 0.0;
    double delta_height = 0.0;

    if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Inside) {
        // Penalize points outside the cylinder's radius
        if (dist_xy > radius) {
            delta_radius = dist_xy - radius;
        } else {
            delta_radius = 0.0;
        }
        // Penalize points outside the height limits
        if (z_error < 0) {
            delta_height = z_error;
        } else if (z_error > height) {
            delta_height = z_error - height;
        } else {
            delta_height = 0.0;
        }
    } else if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Outside) {
        // Only penalize if inside both radius and height bounds
        bool inside_radius = (dist_xy < radius);
        bool inside_height = (0 <= z_error) && (z_error <= height);
        if (inside_radius && inside_height) {
            // Inside the cylinder: penalize
            delta_radius = -(radius - dist_xy);
            double dist_to_base = std::abs(z_error);
            double dist_to_top = std::abs(z_error - height);
            delta_height = -std::min(dist_to_base, dist_to_top);
        } else {
            // Outside the cylinder: constraint satisfied
            delta_radius = 0.0;
            delta_height = 0.0;
        }
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

	out[0] = delta_radius * constraint_is_used;
	out[1] = delta_height * constraint_is_used;

}


}	// END: namespace Kautham