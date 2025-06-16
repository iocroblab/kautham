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

    double delta_radius = 0.0;
    double delta_height = 0.0;

    // Get allowed region (inside or outside)
    auto allowed_region = robot_prob_constraint_->getAllowedVolumeRegion();

    // Calculate radial distance in XY plane
    double dist_xy = std::sqrt(x_error * x_error + y_error * y_error);

    // Calculate max allowed radius at this z
    double max_radius_at_z = (z_error / height) * radius; // Only meaningful if 0 <= z <= height

    if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Inside) {
        // Height constraint
        if (z_error < 0) {
            delta_height = z_error; // Below the tip
        } else if (z_error > height) {
            delta_height = z_error - height; // Above the base
        } else {
            delta_height = 0.0; // Within height range
        }

        // Radius constraint (only if within height)
        if (0 <= z_error && z_error <= height) {
            if (dist_xy > max_radius_at_z) {
                delta_radius = dist_xy - max_radius_at_z;
            } else {
                delta_radius = 0.0;
            }
        } else {
            delta_radius = 0.0; // Outside height, radius constraint not relevant
        }
    } 
    else if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Outside) {
        // Only penalize if inside cone (both height and radius)
        bool inside_height = (z_error >= 0) && (z_error <= height);
        bool inside_radius = false;
        if (inside_height) {
            inside_radius = (dist_xy < max_radius_at_z);
        }
        if (inside_height && inside_radius) {
            // Inside the cone: penalize (negative value)
            delta_height = -std::min(std::abs(z_error), std::abs(z_error - height));
            delta_radius = -(max_radius_at_z - dist_xy);
        } else {
            // Outside cone: constraint satisfied
            delta_height = 0.0;
            delta_radius = 0.0;
        }
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

    out[0] = delta_radius * constraint_is_used;
    out[1] = delta_height * constraint_is_used;

}

}	// END: namespace Kautham