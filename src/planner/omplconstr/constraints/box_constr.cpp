#include <kautham/planner/omplconstr/constraints/box_constr.hpp>

namespace Kautham {

// Constructor implementation
BoxConstraint::BoxConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 3)	// Function have 3 outputs.
{

}

// Constraint function implementation, f(q)=0
void BoxConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionErrorGeometricConstraint(q);

	double x_error = t_error.translation().x();
	double y_error = t_error.translation().y();
	double z_error = t_error.translation().z();

	double length = robot_prob_constraint_->getGeometricParamLength();
	double width = robot_prob_constraint_->getGeometricParamWidth();
	double height = robot_prob_constraint_->getGeometricParamHeight();

	double half_length = length / 2.0;
    double half_width  = width  / 2.0;
    double half_height = height / 2.0;

	double delta_length = 0.0;
    double delta_width  = 0.0;
    double delta_height = 0.0;

	// Get allowed region (inside or outside)
    auto allowed_region = robot_prob_constraint_->getAllowedVolumeRegion();

    // Assume that the box origin is in the mid:
    // For "inside": penalize if outside on any axis.
    // For "outside": penalize only if inside on all axes; otherwise, constraint is satisfied (0).

    if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Inside) {
        // Penalize points outside the box along each axis
        
        if (x_error < -half_length) {   // X (length)
            delta_length = x_error + half_length;
        } else if (x_error > half_length) {
            delta_length = x_error - half_length;
        } else {
            delta_length = 0.0;
        }

        if (y_error < -half_width) {    // Y (width)
            delta_width = y_error + half_width;
        } else if (y_error > half_width) {
            delta_width = y_error - half_width;
        } else {
            delta_width = 0.0;
        }

        if (z_error < -half_height) {   // Z (height)
            delta_height = z_error + half_height;
        } else if (z_error > half_height) {
            delta_height = z_error - half_height;
        } else {
            delta_height = 0.0;
        }
    } else if (allowed_region == Kautham::RobotProblemConstraint::AllowedVolumeRegion::Outside) {
        // Determine if point is inside on all axes:
        bool inside_x = (x_error >= -half_length) && (x_error <= half_length);
        bool inside_y = (y_error >= -half_width)  && (y_error <= half_width);
        bool inside_z = (z_error >= -half_height) && (z_error <= half_height);

        if (inside_x && inside_y && inside_z) {
            // Inside the box: penalize based on min distance to each face
            double dist_to_x = std::min(std::abs(x_error + half_length), std::abs(x_error - half_length));
            double dist_to_y = std::min(std::abs(y_error + half_width), std::abs(y_error - half_width));
            double dist_to_z = std::min(std::abs(z_error + half_height), std::abs(z_error - half_height));
            delta_length = -dist_to_x;
            delta_width  = -dist_to_y;
            delta_height = -dist_to_z;
        } else {
            // Outside the box: constraint satisfied
            delta_length = 0.0;
            delta_width  = 0.0;
            delta_height = 0.0;
        }
    }

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

	// std::cout << "Box output = [" << delta_length << ", " << delta_width << ", " << delta_height << "]" << std::endl;
    out[0] = delta_length * constraint_is_used;
    out[1] = delta_width * constraint_is_used;
    out[2] = delta_height * constraint_is_used;

}


}	// END: namespace Kautham