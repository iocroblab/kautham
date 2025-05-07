#include <kautham/planner/omplconstr/constraints/cylinder_constr.hpp>

namespace Kautham {

// Constructor implementation
CylinderConstraint::CylinderConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 2)	// Function have 2 outputs.
{

}

void CylinderConstraint::useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) {
    // Provide implementation here
}

void CylinderConstraint::printConstraintTarget() const {
    // Provide implementation here
}


bool CylinderConstraint::project(ompl::base::State *state) const {
	
	(void) state;

	// I don't know why, but this TRUE is the key...
    return true;
}

// Constraint function implementation, f(q)=0
void CylinderConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionError(q);

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

	out[0] = delta_radius;
	out[1] = delta_height;

}

// Jacobian of the constraint function implementation
void CylinderConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::MatrixXd> out) const {

	// std::cout << "JACOBIAN: " << std::endl;
	Eigen::VectorXd y1 = q;
	Eigen::VectorXd y2 = q;
	Eigen::VectorXd t1(getCoDimension());
	Eigen::VectorXd t2(getCoDimension());

	// Use a 7-point central difference stencil on each column.
	for (std::size_t j = 0; j < n_; j++)
	{
		const double ax = std::fabs(q[j]);
		// Make step size as small as possible while still giving usable accuracy.
		const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);

		// Can't assume y1[j]-y2[j] == 2*h because of precision errors.
		y1[j] += h;
		y2[j] -= h;
		function(y1, t1);
		function(y2, t2);
		const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
		y1[j] += h;
		y2[j] -= h;
		function(y1, t1);
		function(y2, t2);
		const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
		y1[j] += h;
		y2[j] -= h;
		function(y1, t1);
		function(y2, t2);
		const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

		out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;

		// Reset for next iteration.
		y1[j] = y2[j] = q[j];
	}

}

Eigen::AffineCompact3d CylinderConstraint::calculatePositionError(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    Eigen::AffineCompact3d t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(q);
    // std::cout << "Transformation t_robot_base_link_2_end_effector_link:\n" << t_robot_base_link_2_end_effector_link.matrix() << std::endl;

	Eigen::AffineCompact3d t_robot_base_link_2_geometic_origin = getGeometricTransformationWRTRobotBaseLink();
    // std::cout << "Transformation t_robot_base_link_2_geometic_origin:\n" << t_robot_base_link_2_geometic_origin.matrix() << std::endl;

    // Calculate the pose (position + orientation) error:
	Eigen::AffineCompact3d t_error = t_robot_base_link_2_geometic_origin.inverse() * t_robot_base_link_2_end_effector_link;
    // std::cout << "Transformation t_error:\n" << t_error.matrix() << std::endl;

    return t_error;
}


}	// END: namespace Kautham