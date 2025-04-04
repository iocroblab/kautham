#include <kautham/planner/omplconstr/constraints/box_constr.hpp>

namespace Kautham {

// Constructor implementation
BoxConstraint::BoxConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, num_constraints, tolerance)
{

}

void BoxConstraint::useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) {
    // Provide implementation here
}

void BoxConstraint::printConstraintTarget() const {
    // Provide implementation here
}


bool BoxConstraint::project(ompl::base::State *state) const {
	
	(void) state;

	// I don't know why, but this TRUE is the key...
    return true;
}

// Constraint function implementation, f(q)=0
void BoxConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::AffineCompact3d t_error = calculatePositionError(q);

	double length = robot_prob_constraint_->getGeometricParamLength();
	double width = robot_prob_constraint_->getGeometricParamWidth();
	double height = robot_prob_constraint_->getGeometricParamHeight();

	double delta_length, delta_width, delta_height;

	// Assume that the box origin is in the mid:
	// std::signbit returns true if is a negative value. Is needed to help the jacobian in which direction must be to move.
	delta_length = length/2.0 - std::abs(t_error.translation().x());
	if (std::abs(t_error.translation().x()) < length/2.0) {
		delta_length = 0;
	} else {
		if (std::signbit(t_error.translation().x())) {
			delta_length = -delta_length;
		}
	}

	delta_width = width/2.0 - std::abs(t_error.translation().y());
	if (std::abs(t_error.translation().y()) < width/2.0) {
		delta_width = 0;
	} else {
		if (std::signbit(t_error.translation().y())) {
			delta_width = -delta_width;
		}
	}


	delta_height = height/2.0 - std::abs(t_error.translation().z());
	if (std::abs(t_error.translation().z()) < height/2.0) {
		delta_height = 0;
	} else {
		if (std::signbit(t_error.translation().z())) {
			delta_height = -delta_height;
		}
	}

	// std::cout << "Box output = [" << delta_length << ", " << delta_width << ", " << delta_height << "]" << std::endl;
    out[0] = delta_length;
    out[1] = delta_width;
    out[2] = delta_height;

}

// Jacobian of the constraint function implementation
void BoxConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::MatrixXd> out) const {

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

Eigen::AffineCompact3d BoxConstraint::calculatePositionError(const Eigen::Ref<const Eigen::VectorXd>& q) const {

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