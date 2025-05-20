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

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

	// std::cout << "Box output = [" << delta_length << ", " << delta_width << ", " << delta_height << "]" << std::endl;
    out[0] = delta_length * constraint_is_used;
    out[1] = delta_width * constraint_is_used;
    out[2] = delta_height * constraint_is_used;

}


}	// END: namespace Kautham