#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>

// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

namespace Kautham {

// Constructor implementation
OrientationConstraint::OrientationConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, 3)	// Function have 3 outputs.
{
	// Set the orientation tolerance value defined in the problem by the user (or default value if not set):
	this->setTolerance(_robot_prob_constraint->getToleranceValue());
}

// Constraint function implementation, f(q)=0
void OrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
	/*
	The function method in the Constraint class is marked as const.
	Meaning it cannot modify the state of an object (in out case the tolerance).
	So, the logic that updates tolerance has been moved into a separate non-const helper method.
	The use of const_cast temporarily removes the constness from the Constraint object.
	*/
	// Update tolerance in fuction of the distance if it has been defined as variable:
	if (robot_prob_constraint_->isToleranceVariable()) {
		const_cast<OrientationConstraint*>(this)->updateTolerance(q);
    }

    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::Quaterniond error_orientation = calculateOrientationError(q);
    Eigen::AngleAxisd aa(error_orientation);
    Eigen::VectorXd error_ori = aa.axis() * aa.angle();
    // std::cout << "Error orientation as vector: " << error_ori.transpose() << std::endl;

	// Get the info of which axes are allowed to be free and compute the negation (flip 0 -> 1 and 1 -> 0)
	Eigen::Vector3d restricted_movement_axes = Eigen::Vector3d::Ones() - robot_prob_constraint_->getFreeMovementAxes();

	// The constraint could be defined, but not used. That means, is like a unconstrained geometric problem:
	bool constraint_is_used = robot_prob_constraint_->isConstraintOperative();

	// Compute the output of the function:
    out[0] = error_ori[0] * restricted_movement_axes.x() * constraint_is_used;
    out[1] = error_ori[1] * restricted_movement_axes.y() * constraint_is_used;
    out[2] = error_ori[2] * restricted_movement_axes.z() * constraint_is_used;

}

Eigen::Quaterniond OrientationConstraint::calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    Eigen::AffineCompact3d t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(q);

    Eigen::Quaterniond current_orientation(t_robot_base_link_2_end_effector_link.linear());
    // std::cout << "Transformation t_robot_base_link_2_end_effector_link:\n" << t_robot_base_link_2_end_effector_link.matrix() << std::endl;

    // Calculate the orientation error:
    Eigen::Quaterniond error_orientation = current_orientation.inverse() * this->robot_prob_constraint_->getTargetOrientation();
    // std::cout << "Error orientation (x, y, z, w): " << error_orientation.coeffs().transpose() << std::endl;

    return error_orientation;
}

void OrientationConstraint::updateTolerance(const Eigen::Ref<const Eigen::VectorXd> &q) {
	// Get the current pose:
	Eigen::AffineCompact3d current_sample_t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(q);
	// Compute the distance between the current sample and the goal sample:
	Eigen::AffineCompact3d t_goal_sample_2_current_sample = goal_sample_t_robot_base_link_2_end_effector_link.inverse() * current_sample_t_robot_base_link_2_end_effector_link;
	double distance = t_goal_sample_2_current_sample.translation().norm();
	
	// Use this distance to get the new tolerance in that point using the defined tolerance gradient:
	// New tolerance [rad] = Tolerance [rad] + Distance [m] * Gradient [rad/m]:
	double new_tolerance = robot_prob_constraint_->getToleranceValue() + distance * robot_prob_constraint_->getToleranceGradient();
	this->setTolerance(new_tolerance);
}

}	// END: namespace Kautham
