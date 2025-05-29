#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>
#include <kautham/problem/robot.h>  // Used as a pointer to get FK methods inside the constraint methods.
#include <kautham/util/kthutil/kauthamexception.h>

namespace Kautham {

// Constructor implementation
AbstractOMPLConstraint::AbstractOMPLConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints)
    : ompl::base::Constraint(num_dofs, num_constraints), 
    robot_prob_constraint_(_robot_prob_constraint)
{

}


Eigen::AffineCompact3d AbstractOMPLConstraint::ComputeFKFromRobotBaseLinkToEnfEffectorLink(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    // Get the identifiers of the constrained joints:
    std::vector<std::pair<std::string, uint>> constrained_joints = this->robot_prob_constraint_->getConstrainedJoints();

	// Get the robot base link pose wrt world:
	Eigen::AffineCompact3d t_world_2_robot_base_link = this->associated_robot_->getHomeTransformEigen();
	// std::cout << "Home transformation (wrt Kautham world):\n" << t_world_2_robot_base_link.matrix() << std::endl;

	// Get the initial constraint link (offset between the robot base link and the arm FK) pose wrt world:
	std::string init_reference_link = this->associated_robot_->getLink(constrained_joints.at(0).first)->getParentName();
	Eigen::AffineCompact3d t_world_2_constr_init = this->associated_robot_->Robot::getLinkTransformEigen(init_reference_link);
	// std::cout << "Transformation t_world_2_constr_init:\n" <<  t_world_2_constr_init.matrix() << std::endl;
	
	// Change from world to robot base link refence:
	Eigen::AffineCompact3d t_robot_base_link_2_constr_init = t_world_2_robot_base_link.inverse() * t_world_2_constr_init;
	// std::cout << "Transformation t_robot_base_link_2_constr_init:\n" << t_robot_base_link_2_constr_init.matrix() << std::endl;

    // Compute the FK of the constrained joints using the sample of the planner:
    Eigen::AffineCompact3d t_constr_init_2_constr_end = Eigen::AffineCompact3d::Identity();
    Eigen::AffineCompact3d link_fk;
	for (int i = 0; i < q.size(); ++i) {
    	link_fk = this->associated_robot_->getLink(constrained_joints.at(i).first)->Link::applyJointConfiguration(q(i));
        t_constr_init_2_constr_end = t_constr_init_2_constr_end * link_fk;
	}
    // std::cout << "Transformation t_constr_init_2_constr_end:\n" << t_constr_init_2_constr_end.matrix() << std::endl;

    // Get the orientation from the transformation:
	Eigen::AffineCompact3d t_robot_base_link_2_target_link = t_robot_base_link_2_constr_init * t_constr_init_2_constr_end * this->t_constr_end_2_target_link;
    // std::cout << "Transformation t_robot_base_link_2_target_link:\n" << t_robot_base_link_2_target_link.matrix() << std::endl;
    
    return t_robot_base_link_2_target_link;
}


Eigen::AffineCompact3d AbstractOMPLConstraint::getGeometricTransformationWRTRobotBaseLink() const {

	// Get the robot base link pose wrt world:
	Eigen::AffineCompact3d t_world_2_robot_base_link = this->associated_robot_->getHomeTransformEigen();
	// std::cout << "Transformation t_world_2_robot_base_link:\n" << t_world_2_robot_base_link.matrix() << std::endl;
	
	// Get the referenced pose wrt world:
	Eigen::AffineCompact3d t_world_2_referenced_base = this->referenced_entity_->getHomeTransformEigen();
	// std::cout << "Transformation t_world_2_referenced_base:\n" << t_world_2_referenced_base.matrix() << std::endl;

	// Change from world to robot base link refence:
	Eigen::AffineCompact3d t_robot_base_link_2_referenced_base = t_world_2_robot_base_link.inverse() * t_world_2_referenced_base;
	// std::cout << "Transformation t_robot_base_link_2_referenced_base:\n" << t_robot_base_link_2_referenced_base.matrix() << std::endl;

	// Get the geometric origin wrt the referenced base:
	Eigen::AffineCompact3d t_referenced_base_2_geometic_origin = this->robot_prob_constraint_->getReferencedFrameOrigin();
	// std::cout << "Transformation t_referenced_base_2_geometic_origin:\n" << t_referenced_base_2_geometic_origin.matrix() << std::endl;

	// Finally compute the geometric origin wrt the robot base link:
	Eigen::AffineCompact3d t_robot_base_link_2_geometic_origin = t_robot_base_link_2_referenced_base * t_referenced_base_2_geometic_origin;
	// std::cout << "Transformation t_robot_base_link_2_geometic_origin:\n" << t_robot_base_link_2_geometic_origin.matrix() << std::endl;

    return t_robot_base_link_2_geometic_origin;
}


void AbstractOMPLConstraint::initConstraintStuff() {
	this->t_constr_end_2_target_link = AbstractOMPLConstraint::getTransfromationFromConstrEndLink2TargetLink();
	// std::cout << "Transformation t_constr_end_2_target_link:\n" << this->t_constr_end_2_target_link.matrix() << std::endl;
}


Eigen::AffineCompact3d AbstractOMPLConstraint::getTransfromationFromConstrEndLink2TargetLink() {

    Eigen::AffineCompact3d t_constr_end_2_target_link = Eigen::AffineCompact3d::Identity();

	std::string target_link = this->robot_prob_constraint_->getTargetLink();
	std::string last_constr_link = this->robot_prob_constraint_->getLastConstrainedJoint().first;

	// Ensure that the Target Link is either the final link in the constraint chain or comes after it:
	if (target_link == last_constr_link) {
		return t_constr_end_2_target_link;
	} else {
		std::vector<Link*> constr_chain = this->associated_robot_->getLink(last_constr_link)->Link::getChainFromRoot();
		for (Link* link : constr_chain) {
			if (link->getName() == target_link) {
				std::string msg = "Error: The Target Link is not correct.";
				std::string details = "Ensure that the Target Link is either the final link in the constraint chain or comes after it.";
				throw KthExcp(msg,details);
			}
		}
	}

	// Compute the FK from the end of the constraint to the target link:
	std::vector<Link*> target_chain = this->associated_robot_->getLink(target_link)->Link::getChainFrom(last_constr_link);
	Eigen::AffineCompact3d link_fk;
	for (Link* link : target_chain) {
		link_fk = link->Link::applyJointConfiguration(link->Link::getTheta());
		// std::cout << "Transformation link_fk:\n" << link_fk.matrix() << std::endl;
		t_constr_end_2_target_link = t_constr_end_2_target_link * link_fk;
		// std::cout << "Transformation t_constr_end_2_target_link:\n" << t_constr_end_2_target_link.matrix() << std::endl;
	}

	return t_constr_end_2_target_link;
}


bool AbstractOMPLConstraint::project(ompl::base::State *state) const {
	
	(void) state;

	// I don't know why, but this TRUE is the key...
    return true;
}


void AbstractOMPLConstraint::setGoalSampleConfiguration(const std::vector<double> _goal_config) {
	Eigen::Map<const Eigen::VectorXd> mapped_goal_config(_goal_config.data(), _goal_config.size());
	this->goal_sample_t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(mapped_goal_config);
	
	// std::cout << "Constraint: " << this->robot_prob_constraint_->getConstraintId() << std::endl;
	// std::cout << "Transformation in the goal, from robot_base_link 2 end_effector_link:\n" << goal_sample_t_robot_base_link_2_end_effector_link.matrix() << std::endl;
	
	// Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation_matrix = goal_sample_t_robot_base_link_2_end_effector_link.rotation();
    Eigen::Quaterniond quaternion(rotation_matrix);
	std::cout << "Orientation (qx,qy,qz,qw): "
	<< quaternion.x() << ", "
	<< quaternion.y() << ", "
	<< quaternion.z() << ", "
	<< quaternion.w() << std::endl;
}


Eigen::AffineCompact3d AbstractOMPLConstraint::calculatePositionErrorGeometricConstraint(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    Eigen::AffineCompact3d t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(q);
    // std::cout << "Transformation t_robot_base_link_2_end_effector_link:\n" << t_robot_base_link_2_end_effector_link.matrix() << std::endl;

	Eigen::AffineCompact3d t_robot_base_link_2_geometic_origin = getGeometricTransformationWRTRobotBaseLink();
    // std::cout << "Transformation t_robot_base_link_2_geometic_origin:\n" << t_robot_base_link_2_geometic_origin.matrix() << std::endl;

    // Calculate the pose (position + orientation) error:
	Eigen::AffineCompact3d t_error = t_robot_base_link_2_geometic_origin.inverse() * t_robot_base_link_2_end_effector_link;
    // std::cout << "Transformation t_error:\n" << t_error.matrix() << std::endl;

    return t_error;
}


}   // END: namespace Kautham