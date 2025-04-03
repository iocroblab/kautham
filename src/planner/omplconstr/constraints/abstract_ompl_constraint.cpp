#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>
#include <kautham/problem/robot.h>  // Used as a pointer to get FK methods inside the constraint methods.

// Constructor implementation
AbstractOMPLConstraint::AbstractOMPLConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : ompl::base::Constraint(num_dofs, num_constraints, tolerance), 
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

	// ToDo: Get the orientation link post-constraint offset:
    Eigen::AffineCompact3d t_constr_end_2_end_effector_link = Eigen::AffineCompact3d::Identity();
	// std::string orilink = this->robot_prob_constraint_->getOrientationLink();
	// std::cout << "OriLink: " << orilink << std::endl;

    // Get the orientation from the transformation:
	Eigen::AffineCompact3d t_robot_base_link_2_end_effector_link = t_robot_base_link_2_constr_init * t_constr_init_2_constr_end * t_constr_end_2_end_effector_link;
    // std::cout << "Transformation t_robot_base_link_2_end_effector_link:\n" << t_robot_base_link_2_end_effector_link.matrix() << std::endl;
    
    return t_robot_base_link_2_end_effector_link;
}