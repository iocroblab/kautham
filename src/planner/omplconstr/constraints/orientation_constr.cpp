#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>
#include <kautham/problem/robot.h>  // Used as a pointer to get FK methods inside the constraint methods.

// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

// Constructor implementation
OrientationConstraint::OrientationConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : AbstractOMPLConstraint(_robot_prob_constraint, num_dofs, num_constraints, tolerance)
{

}

void OrientationConstraint::useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) {
	std::cout << "OrientationConstraint: useJointConfig2SetConstraintTarget() method." << std::endl;
	// Get the transformation of the TCP w.r.t. the base of the UR5:
    // Eigen::AffineCompact3d transformation_base_tool0;
    // evaluateFKUR5(transformation_base_tool0, 
    //                 joint_config[0], 
    //                 joint_config[1], 
    //                 joint_config[2], 
    //                 joint_config[3], 
    //                 joint_config[4], 
    //                 joint_config[5]);

// 	this->robot_prob_constraint_->setTargetOrientation(Eigen::Quaterniond(transformation_base_tool0.rotation()));
}

void OrientationConstraint::printConstraintTarget() const {
	std::cout << "Target Orientation (Quaternion: qx qy qz qw): "
			<< this->robot_prob_constraint_->getTargetOrientation().coeffs().transpose() << std::endl;
}

bool OrientationConstraint::project(ompl::base::State *state) const {
	
	(void) state;

	// I don't know why, but this TRUE is the key...
    return true;
}

// Constraint function implementation, f(q)=0
void OrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const {
    
    // Note: 'q' is not the current joint values, is the sampled joint values by the OMPL.
    // std::cout << "q = " << q.transpose() << std::endl;
     
    Eigen::Quaterniond error_orientation = calculateOrientationError(q);
    Eigen::AngleAxisd aa(error_orientation);
    Eigen::VectorXd error_ori = aa.axis() * aa.angle();
    // std::cout << "Error orientation as vector: " << error_ori.transpose() << std::endl;

    out[0] = error_ori[0];
    out[1] = error_ori[1];
    out[2] = error_ori[2];

}

// Jacobian of the constraint function implementation
void OrientationConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::MatrixXd> out) const {

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

Eigen::Quaterniond OrientationConstraint::calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    Eigen::AffineCompact3d t_robot_base_link_2_end_effector_link = ComputeFKFromRobotBaseLinkToEnfEffectorLink(q);

    Eigen::Quaterniond current_orientation(t_robot_base_link_2_end_effector_link.linear());
    // std::cout << "Transformation t_robot_base_link_2_end_effector_link:\n" << t_robot_base_link_2_end_effector_link.matrix() << std::endl;

    // Calculate the orientation error:
    Eigen::Quaterniond error_orientation = current_orientation.inverse() * this->robot_prob_constraint_->getTargetOrientation();
    // std::cout << "Error orientation (x, y, z, w): " << error_orientation.coeffs().transpose() << std::endl;

    return error_orientation;
}


Eigen::AffineCompact3d OrientationConstraint::ComputeFKFromRobotBaseLinkToEnfEffectorLink(const Eigen::Ref<const Eigen::VectorXd>& q) const {

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