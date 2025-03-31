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
    Eigen::AffineCompact3d transformation_base_tool0;
    evaluateFKUR5(transformation_base_tool0, 
                    joint_config[0], 
                    joint_config[1], 
                    joint_config[2], 
                    joint_config[3], 
                    joint_config[4], 
                    joint_config[5]);

	this->robot_prob_constraint_->setTargetOrientation(Eigen::Quaterniond(transformation_base_tool0.rotation()));
}

void OrientationConstraint::printConstraintTarget() const {
	std::cout << "Target Orientation (Quaternion: qx qy qz qw): "
			<< this->robot_prob_constraint_->getTargetOrientation().coeffs().transpose() << std::endl;
}

bool OrientationConstraint::project(ompl::base::State *state) const {
	
	(void) state;

	// std::cout << "CUSTOM PROJECT" << std::endl;
	// // Cast to ProjectedStateSpace::StateType
    // const auto *constrained_state = state->as<ompl::base::ProjectedStateSpace::StateType>();
    
    // if (constrained_state) {
    //     std::cout << "Projected State:" << std::endl;
        
    //     // Get the underlying state
	// 	const auto* constrainedRealVector_state = constrained_state->getState()->as<ompl::base::RealVectorStateSpace::StateType>();
        
	// 	if (constrainedRealVector_state) {
	// 		std::cout << "Num DOF = " << this->num_dofs_ << std::endl;
	// 		if (constrainedRealVector_state->values == nullptr) {
    //             std::cout << "Error: constrainedRealVector_state->values is null" << std::endl;
    //             return false;
    //         } else {
	// 			for (unsigned int i = 0; i < this->num_dofs_; ++i) {
	// 				std::cout << "  Dimension " << i << ": " << &(constrainedRealVector_state->values[i]) << std::endl;
	// 				std::cout << "  Dimension " << i << ": " << (constrainedRealVector_state->values[i]) << std::endl;
	// 			}
	// 		}
	// 	} else {
	// 		std::cout << "Error: Unable to cast state to RealVectorStateSpace::StateType" << std::endl;
	// 	}

    // } else {
    //     std::cout << "Error: Unable to cast state to ProjectedStateSpace::StateType" << std::endl;
    // }

	// I don't know why, but this TRUE is the key...
    return true;
}

// bool OrientationConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const
//  {
// 	std::cout << "NEW PROJECT" << std::endl;
//     std::cout << "v_x = " << x.transpose() << std::endl;

// 	// Newton's method
// 	unsigned int iter = 0;
// 	double norm = 0;
// 	Eigen::VectorXd f(getCoDimension());
// 	Eigen::MatrixXd j(getCoDimension(), n_);

// 	const double squaredTolerance = tolerance_ * tolerance_;

// 	size_t count = 0;

// 	function(x, f);
// 	while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
// 	{
// 		jacobian(x, j);
// 		x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
// 		std::cout << "J_x = " << x.transpose() << std::endl;
// 		function(x, f);
// 		std::cout << "Count = " << count << std::endl;
// 		count++;
// 	}

// 	return norm < squaredTolerance;
//  }


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

	// std::cout << "TESTING:" << std::endl;
	// Eigen::AffineCompact3d t_base_link = this->associated_robot_->getHomeTransformEigen();
	// std::cout << "Home transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_base_link.matrix() << std::endl;

	// Eigen::AffineCompact3d t_world_right_base = this->associated_robot_->Robot::getLinkTransformEigen("right_base_link");
	// std::cout << "Link world_right_base transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_world_right_base.matrix() << std::endl;

	// Eigen::AffineCompact3d t_world_left_base = this->associated_robot_->Robot::getLinkTransformEigen("left_base_link");
	// std::cout << "Link left_base_link transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_world_left_base.matrix() << std::endl;

	// Eigen::AffineCompact3d t_right_wrist_3 = this->associated_robot_->Robot::getLinkTransformEigen("right_wrist_3_link");
	// std::cout << "Link right_wrist_3_link transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_right_wrist_3.matrix() << std::endl;

	// Eigen::AffineCompact3d t_robot_base_2_arm_base = 
    //     Eigen::Translation3d(0.256, 0.000, 1.389) * 
    // //     Eigen::Quaterniond(0.3827733, 0.000, 0.9238423, 0.000);	// qw, qx, qy, qz	-> arm_right_base_link
    //     // Eigen::Quaterniond(0.000, 0.925, 0.000, 0.383);	// qw, qx, qy, qz	-> arm_right_base (no existeix en el urdf simplificat)
    //     Eigen::Quaterniond(-0.0845528, -0.4927906, -0.7814777, -0.3732304);	// qw, qx, qy, qz	-> right_base_link (URDF Kautham)
	// std::cout << "Hardcoded t_robot_base_2_arm_base transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_robot_base_2_arm_base.matrix() << std::endl;


	// Eigen::AffineCompact3d t_world_2_arm_base = t_base_link * t_robot_base_2_arm_base;
	// std::cout << "Hardcoded t_world_2_arm_base transformation (wrt Kautham world): " << std::endl;
	// std::cout << t_world_2_arm_base.matrix() << std::endl;


    // Get the transformation of the TCP w.r.t. the base of the UR5:
    Eigen::AffineCompact3d transformation_base_tool0;
    evaluateFKUR5(transformation_base_tool0, q[0], q[1], q[2], q[3], q[4], q[5]);
    // std::cout << "Input transformation:\n" << transformation_base_tool0.matrix() << std::endl;

    // Get the orientation from the transformation:
    Eigen::Quaterniond current_orientation(transformation_base_tool0.linear());
    // std::cout << "Current orientation (x, y, z, w): " << current_orientation.coeffs().transpose() << std::endl;

    // Calculate the orientation error:
    Eigen::Quaterniond error_orientation = current_orientation.inverse() * this->robot_prob_constraint_->getTargetOrientation();
    // std::cout << "Error orientation (x, y, z, w): " << error_orientation.coeffs().transpose() << std::endl;


	// Eigen::AffineCompact3d t_world_tcp = t_world_right_base * transformation_base_tool0;
    // std::cout << "Equivalent transformation:\n" << t_world_tcp.matrix() << std::endl;


    return error_orientation;
}

void OrientationConstraint::evaluateFKUR5(Eigen::AffineCompact3d& _transformation_base_tool0, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const {

    Eigen::Matrix3d rotation;

    rotation(0,0) = std::cos(q6) * (
                        std::sin(q1) * std::sin(q5) +
                        std::cos(q2 + q3 + q4) * std::cos(q1) * std::cos(q5)
                    ) -
                    std::sin(q2 + q3 + q4) * std::cos(q1) * std::sin(q6);

    rotation(0,1) = - std::sin(q6) * (
                        std::sin(q1) * std::sin(q5) +
                        std::cos(q2 + q3 + q4) * std::cos(q1) * std::cos(q5)
                    ) -
                    std::sin(q2 + q3 + q4) * std::cos(q1) * std::cos(q6);

    rotation(0,2) = std::cos(q5) * std::sin(q1) -
                    std::cos(q2 + q3 + q4) * std::cos(q1) * std::sin(q5);

    rotation(1,0) = - std::cos(q6) * (
                        std::cos(q1) * std::sin(q5) -
                        std::cos(q2 + q3 + q4) * std::cos(q5) * std::sin(q1)
                    ) -
                    std::sin(q2 + q3 + q4) * std::sin(q1) * std::sin(q6);

    rotation(1,1) = std::sin(q6) * (
                        std::cos(q1) * std::sin(q5) -
                        std::cos(q2 + q3 + q4) * std::cos(q5) * std::sin(q1)
                    ) -
                    std::sin(q2 + q3 + q4) * std::cos(q6) * std::sin(q1);

    rotation(1,2) = - std::cos(q1) * std::cos(q5) -
                    std::cos(q2 + q3 + q4) * std::sin(q1) * std::sin(q5);

    rotation(2,0) = std::cos(q2 + q3 + q4) * std::sin(q6) +
                    std::sin(q2 + q3 + q4) * std::cos(q5) * std::cos(q6);

    rotation(2,1) = std::cos(q2 + q3 + q4) * std::cos(q6) -
                    std::sin(q2 + q3 + q4) * std::cos(q5) * std::sin(q6);

    rotation(2,2) = - std::sin(q2 + q3 + q4) * std::sin(q5);

    _transformation_base_tool0.linear() = rotation;

    Eigen::Vector3d translation;

    translation(0) =    (2183.0/20000.0) * std::sin(q1)
                        - (17.0/40.0) * std::cos(q1) * std::cos(q2)
                        + (823.0/10000.0) * std::cos(q5) * std::sin(q1)
                        - (823.0/10000.0) * std::cos(q2 + q3 + q4) * std::cos(q1) * std::sin(q5)
                        + (1893.0/20000.0) * std::cos(q2 + q3) * std::cos(q1) * std::sin(q4)
                        + (1893.0/20000.0) * std::sin(q2 + q3) * std::cos(q1) * std::cos(q4)
                        - (1569.0/4000.0) * std::cos(q1) * std::cos(q2) * std::cos(q3)
                        + (1569.0/4000.0) * std::cos(q1) * std::sin(q2) * std::sin(q3);

    translation(1) =    (1569.0/4000.0) * std::sin(q1) * std::sin(q2) * std::sin(q3)
                        - (823.0/10000.0) * std::cos(q1) * std::cos(q5)
                        - (17.0/40.0) * std::cos(q2) * std::sin(q1)
                        - (2183.0/20000.0) * std::cos(q1)
                        - (823.0/10000.0) * std::cos(q2 + q3 + q4) * std::sin(q1) * std::sin(q5)
                        + (1893.0/20000.0) * std::cos(q2 + q3) * std::sin(q1) * std::sin(q4)
                        + (1893.0/20000.0) * std::sin(q2 + q3) * std::cos(q4) * std::sin(q1)
                        - (1569.0/4000.0) * std::cos(q2) * std::cos(q3) * std::sin(q1);

    translation(2) =    (1893.0/20000.0) * std::sin(q2 + q3) * std::sin(q4)
                        - (17.0/40.0) * std::sin(q2)
                        - (823.0/10000.0) * std::sin(q5) * (
                            std::cos(q2 + q3) * std::sin(q4)
                            + std::sin(q2 + q3) * std::cos(q4)
                        )
                        - (1893.0/20000.0) * std::cos(q2 + q3) * std::cos(q4)
                        - (1569.0/4000.0) * std::sin(q2 + q3)
                        + (6446200305038995.0/72057594037927936.0);

    _transformation_base_tool0.translation() = translation;

}
