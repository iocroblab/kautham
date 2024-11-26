#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>

// Constructor implementation
OrientationConstraint::OrientationConstraint(const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance)
    : ompl::base::Constraint(num_dofs, num_constraints, tolerance)
    , num_dofs_(num_dofs), num_constraints_(num_constraints)
{

}

bool OrientationConstraint::setTargetOrientation(const Eigen::Quaterniond& ori) {
    this->target_orientation_ = ori;
    return true;
}

void OrientationConstraint::setJointConfigAsTargetOrientation(const std::vector<double>& joint_config) {
    // Get the transformation of the TCP w.r.t. the base of the UR5:
    Eigen::AffineCompact3d transformation_base_tool0;
    evaluateFKUR5(transformation_base_tool0, 
                    joint_config[0], 
                    joint_config[1], 
                    joint_config[2], 
                    joint_config[3], 
                    joint_config[4], 
                    joint_config[5]);

    this->target_orientation_ = Eigen::Quaterniond(transformation_base_tool0.rotation());
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
// void OrientationConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::MatrixXd> out) const {

    // Eigen::MatrixXd jacobian_geometric(6,6);
    // evaluateJacobianUR5(jacobian_geometric, q[0], q[1], q[2], q[3], q[4], q[5]);

    // for (std::size_t i = 0; i < num_constraints_; ++i) {
    //     // out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);  // 3x3 -> 1x3
    //     out.row(i) = jacobian_geometric.bottomRows(i);  // 3x3 -> 1xnum_constraints_
    // }
// }

Eigen::Quaterniond OrientationConstraint::calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const {

    // Get the transformation of the TCP w.r.t. the base of the UR5:
    Eigen::AffineCompact3d transformation_base_tool0;
    evaluateFKUR5(transformation_base_tool0, q[0], q[1], q[2], q[3], q[4], q[5]);
    // std::cout << "Input transformation:\n" << transformation_base_tool0.matrix() << std::endl;

    // Get the orientation from the transformation:
    Eigen::Quaterniond current_orientation(transformation_base_tool0.linear());
    // std::cout << "Current orientation (x, y, z, w): " << current_orientation.coeffs().transpose() << std::endl;

    // Calculate the orientation error:
    Eigen::Quaterniond error_orientation = current_orientation.inverse() * target_orientation_;
    // std::cout << "Error orientation (x, y, z, w): " << error_orientation.coeffs().transpose() << std::endl;

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

    // this->RobotArm::normalizeTransformation(_transformation_base_tool0);

}

void OrientationConstraint::evaluateJacobianUR5(Eigen::MatrixXd& _jacobian, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const {

    (void)q6;
    
	_jacobian(0,0) = (2183*std::cos(q1))/20000
						+ (823*std::cos(q1)*std::cos(q5))/10000
						+ (17*std::cos(q2)*std::sin(q1))/40
						- (1569*std::sin(q1)*std::sin(q2)*std::sin(q3))/4000
						+ (823*std::cos(q2 + q3 + q4)*std::sin(q1)*std::sin(q5))/10000
						- (1893*std::cos(q2 + q3)*std::sin(q1)*std::sin(q4))/20000
						- (1893*std::sin(q2 + q3)*std::cos(q4)*std::sin(q1))/20000
						+ (1569*std::cos(q2)*std::cos(q3)*std::sin(q1))/4000;

	_jacobian(0,1) = std::cos(q1)*(
							(1569*std::sin(q2 + q3))/4000
							+ (17*std::sin(q2))/40
							- (1893*std::sin(q2 + q3)*std::sin(q4))/20000
							+ (1893*std::cos(q2 + q3)*std::cos(q4))/20000
							+ std::sin(q5)*(
								(823*std::cos(q2 + q3)*std::sin(q4))/10000
								+ (823*std::sin(q2 + q3)*std::cos(q4))/10000
							)
						);

	_jacobian(0,2) = std::cos(q1)*(
							(1893*std::cos(q2 + q3 +q4))/20000
							+ (1569*std::sin(q2 + q3))/4000
							+ (823*std::sin(q2 + q3 + q4)*std::sin(q5))/10000
						);

	_jacobian(0,3) = std::cos(q1)*(
							(1893*std::cos(q2 + q3 +q4))/20000
							+ (823*std::sin(q2 + q3 + q4)*std::sin(q5))/10000
						);

	_jacobian(0,4) = (823.0/10000.0)*(
							std::cos(q1)*std::cos(q2)*std::cos(q5)*std::sin(q3)*std::sin(q4)
							- std::cos(q1)*std::cos(q2)*std::cos(q3)*std::cos(q4)*std::cos(q5)
							- std::sin(q1)*std::sin(q5)
							+ std::cos(q1)*std::cos(q3)*std::cos(q5)*std::sin(q2)*std::sin(q4)
							+ std::cos(q1)*std::cos(q4)*std::cos(q5)*std::sin(q2)*std::sin(q3)
						);

	_jacobian(0,5) = 0;

	_jacobian(1,0) = (2183*std::sin(q1))/20000
                        - (17*std::cos(q1)*std::cos(q2))/40
						+ (823*std::cos(q5)*std::sin(q1))/10000
						- (823*std::cos(q2 + q3 + q4)*std::cos(q1)*std::sin(q5))/10000
						+ (1893*std::cos(q2 + q3)*std::cos(q1)*std::sin(q4))/20000
						+ (1893*std::sin(q2 + q3)*std::cos(q1)*std::cos(q4))/20000
						- (1569*std::cos(q1)*std::cos(q2)*std::cos(q3))/4000
						+ (1569*std::cos(q1)*std::sin(q2)*std::sin(q3))/4000;

	_jacobian(1,1) = std::sin(q1)*(
							(1569*std::sin(q2 + q3))/4000
							+ (17*std::sin(q2))/40
							- (1893*std::sin(q2 + q3)*std::sin(q4))/20000
							+ (1893*std::cos(q2 + q3)*std::cos(q4))/20000
							+ std::sin(q5)*(
								(823*std::cos(q2 + q3)*std::sin(q4))/10000
								+ (823*std::sin(q2 + q3)*std::cos(q4))/10000
							)
						);

	_jacobian(1,2) = std::sin(q1)*(
							(1893*std::cos(q2 + q3 +q4))/20000
							+ (1569*std::sin(q2 + q3))/4000
							+ (823*std::sin(q2 + q3 + q4)*std::sin(q5))/10000
						);

	_jacobian(1,3) = std::sin(q1)*(
							(1893*std::cos(q2 + q3 +q4))/20000
							+ (823*std::sin(q2 + q3 + q4)*std::sin(q5))/10000
						);

	_jacobian(1,4) = (823.0/10000.0)*(
							std::cos(q1)*std::sin(q5)
							- std::cos(q2)*std::cos(q3)*std::cos(q4)*std::cos(q5)*std::sin(q1)
							+ std::cos(q2)*std::cos(q5)*std::sin(q1)*std::sin(q3)*std::sin(q4)
							+ std::cos(q3)*std::cos(q5)*std::sin(q1)*std::sin(q2)*std::sin(q4)
							+ std::cos(q4)*std::cos(q5)*std::sin(q1)*std::sin(q2)*std::sin(q3)
						);

	_jacobian(1,5) = 0;

	_jacobian(2,0) = 0;

	_jacobian(2,1) = (1893*std::sin(q2 + q3 + q4))/20000
						- (823*std::sin(q2 + q3 + q4 + q5))/20000
						- (1569*std::cos(q2 + q3))/4000
						- (17*std::cos(q2))/40
						+ (823*std::sin(q2 + q3 + q4 - q5))/20000;

	_jacobian(2,2) = (1893*std::sin(q2 + q3 + q4))/20000
						- (823*std::sin(q2 + q3 + q4 + q5))/20000
						- (1569*std::cos(q2 + q3))/4000
						+ (823*std::sin(q2 + q3 + q4 - q5))/20000;

	_jacobian(2,3) = (1893*std::sin(q2 + q3 + q4))/20000
						- (823*std::sin(q2 + q3 + q4 + q5))/20000
						+ (823*std::sin(q2 + q3 + q4 - q5))/20000;

	_jacobian(2,4) = - (823*std::sin(q2 + q3 + q4 + q5))/20000
						- (823*std::sin(q2 + q3 + q4 - q5))/20000;

	_jacobian(2,5) = 0;

	_jacobian(3,0) = 0;

	_jacobian(3,1) = std::sin(q1);

	_jacobian(3,2) = std::sin(q1);

	_jacobian(3,3) = std::sin(q1);

	_jacobian(3,4) = std::sin(q2 + q3 + q4)*std::cos(q1);

	_jacobian(3,5) = std::cos(q5)*std::sin(q1)
						- std::cos(q2 + q3 + q4)*std::cos(q1)*std::sin(q5);

	_jacobian(4,0) = 0;

	_jacobian(4,1) = - std::cos(q1);

	_jacobian(4,2) = - std::cos(q1);

	_jacobian(4,3) = - std::cos(q1);

	_jacobian(4,4) = std::sin(q2 + q3 + q4)*std::sin(q1);

	_jacobian(4,5) = - std::cos(q1)*std::cos(q5)
						- std::cos(q2 + q3 + q4)*std::sin(q1)*std::sin(q5);

	_jacobian(5,0) = 1;

	_jacobian(5,1) = 0;

	_jacobian(5,2) = 0;

	_jacobian(5,3) = 0;

	_jacobian(5,4) = - std::cos(q2 + q3 + q4);

	_jacobian(5,5) = - std::sin(q2 + q3 + q4)*std::sin(q5);

}
