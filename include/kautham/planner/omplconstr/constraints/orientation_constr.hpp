#ifndef ORIENTATION_CONSTR_HPP
#define ORIENTATION_CONSTR_HPP

#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>
#include <Eigen/Core>

class OrientationConstraint : public AbstractOMPLConstraint {
    public:
        // Constructor: Set dimension of ambient space and the number of constraint equations
        OrientationConstraint(const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance);

        bool project(ompl::base::State *state) const override;
        // bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

        // Function that computes the constraint value:
        void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

        // Function that computes the Jacobian of the constraint function:
        void jacobian(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::MatrixXd> out) const override;

        bool setTargetOrientation(const Eigen::Quaterniond& ori);

        void useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) override;
        void printConstraintTarget() const override;

    private:

        Eigen::Quaterniond target_orientation_;

        Eigen::Quaterniond calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const;

        void evaluateFKUR5(Eigen::AffineCompact3d& _transformation_base_tool0, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const;

        void evaluateJacobianUR5(Eigen::MatrixXd& _jacobian, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const;

};

#endif // ORIENTATION_CONSTR_HPP
