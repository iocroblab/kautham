#ifndef ORIENTATION_CONSTR_HPP
#define ORIENTATION_CONSTR_HPP

#include <ompl/base/Constraint.h>
#include <Eigen/Core>

class OrientationConstraint : public ompl::base::Constraint {
    public:
        // Constructor: Set dimension of ambient space and the number of constraint equations
        OrientationConstraint(const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance);

        bool project(ompl::base::State *state) const override;

        // Function that computes the constraint value:
        void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

        // Function that computes the Jacobian of the constraint function:
        void jacobian(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::MatrixXd> out) const override;

        bool setTargetOrientation(const Eigen::Quaterniond& ori);

        void setJointConfigAsTargetOrientation(const std::vector<double>& joint_config);

        inline void printTargetOrientation() const {
            std::cout << "Target Orientation (Quaternion: qx qy qz qw): "
                << target_orientation_.coeffs().transpose() << std::endl;
        }

    private:

        const unsigned int num_dofs_;

        unsigned int num_constraints_;

        Eigen::Quaterniond target_orientation_;

        Eigen::Quaterniond calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const;

        void evaluateFKUR5(Eigen::AffineCompact3d& _transformation_base_tool0, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const;

        void evaluateJacobianUR5(Eigen::MatrixXd& _jacobian, const double q1, const double q2, const double q3, const double q4, const double q5, const double q6) const;

};

#endif // ORIENTATION_CONSTR_HPP
