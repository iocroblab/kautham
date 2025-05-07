#ifndef SPHERE_CONSTR_HPP
#define SPHERE_CONSTR_HPP

#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

namespace Kautham {

class SphereConstraint : public AbstractOMPLConstraint {
    public:
        // Constructor: Set dimension of ambient space and the number of constraint equations
        SphereConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs);

        bool project(ompl::base::State *state) const override;

        // Function that computes the constraint value:
        void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

        // Function that computes the Jacobian of the constraint function:
        void jacobian(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::MatrixXd> out) const override;

        void useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) override;
        void printConstraintTarget() const override;

    private:

        Eigen::AffineCompact3d calculatePositionError(const Eigen::Ref<const Eigen::VectorXd>& q) const;

};

}

#endif // SPHERE_CONSTR_HPP
