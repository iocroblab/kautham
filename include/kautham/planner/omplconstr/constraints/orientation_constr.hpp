#ifndef ORIENTATION_CONSTR_HPP
#define ORIENTATION_CONSTR_HPP

#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

namespace Kautham {

    class OrientationConstraint : public AbstractOMPLConstraint {
        public:
            // Constructor: Set dimension of ambient space and the number of constraint equations
            OrientationConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs);

            // Function that computes the constraint value:
            void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

        private:

            Eigen::Quaterniond calculateOrientationError(const Eigen::Ref<const Eigen::VectorXd>& q) const;

            void updateTolerance(const Eigen::Ref<const Eigen::VectorXd> &q);

    };

}

#endif // ORIENTATION_CONSTR_HPP
