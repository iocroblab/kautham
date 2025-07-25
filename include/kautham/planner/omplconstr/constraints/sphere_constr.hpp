#ifndef SPHERE_CONSTR_HPP
#define SPHERE_CONSTR_HPP

#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

namespace Kautham {

    class SphereConstraint : public AbstractOMPLConstraint {
        public:
            // Constructor: Set dimension of ambient space and the number of constraint equations
            SphereConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs);

            // Function that computes the constraint value:
            void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

    };

}

#endif // SPHERE_CONSTR_HPP
