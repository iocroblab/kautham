#ifndef BOX_CONSTR_HPP
#define BOX_CONSTR_HPP

#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

namespace Kautham {

    class BoxConstraint : public AbstractOMPLConstraint {
        public:
            // Constructor: Set dimension of ambient space and the number of constraint equations
            BoxConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs);

            // Function that computes the constraint value:
            void function(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::Ref<Eigen::VectorXd> out) const override;

    };

}

#endif // BOX_CONSTR_HPP
