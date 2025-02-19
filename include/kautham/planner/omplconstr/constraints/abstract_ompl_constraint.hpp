#ifndef ABSTRACT_OMPL_CONSTRAINT_HPP
#define ABSTRACT_OMPL_CONSTRAINT_HPP

#include <ompl/base/Constraint.h>

class AbstractOMPLConstraint : public ompl::base::Constraint {
    public:
        // Constructor: Set dimension of ambient space and the number of constraint equations
        AbstractOMPLConstraint(const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance);

        virtual ~AbstractOMPLConstraint() = default;

        // Virtual method to be overridden by derived classes
        virtual void useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) = 0;    // Polymorphic behavior
        virtual void printConstraintTarget() const = 0;    // Polymorphic behavior

    protected:
        const unsigned int num_dofs_;

        unsigned int num_constraints_;

        double tolerance_;

};

#endif // ABSTRACT_OMPL_CONSTRAINT_HPP
