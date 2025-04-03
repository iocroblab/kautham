#ifndef ABSTRACT_OMPL_CONSTRAINT_HPP
#define ABSTRACT_OMPL_CONSTRAINT_HPP

#include <ompl/base/Constraint.h>
#include <kautham/problem/prob_robot_constr.hpp>

// Forward declare the Robot class in the Kautham namespace
namespace Kautham {
    class Robot;
}

class AbstractOMPLConstraint : public ompl::base::Constraint {
    public:
        // Constructor: Set dimension of ambient space and the number of constraint equations
        AbstractOMPLConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints, const double tolerance);

        virtual ~AbstractOMPLConstraint() = default;

        // Virtual method to be overridden by derived classes
        virtual void useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) = 0;    // Polymorphic behavior
        virtual void printConstraintTarget() const = 0;    // Polymorphic behavior

        inline void associateRobot(Kautham::Robot* _associated_robot){associated_robot_ = _associated_robot;}

    protected:

        std::shared_ptr<Kautham::RobotProblemConstraint>  robot_prob_constraint_;    // Pointer to RobotProblemConstraint
        
        Kautham::Robot* associated_robot_;

};

#endif // ABSTRACT_OMPL_CONSTRAINT_HPP
