#ifndef ABSTRACT_OMPL_CONSTRAINT_HPP
#define ABSTRACT_OMPL_CONSTRAINT_HPP

#include <ompl/base/Constraint.h>
#include <kautham/problem/prob_robot_constr.hpp>
#include <Eigen/Core>

namespace Kautham {
    
    // Forward declare the Robot class in the Kautham namespace
    class Robot;

    class AbstractOMPLConstraint : public ompl::base::Constraint {
        public:
            // Constructor: Set dimension of ambient space and the number of constraint equations
            AbstractOMPLConstraint(std::shared_ptr<Kautham::RobotProblemConstraint>  _robot_prob_constraint, const unsigned int num_dofs, const unsigned int num_constraints);

            virtual ~AbstractOMPLConstraint() = default;

            // Virtual method to be overridden by derived classes
            virtual void useJointConfig2SetConstraintTarget(const std::vector<double>& joint_config) = 0;    // Polymorphic behavior
            virtual void printConstraintTarget() const = 0;    // Polymorphic behavior

            inline void associateRobot(Kautham::Robot* _associated_robot){associated_robot_ = _associated_robot;}
            inline void assignReferencedEntity(Kautham::Robot* _referenced_entity){referenced_entity_ = _referenced_entity;}
            void setGoalSampleConfiguration(const std::vector<double> _goal_config);

            void initConstraintStuff();

        protected:

            std::shared_ptr<Kautham::RobotProblemConstraint>  robot_prob_constraint_;    // Pointer to RobotProblemConstraint
            
            Kautham::Robot* associated_robot_;
            Kautham::Robot* referenced_entity_;

            Eigen::AffineCompact3d goal_sample_t_robot_base_link_2_end_effector_link;   //!< Used to compute the distance between the goal sample and the current sample.
            
            Eigen::AffineCompact3d ComputeFKFromRobotBaseLinkToEnfEffectorLink(const Eigen::Ref<const Eigen::VectorXd>& q) const;
            
            Eigen::AffineCompact3d getGeometricTransformationWRTRobotBaseLink() const;
            
            
        private:

            Eigen::AffineCompact3d t_constr_end_2_target_link;   //!< Used to add an offset after the constraint.
        
            Eigen::AffineCompact3d getTransfromationFromConstrEndLink2TargetLink();

    };
}

#endif // ABSTRACT_OMPL_CONSTRAINT_HPP
