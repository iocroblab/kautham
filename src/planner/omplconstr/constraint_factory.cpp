#include <kautham/planner/omplconstr/constraint_factory.hpp>

#if defined(KAUTHAM_USE_OMPL)

#include <iostream> // For std::cout
#include <stdexcept>

// Include all the constraints:
#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>



namespace Kautham {
    //! Namespace omplconstrplanner contains the planners based on the OMPL::constraint library
    namespace omplconstrplanner{

        // Constructor to register constraints automatically
        // ConstraintFactory::ConstraintFactory() {
        //     registerConstraints();
        // }

        void ConstraintFactory::registerConstraint(const std::string& name, ConstraintCreator creator) {
            creators_[name] = creator;
        }

        std::shared_ptr<ConstraintFactory::ConstraintBase> ConstraintFactory::createConstraint(const std::string& name, std::shared_ptr<Kautham::RobotProblemConstraint> _robot_prob_constraint, unsigned int ambientDim, unsigned int coDim, double tolerance) {
            auto it = creators_.find(name);
            if (it == creators_.end()) {
                throw std::runtime_error("Constraint type not found: " + name);
            }
            return it->second(_robot_prob_constraint, ambientDim, coDim, tolerance);
        }

        void ConstraintFactory::registerConstraints() {
            registerConstraint("arm_orientation",
                [](std::shared_ptr<Kautham::RobotProblemConstraint> _robot_prob_constraint, unsigned int ambientDim, unsigned int coDim, double tolerance) 
                    -> std::shared_ptr<ConstraintFactory::ConstraintBase> {
                        return std::make_shared<OrientationConstraint>(_robot_prob_constraint, ambientDim, coDim, tolerance);
                }
            );
        }

        void ConstraintFactory::printRegisteredConstraintNames() const {
            std::cout << "Registered Constraints: " << std::endl;
            for (const auto& pair : creators_) {
                std::cout << " - " << pair.first << std::endl;
            }
            std::cout << std::endl;
        }


    }
}

#endif // KAUTHAM_USE_OMPL