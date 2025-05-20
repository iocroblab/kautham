#pragma once

#include <ompl/base/Constraint.h>

namespace Kautham {

    class KauthamOMPLConstraintIntersection : public ompl::base::ConstraintIntersection {
        public:
            // Inherit constructors
            using ompl::base::ConstraintIntersection::ConstraintIntersection;

            // Override project
            bool project(ompl::base::State *state) const override
            {
                // Sequentially project onto each constraint in the intersection
                for (const auto &c : constraints_)
                {
                    if (!c->project(state))
                        return false;
                }
                return true;
            }
    };

}