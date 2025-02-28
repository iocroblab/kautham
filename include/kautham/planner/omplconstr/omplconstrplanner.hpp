/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Pol Ramon Canyameres, Jan Rosell */

#ifndef OMPLCONSTRAINTPLANNER_HPP
#define OMPLCONSTRAINTPLANNER_HPP

#if defined(KAUTHAM_USE_OMPL)

#include <kautham/planner/planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
// #include <kautham/sampling/sampling.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class AbstractOMPLConstraint;

namespace Kautham {
// /** \addtogroup ControlPlanners
//  *  @{
//  */

    namespace omplconstrplanner{

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Class KauthamConstrStateSampler
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This class represents a  state sampler based on the sampling of the Kautham control space and
        //! its conversion to samples.
        class KauthamConstrStateSampler : public ob::CompoundStateSampler {
            public:
                KauthamConstrStateSampler(const ob::StateSpace *sspace, Planner *p);
                ~KauthamConstrStateSampler();
                
                virtual void sampleUniform(ob::State *state);

                RandomSampler* _samplerRandom;

            protected:
                ompl::RNG rng_; //!< random generator
                Planner *kauthamPlanner_; //!< pointer to planner in order to have access to the workspace
        };


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // AUXILIAR Functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        ob::StateSamplerPtr allocStateSampler(const ob::StateSpace *mysspace, Planner *p);


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Class omplconstrPlanner
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This class is the base class for all the kautham planners that use the ompl::base::Constraint planners.

        class omplConstraintPlanner : public Planner {
            public:

                omplConstraintPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
                
                ~omplConstraintPlanner();

                std::shared_ptr<ompl::base::Planner> ompl_planner_;

                std::shared_ptr<ompl::base::ProblemDefinition> pdef_;

                std::map<std::string, std::shared_ptr<AbstractOMPLConstraint>> constraint_map_;

                // Override the pure virtual functions from Planner
                virtual bool setParameters() override;  // Implemented in cpp
                virtual bool trySolve() override;       // Implemented in cpp

                void smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate);

                void omplState2smp(const ob::State *state, Sample* smp);
                void omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp);

                inline std::vector<size_t> getSpaceIndexMapping() {return index_mapping_;}

                //! Returns a pointer to the problem definition
                inline ompl::base::StateSpace *StateSpace() {return space_.get();}

                //! Returns a pointer to the problem definition
                inline ob::ProblemDefinition *ProblemDefinition() {return pdef_.get();}
                bool have_constr_space;
                bool have_unconstr_space;

                inline vector<int> *getDisabledControls(){return &_disabledcontrols;}
                inline ob::StateSpacePtr getSpace(){return space_;}

            private:
                double _planningTime;
                double _range;

                std::shared_ptr<ompl::base::StateSpace> space_;

                std::shared_ptr<ompl::base::SpaceInformation> si_;

                // Needed because the space order of getSolutionPath() is not equal to the robot if constraints are used.
                // The order of the vector follows the urdf_joint_index, and it`s values follows the ompl_solution_index.
                std::vector<size_t> index_mapping_;

                vector<int> _disabledcontrols;//!< those disabled controls will not be sampled, they are fixed at 0.5

                void assignConstrTargetFromState(const ob::ScopedState<ob::CompoundStateSpace> _ompl_state);
                
                //! This function is used to verify that the low bound is below the high bound
                inline void filterBounds(double &l, double &h, double epsilon)
                {
                    if((h - l) < epsilon) h = l + epsilon;
                }
            };

        }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif // OMPLCONSTRAINTPLANNER_HPP