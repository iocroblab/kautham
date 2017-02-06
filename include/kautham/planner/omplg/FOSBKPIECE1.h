/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#ifndef FOSBKPIECE1_H
#define FOSBKPIECE1_H


#include <ompl/geometric/planners/PlannerIncludes.h>
#include "FOSDiscretization.h"
#include <armadillo>
#include "omplplanner.h"
#include <kautham/planner/omplg/synergy_tree.h>


namespace ompl {
    namespace geometric {
        /** \brief Bi-directional KPIECE with one level of discretization */
        class FOSBKPIECE1 : public base::Planner {
        public:
            /** \brief Constructor. */
            FOSBKPIECE1(const base::SpaceInformationPtr &si,  Kautham::omplplanner::omplPlanner *pl,
                        SynergyTree *st, double exploration = 0.25,
                        double initial_lambda = 1., unsigned int update_freq = 20);

            /** \brief Destructor. */
            virtual ~FOSBKPIECE1();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** Reset internal data. */
            virtual void clear();

            void setTreeBias(double treeBias) {
                treeBias_ = treeBias;
            }

            double getTreeBias() const {
                return treeBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance) {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const {
                return maxDistance_;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp) {
                dStart_.setBorderFraction(bp);
                dGoal_.setBorderFraction(bp);
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const {
                return dStart_.getBorderFraction();
            }

            /** \brief When extending a motion from a cell, the
                extension can be successful or it can fail. If the
                extension fails, the score of the cell is multiplied
                by \e factor. These number should be in the range (0, 1]. */
            void setFailedExpansionCellScoreFactor(double factor) {
                failedExpansionScoreFactor_ = factor;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getFailedExpansionCellScoreFactor() const {
                return failedExpansionScoreFactor_;
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction. */
            void setMinValidPathFraction(double fraction) {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const {
                return minValidPathFraction_;
            }

            void setExploration(double exploration)
            {
                explorationSetting_ = exploration;
            }

            double getExploration() const
            {
                return explorationSetting_;
            }

            void setInitialLambda(double initial_lambda) {
                initialLambda_ = initial_lambda;
            }

            double getInitialLambda() const {
                return initialLambda_;
            }

            void setEfficiencyThreshold(double efficiency_threshold) {
                efficiencyThreshold_ = efficiency_threshold;
            }

            double getEfficiencyThreshold() {
                return efficiencyThreshold_;
            }

            void setUpdateFrequency(unsigned int update_freq)
            {
                nth_step_ = update_freq;
            }

            unsigned int getUpdateFrequency() const
            {
                return nth_step_;
            }

            void setInteriorCellNeighborLimit(unsigned int count) {
                dStart_.setInteriorCellNeighborLimit(count);
                dGoal_.setInteriorCellNeighborLimit(count);
            }

            unsigned int getInteriorCellNeighborLimit() const {
                return dStart_.getInteriorCellNeighborLimit();
            }

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator) {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name) {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const {
                return projectionEvaluator_;
            }

            virtual void setup();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:
            void omplState2armaVec(const ompl::base::State *s, arma::vec &q) const;

            void armaVec2omplState(const arma::vec &q, ompl::base::State *s) const;

            /** \brief Representation of a motion for this algorithm */
            class Motion {
            public:
                Motion() : root(nullptr), state(nullptr), parent(nullptr) {}

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) :
                    root(nullptr), state(si->allocState()), parent(nullptr) {}

                ~Motion() {}

                /** \brief The root state (start state) that leads to this motion */
                const base::State *root;

                /** \brief The state contained by this motion */
                base::State *state;

                /** \brief The parent motion in the exploration tree */
                Motion *parent;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const {
                return si_->distance(a->state, b->state);
            }

            NearestNeighbors<Motion*> *getDefaultNearestNeighbors() const {
                return tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this);
            }

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** Use the vector field to alter the direction of a sample. */
            arma::vec getNewDirection(const arma::vec &vrand, const arma::vec &vfield);

            /**
             * Every nth time this function is called (where the nth step is the update
             * frequency given in the constructor) the value of lambda is updated and
             * the counts of efficient and inefficient samples added to the tree are
             * reset to 0. The measure for exploration inefficiency is also reset to 0.
             */
            void updateGain();

            /**
             * This attempts to extend the tree from the motion m to a new motion
             * in the direction specified by the vector v.
             */
            Motion *extendTree(Motion *m, Discretization<Motion>::Cell *cell,
                               const base::State *biasState, bool reverseVF, base::State *tmp);

            /**
             * Updates measures for exploration efficiency if a given motion m is added to the
             * nearest NearestNeighbors structure.
             */
            void updateExplorationEfficiency(Motion *m, Discretization<Motion>::Cell *cell);

            /** \brief The employed state sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The start tree */
            Discretization<Motion> dStart_;

            /** \brief The goal tree */
            Discretization<Motion> dGoal_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** Synergy tree that creates the vector field. */
            const SynergyTree *st_;

            std::map<unsigned int,std::pair<std::set<unsigned int>,std::set<unsigned int> > > robotJoint;

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double failedExpansionScoreFactor_;

            double treeBias_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double minValidPathFraction_;

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The pair of states in each tree connected during planning. Used for PlannerData computation */
            std::pair<base::State*, base::State*> connectionPoint_;

            /** Number of efficient nodes. */
            unsigned int efficientCount_;

            /** Number of inefficient nodes. */
            unsigned int inefficientCount_;

            /** Current inefficiency. */
            double explorationInefficiency_;

            /** User-specified exploration parameter. */
            double explorationSetting_;

            /** Current lambda value. */
            double lambda_;

            double initialLambda_;

            double efficiencyThreshold_;

            /** The number of steps until lambda is updated and efficiency metrics are reset. */
            unsigned int nth_step_;

            /** Current number of steps since lambda was updated/initialized. */
            unsigned int step_;

            /** Dimensionality of vector field */
            unsigned int vfdim_;
        };
    }
}


#endif // FOSBKPIECE1_H
