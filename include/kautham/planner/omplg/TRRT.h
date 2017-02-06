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


#ifndef TRRT_H
#define TRRT_H


#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/OptimizationObjective.h>


/*
 NOTES:
 **Variable Names that have been converted to longer versions from standards:
 nearest_neighbors_ -> nn_
 planner_termination_condition -> ptc

 **Inherited Member Variables Key:
 si_ -> SpaceInformation
 pdef_ -> ProblemDefinition
 pis_ -> PlannerInputStates - Utility class to extract valid input states
*/


namespace ompl {
    namespace geometric {
        class TRRT : public base::Planner {
        public:
            TRRT(const base::SpaceInformationPtr &si);

            virtual ~TRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition
                                              &plannerTerminationCondition);

            virtual void clear();

            void setGoalBias(double goalBias) {
                goalBias_ = goalBias;
            }

            double getGoalBias() const {
                return goalBias_;
            }

            void setRange(double distance) {
                maxDistance_ = distance;
            }

            double getRange() const {
                return maxDistance_;
            }

            void setMaxStatesFailed(double maxStatesFailed) {
                maxStatesFailed_ = maxStatesFailed;
            }

            double getMaxStatesFailed() const {
                return maxStatesFailed_;
            }

            void setTempChangeFactor(double tempChangeFactor) {
                tempChangeFactor_ = tempChangeFactor;
            }

            double getTempChangeFactor() const {
                return tempChangeFactor_;
            }

            void setMinTemperature(double minTemperature) {
                minTemperature_ = minTemperature;
            }

            double getMinTemperature() const {
                return minTemperature_;
            }

            void setInitTemperature(double initTemperature) {
                initTemperature_ = initTemperature;
            }

            double getInitTemperature() const {
                return initTemperature_;
            }

            void setFrontierThreshold(double frontier_threshold) {
                frontierThreshold_ = frontier_threshold;
            }

            double getFrontierThreshold() const {
                return frontierThreshold_;
            }

            void setFrontierNodeRatio(double frontierNodeRatio) {
                frontierNodeRatio_ = frontierNodeRatio;
            }

            double getFrontierNodeRatio() const {
                return frontierNodeRatio_;
            }

            void setKConstant(double kConstant) {
                kConstant_ = kConstant;
            }

            double getKConstant() const {
                return kConstant_;
            }

            template<template<typename T> class NN>
            void setNearestNeighbors() {
                nearestNeighbors_.reset(new NN<Motion*>());
            }

            virtual void setup();


        protected:
            class Motion {
            public:
                Motion() : state(NULL), parent(NULL) {
                }

                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()), parent(NULL) {
                }

                ~Motion() {
                }

                base::State *state;

                Motion *parent;
            };

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const {
                return si_->distance(a->state, b->state);
            }

            bool transitionTest(base::Cost cost, double distance);

            bool minExpansionControl(double randMotionDistance);


            base::StateSamplerPtr sampler_;

            std::shared_ptr<NearestNeighbors<Motion*> > nearestNeighbors_;

            double goalBias_;

            double maxDistance_;

            RNG rng_;

            Motion *lastGoalMotion_;

            bool verbose_;

            // *********************************************************************************************************
            // TRRT-Specific Variables
            // *********************************************************************************************************

            // Transtion Test -----------------------------------------------------------------------

            double temp_;

            double kConstant_;

            unsigned int maxStatesFailed_;

            double tempChangeFactor_;

            double minTemperature_;

            double initTemperature_;

            unsigned int numStatesFailed_;

            // Minimum Expansion Control --------------------------------------------------------------

            double nonfrontierCount_;

            double frontierCount_;

            double frontierThreshold_;

            double frontierNodeRatio_;

            bool stateInBoxZos_;

            ompl::base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif // TRRT_H
