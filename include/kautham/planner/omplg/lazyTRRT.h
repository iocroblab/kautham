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


#ifndef LAZYTRRT_H
#define LAZYTRRT_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/OptimizationObjective.h>

namespace ompl {
    namespace geometric {
        class LazyTRRT : public base::Planner {
        public:
            LazyTRRT(const base::SpaceInformationPtr &si);

            ~LazyTRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

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

            void setMinCollisionThreshold(double minCollThr) {
                minCollThr_ = minCollThr;
            }

            double getMinCollisionThreshold() const {
                return minCollThr_;
            }

            void setMaxCollisionThreshold(double maxCollThr) {
                maxCollThr_ = maxCollThr;
            }

            double getMaxCollisionThreshold() const {
                return maxCollThr_;
            }

            virtual void setup();

        protected:
            class Motion {
            public:

                Motion(): state(NULL),parent(NULL),valid(false) {
                }

                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()),parent(NULL),valid(false) {
                }

                ~Motion() {
                }

                base::State *state;

                Motion *parent;

                std::vector<Motion*> children;

                bool valid;

                base::Cost cost;
            };

            void removeMotion(Motion *motion);

            void freeMemory();

            double distanceFunction(const Motion *a, const Motion *b) const {
                return si_->distance(a->state,b->state);
            }

            bool transitionTest(double childCost, double parentCost, double distance);

            bool minExpansionControl(double randMotionDistance);


            base::StateSamplerPtr sampler_;

            std::shared_ptr< NearestNeighbors<Motion*> > nearestNeighbors_;

            double goalBias_;

            double maxDistance_;

            RNG rng_;

            Motion *lastGoalMotion_;

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

            unsigned int nonfrontierCount_;

            unsigned int frontierCount_;

            double frontierThreshold_;

            double frontierNodeRatio_;

            ompl::base::OptimizationObjectivePtr opt_;

            double minCollThr_;

            double maxCollThr_;
        };
    }
}

#endif // LAZYTRRT_H
