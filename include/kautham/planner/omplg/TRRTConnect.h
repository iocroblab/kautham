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


#ifndef TRRTCONNECT_H
#define TRRTCONNECT_H


#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/OptimizationObjective.h>


namespace ompl {
    namespace geometric {
        class TRRTConnect : public base::Planner {
        public:
            TRRTConnect(const base::SpaceInformationPtr &si);

            virtual ~TRRTConnect();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            void setRange(double distance) {
                maxDistance_ = distance;
            }

            double getRange() const {
                return maxDistance_;
            }

            void setMaxStatesSucceed(double maxStatesSucceed) {
                maxStatesSucceed_ = maxStatesSucceed;
            }

            double getMaxStatesSucceed() const {
                return maxStatesSucceed_;
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
                if (initTemperature_ < minTemperature_) initTemperature_ = minTemperature_;
                if (tStart_.temp_ < minTemperature_) tStart_.temp_ = minTemperature_;
                if (tGoal_.temp_ < minTemperature_) tGoal_.temp_ = minTemperature_;
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

            void setDelayCC(bool delayCC) {
                delayCC_ = delayCC;
            }

            bool getDelayCC() const {
                return delayCC_;
            }

            template<template<typename T> class NN>
            void setNearestNeighbors() {
                tStart_.reset(new NN<Motion*>());
                tGoal_.reset(new NN<Motion*>());
            }

            virtual void setup();

        protected:
            class Motion {
            public:
                Motion() : root(NULL),state(NULL),parent(NULL) {
                }

                Motion(const base::SpaceInformationPtr &si) :
                    root(NULL),state(si->allocState()),parent(NULL) {
                }

                ~Motion() {
                }

                const base::State *root;

                base::State *state;

                Motion *parent;
            };

            //For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare {
                CostIndexCompare(const std::vector<base::Cost> &costs,
                                 const base::OptimizationObjective &opt) :
                    costs_(costs), opt_(opt) {

                }

                bool operator()(unsigned i, unsigned j) {
                    return opt_.isCostBetterThan(costs_[i],costs_[j]);
                }

                const std::vector<base::Cost> &costs_;

                const base::OptimizationObjective &opt_;
            };

            class TreeData : public std::shared_ptr<NearestNeighbors<Motion*> > {
            public:
                TreeData() : stateInBoxZos_(false),compareFn_(NULL) {
                }
                bool start_;

                double temp_;

                unsigned int numStatesSucceed_;

                unsigned int numStatesFailed_;

                unsigned int nonFrontierCount_;

                unsigned int frontierCount_;

                bool stateInBoxZos_;

                std::vector<base::Cost> costs_;

                std::vector<std::size_t> sortedCostIndices_;

                CostIndexCompare *compareFn_;

                base::State *root;
            };

            struct TreeGrowingInfo {
                base::State *xstate;

                Motion *xmotion;
            };

            enum ExtendResult {
                TRAPPED,
                ADVANCED,
                REACHED
            };

            void freeMemory();

            void freeTreeMemory(TreeData &tree);

            void clearTree(TreeData &tree);

            double distanceFunction(const Motion *a, const Motion *b) const {
                return si_->distance(a->state,b->state);
            }

            virtual ExtendResult extend(TreeData &tree, TreeGrowingInfo &tgi,
                                        Motion *rmotion);

            virtual bool connect(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

            bool transitionTest(base::Cost cost, double distance,
                                TreeData &tree, bool updateTemp);

            bool minExpansionControl(double distance, TreeData &tree);

            Motion *minCostNeighbor(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);


            base::StateSamplerPtr sampler_;

            bool delayCC_;

            double k_rrg_;

            double maxDistance_;

            double kConstant_;

            double frontierThreshold_;

            double frontierNodeRatio_;

            double tempChangeFactor_;

            double minTemperature_;

            double initTemperature_;

            unsigned int maxStatesSucceed_;

            unsigned int maxStatesFailed_;

            TreeData tStart_;

            TreeData tGoal_;

            RNG rng_;

            std::pair<base::State*, base::State*> connectionPoint_;

            ompl::base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif // TRRTCONNECT_H
