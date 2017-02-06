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

/* Authors: Nestor Garcia Hidalgo */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_FOSVFRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_FOSVFRRT_

#include <limits>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <armadillo>
#include <kautham/planner/omplg/synergy_tree.h>
#include "omplplanner.h"

namespace ompl
{
    namespace geometric
    {
        /**
            \anchor gFOSVFRRT
            \par Short description
            Vector Field Rapidly-exploring Random Tree (FOSVFRRT) is a tree-based
            motion planner that tries to minimize the so-called upstream cost
            of a path. The upstream cost is defined by an integral over a
            user-defined vector field.

            \par External documentation
            I. Ko, B. Kim, and F. C. Park, Randomized path planning on vector fields, <em>Intl. J. of Robotics Research,</em> 33(13), pp. 1664–1682, 2014. DOI: [10.1177/0278364914545812](http://dx.doi.org/10.1177/0278364914545812)<br>

            [[PDF]](http://robotics.snu.ac.kr/fcp/files/_pdf_files_publications/201411_Randomized%20path%20planning.pdf)

        */
        class FOSVFRRT : public RRT
        {
        public:
            /** Constructor. */
            FOSVFRRT(const base::SpaceInformationPtr &si, Kautham::omplplanner::omplPlanner *pl,
                     SynergyTree *st, double exploration = 0.25,
                     double initial_lambda = 1., unsigned int update_freq = 20);

            /** Destructor. */
            virtual ~FOSVFRRT();

            /** Reset internal data. */
            virtual void clear();

            /** Make a Monte Carlo estimate for the mean vector norm in the field. */
            double determineMeanNorm();

            /** Use the vector field to alter the direction of a sample. */
            arma::vec getNewDirection(const base::State *snear, const base::State *srand);

            /** Calculates the weight omega which can be used to compute alpha and beta. */
            double biasedSampling (const arma::vec &vrand, const arma::vec &vfield,
                                   double lambdaScale);

            /**
             * Every nth time this function is called (where the nth step is the update
             * frequency given in the constructor) the value of lambda is updated and
             * the counts of efficient and inefficient samples added to the tree are
             * reset to 0. The measure for exploration inefficiency is also reset to 0.
             */
            void updateGain ();

            /**
             * Computes alpha and beta, using these values to produce the vector returned by
             * getNewDirection. This produced vector can be used to determine the direction an
             * added state should be to maximize the upstream criterion of the produced path.
             */
            arma::vec computeAlphaBeta (double omega, const arma::vec &vrand,
                                        const arma::vec &vfield);

            /**
             * This attempts to extend the tree from the motion m to a new motion
             * in the direction specified by the vector v.
             */
            Motion *extendTree (Motion *m, base::State* rstate, bool biased, const arma::vec &v);
            /**
             * Updates measures for exploration efficiency if a given motion m is added to the
             * nearest NearestNeighbors structure.
             */
            void updateExplorationEfficiency(Motion *m);

            /** Solve the planning problem. */
            virtual base::PlannerStatus solve (const base::PlannerTerminationCondition &ptc);

            virtual void setup();

            void setExploration(double exploration)
            {
                explorationSetting_ = exploration;
            }

            double getExploration() const
            {
                return explorationSetting_;
            }

            void setInitialLambda(double initial_lambda)
            {
                initialLambda_ = initial_lambda;
            }

            double getInitialLambda() const
            {
                return initialLambda_;
            }

            void setUpdateFrequency(unsigned int update_freq)
            {
                nth_step_ = update_freq;
            }

            unsigned int getUpdateFrequency() const
            {
                return nth_step_;
            }

            void setOriginalExtend(bool use_original_extend) {
                originalExtend_ = use_original_extend;
            }

            bool getOriginalExtend() const {
                return originalExtend_;
            }

            void setInefficiencyThreshold(double threshold)
            {
                inefficiencyThreshold_ = threshold;
            }

            double getInefficiencyThreshold() const
            {
                return inefficiencyThreshold_;
            }

        private:
            void omplState2armaVec(const ompl::base::State *s, arma::vec &q) const;

            void armaVec2omplState(const arma::vec &q, ompl::base::State *s) const;
            
            /** SynergyTree associated with the space. */
            SynergyTree *st_;

            std::map<unsigned int,std::pair<std::set<unsigned  int>,std::set<unsigned  int> > > robotJoint;

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

            /** The number of steps until lambda is updated and efficiency metrics are reset. */
            unsigned int nth_step_;

            /** Current number of steps since lambda was updated/initialized. */
            unsigned int step_;

            /** Average norm of vectors in the field. */
            double meanNorm_;

            /** Dimensionality of vector field */
            unsigned int vfdim_;

            bool originalExtend_;

            double inefficiencyThreshold_;
        };
    }
}
#endif
