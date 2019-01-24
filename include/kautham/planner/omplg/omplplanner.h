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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

#if !defined(_omplPLANNER_H)
#define _omplPLANNER_H
#if defined(KAUTHAM_USE_OMPL)
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <ompl/util/Exception.h>
namespace og = ompl::geometric;
namespace oc = ompl::control;
namespace ob = ompl::base;


#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/planner.h>


using namespace std;

namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */
    namespace omplplanner{


        /////////////////////////////////////////////////////////////////////////////////////////////////
        //AUXILIAR Functions
        ob::StateSamplerPtr allocStateSampler(const ob::StateSpace *mysspace, Planner *p);
        ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, Planner *p);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Class weigthedRealVectorStateSpace
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This class represents a RealVectorStateSpace with a weighted distance. It is used to describe the state space of
        //! the kinematic chains where different weights are set to the joints.
        class weigthedRealVectorStateSpace:public ob::RealVectorStateSpace
        {
        public:
            weigthedRealVectorStateSpace(unsigned int dim=0);
            ~weigthedRealVectorStateSpace(void);
            void setWeights(vector<KthReal> w);
            double distance(const ob::State *state1, const ob::State *state2) const;
        protected:
            vector<KthReal> weights;
        };

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Class KauthamValidStateSampler
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This class represents a valid state sampler based on the sampling of the Kautham control space and
        //! its conversion to samples. Does a collision-check to verify validity.
        class KauthamValidStateSampler : public ob::ValidStateSampler
        {
        public:
            KauthamValidStateSampler(const ob::SpaceInformation *si, Planner *p);
            //virtual bool sample(ob::State *state, string samplername="Random");
            virtual bool sample(ob::State *state);
            virtual bool sampleNear(ob::State *state, const ob::State *near, const double distance);

        protected:
            ompl::RNG rng_; //random generator
            Planner *kauthamPlanner_; //pointer to planner in order to have access to the workspace
            const ob::SpaceInformation *si_;
            vector<Sampler*> _samplerVector;
            SDKSampler* _samplerSDK;
            HaltonSampler* _samplerHalton;
            GaussianSampler* _samplerGaussian;
            RandomSampler* _samplerRandom;
        };


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Class KauthamStateSampler
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This class represents a  state sampler based on the sampling of the Kautham control space and
        //! its conversion to samples.
        class KauthamStateSampler : public ob::CompoundStateSampler
        {
        public:
            KauthamStateSampler(const ob::StateSpace *sspace, Planner *p);
            ~KauthamStateSampler();
            void setCenterSample(ob::State *state, double th);
            virtual void sampleUniform(ob::State *state);
            virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);

            RandomSampler* _samplerRandom;
            //HaltonSampler* _samplerHalton;

        protected:
            ompl::RNG rng_; //!< random generator
            Planner *kauthamPlanner_; //!< pointer to planner in order to have access to the workspace
            Sample *centersmp;
            double threshold;
        };



        ////////////////////////////////////////////////////////////////////////////////////////
        //Class omplPlanner
        ////////////////////////////////////////////////////////////////////////////////////////
        class omplPlanner:public Planner {
        public:
            //Add public data and functions
            omplPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr);
            ~omplPlanner();

            bool trySolve();//reimplemented
            bool setParameters();//reimplemented
            SoSeparator *getIvPathScene();//reimplemented
            SoSeparator *getIvCspaceScene();//reimplemented
            void drawPath(bool show);
            void drawCspace(unsigned int robot = 0, unsigned int link = 0);

            void omplState2smp(const ob::State *state, Sample* smp);
            void smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate);
            void omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp);
            inline ob::StateSpacePtr getSpace(){return space;}
            void filterBounds(double &l, double &h, double epsilon);

            inline void setSamplerUsed(int su){_samplerUsed=su;}
            inline int getSamplerUsed(){return _samplerUsed;}

            inline bool getInterpolate() { return _interpolate;}
            inline void setInterpolate(bool interpolate) {_interpolate = interpolate;}

            void disablePMDControlsFromSampling(bool enableall=false);
            inline vector<int> *getDisabledControls(){return &_disabledcontrols;}

            og::SimpleSetupPtr ss;

            //! Returns the simple setup pointer
            inline og::SimpleSetupPtr SimpleSetupPtr() {return ss;}

            //! Returns a pointer to the simple setup
            inline og::SimpleSetup *SimpleSetup() {return ss.get();}

        protected:
            //Add protected data and functions
            KthReal _planningTime;
            //og::SimpleSetupPtr ss;
            ob::StateSpacePtr space;
            ob::SpaceInformationPtr si;


            unsigned int _validSegmentCount;
            int _samplerUsed;
            unsigned int _simplify;
            bool _interpolate;
            bool _incremental;
            unsigned _drawnrobot; //!< Index of the robot whose Cspace is drawn. Defaults to 0.
            bool _drawnPath; //! Flag to show/hide path into workspace scene
            vector<int> _disabledcontrols;//!< those disabled controls will not be sampled, they are fixed at 0.5

        private:
            //Add private data and functions
        };
    }
    /** @}   end of Doxygen module */
}
#endif // KAUTHAM_USE_OMPL
#endif  //_omplPLANNER_H

