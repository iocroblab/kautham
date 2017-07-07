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


#if !defined(_omplcPLANNER_H)
#define _omplcPLANNER_H

#if defined(KAUTHAM_USE_OMPL)
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

#include <ompl/base/goals/GoalStates.h>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/planner.h>


using namespace std;


namespace Kautham {
/** \addtogroup ControlPlanners
 *  @{
 */

  namespace omplcplanner{

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //AUXILIAR Functions
    bool isStateValid(const oc::SpaceInformation *si, const ob::State *state, Planner *p);


    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class KinematicRobotModel
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //! This class represents the motion of the robot
    //! The derived classes should reimplement the operator() that defines qdot = f(q, u)
    class KinematicRobotModel
    {
    public:
        KinematicRobotModel(const ob::StateSpacePtr space);
        virtual void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const;
        void update(ob::State *state, const std::valarray<double> &dstate) const;
        void setParameter(int i, double d);
        double getParameter(int i);
    protected:
        //const ob::StateSpace *space_;
        const ob::StateSpacePtr space_;
        vector<double> param_;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class EulerIntegrator
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //! This class is a simple integrator (Euclidean method)
    //! It is a template class, and the ode used is defined by F.
    template<typename F>
    class EulerIntegrator
    {
    public:
        EulerIntegrator(const ob::StateSpacePtr space, double timeStep) : space_(space), timeStep_(timeStep), ode_(space)
        {
        }

        void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
        {
            double t = timeStep_;
            std::valarray<double> dstate;
            space_->copyState(result, start);
            //integrates during a while (defined by the duration parameter)
            while (t < duration + std::numeric_limits<double>::epsilon())
            {
                //qdot = f(q, u). From result(q), apply control(u) to obtain the increment dstate (qdot)
                ode_(result, control, dstate);

                /**/
                //update, i.e. y(n+1) = y(n) + d, where d is a fraction of the computed dstate (e.g. timeStep_ = 0.001)
                ode_.update(result, timeStep_ * dstate);
                t += timeStep_;
            }
            if (t + std::numeric_limits<double>::epsilon() > duration)
            {
                ode_(result, control, dstate);
                ode_.update(result, (t - duration) * dstate);
            }
        }

        double getTimeStep(void) const
        {
            return timeStep_;
        }

        void setTimeStep(double timeStep)
        {
            timeStep_ = timeStep;
        }

        F* getOde()
        {
            return &ode_;
        }

    private:
        const ob::StateSpacePtr space_;
        double timeStep_;
        F  ode_;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class omplcPlannerStatePropagator
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //! This class implements the state propagator used for all the omplcPlanners.
    //! It used the EulerIntegrator class to define an integrator for the propagate function.
    //! It derives from the oc::StatePropagator and includes some functions to set and get the integration time step.
    template<typename KinModel>
    class omplcPlannerStatePropagator : public oc::StatePropagator
    {
    public:
        omplcPlannerStatePropagator(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si),
                                                                 integrator_(si->getStateSpace(), 0.0)
        {
        }

        void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
        {
            integrator_.propagate(state, control, duration, result);
        }

        void setIntegrationTimeStep(double timeStep)
        {
            integrator_.setTimeStep(timeStep);
        }

        double getIntegrationTimeStep(void) const
        {
            return integrator_.getTimeStep();
        }

        EulerIntegrator<KinModel> *getIntegrator()
        {
            return &integrator_;
        }

    private:
        EulerIntegrator<KinModel> integrator_;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class omplcPlanner
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //! This class is the base class for all the kautham planners that use the ompl::control planners.
    class omplcPlanner:public Planner {
	    public:
        omplcPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, oc::SimpleSetup *ssptr);
        ~omplcPlanner();
        
		bool trySolve();
		bool setParameters();
        //Add public data and functionsvoid
        SoSeparator *getIvCspaceScene();//reimplemented
        void drawCspace();
        void drawCspaceSE3();
        void drawCspaceRn();

        void omplState2smp(const ob::State *state, Sample* smp);
        void smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate);
        void omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp);
        void filterBounds(double &l, double &h, double epsilon);

        //! Returns the simple setup pointer
        inline oc::SimpleSetupPtr SimpleSetupPtr() {return ss;}

        //! Returns a pointer to the simple setup
        inline oc::SimpleSetup *SimpleSetup() {return ss.get();}

        inline ob::StateSpacePtr getSpace(){return space;}
		protected:
		//Add protected data and functions
        KthReal _planningTime;

        oc::SimpleSetupPtr ss;
        ob::StateSpacePtr space;
        oc::ControlSpacePtr spacec;


        bool _incremental;

	    private:
		//Add private data and functions
	  };
  }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplcPLANNER_H

