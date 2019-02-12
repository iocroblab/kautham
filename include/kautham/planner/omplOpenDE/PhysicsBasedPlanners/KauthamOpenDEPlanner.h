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

/* Author: Joan Fontanals Martinez, Muhayyuddin */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_KauthamOpenDEplanner_H)
#define _KauthamOpenDEplanner_H
#define dDOUBLE
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/config.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/Control.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/config.h>
#include <iostream>

#include <ode/ode.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamDEGoalRegion.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamDEGoalSamplableRegion.h>
#include <kautham/planner/planner.h>
#include <kautham/sampling/state.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>

#include"displayOpenDE.h"

#define _USE_MATH_DEFINES

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;

namespace Kautham {
/** \addtogroup Planner
 *  @{
 */
namespace omplcplanner{


/////////////////////////////////////////////////////////////////////////////////////////////////
// Class KauthamOpenDEPlanner
/////////////////////////////////////////////////////////////////////////////////////////////////

/*! KauthanDEplanner is the base class for all the planner that use the dynamic enviroment for planning.
 * All the planners will be drived from this class and reimplement the trysolve function.
 */
class KauthamDEPlanner: public Planner
{

public:

    KthReal _propagationStepSize; //!< Define the step size of the world.
    KthReal _maxspeed; //!< describe the max. speed of motors.
    bool _onlyend; //!< describe that only TCP will move of complete robot.
    KthReal _planningTime; //!< describe the max. planning time.
    KthReal _maxContacts; //!< describe the max. No of contactes to be considered in ODE when two bodies are in contact.
    KthReal _minControlSteps;//!< Define the minimum number of time a control will be applied
    KthReal _maxControlSteps;//!< Define the max number of time a control will be applies.
    KthReal _controlDimensions;//!< specify the number of control dimension.
    KthReal _erp;//!< Represents the value f error reduction parameter for ODE.
    KthReal _cfm;//!< Represents constraint force mixing for ODE.
    bool _isKchain;
    int _drawnrobot; //!< Index of the robot whose Cspace is drawn. Defaults to 0.
    double Action;
    std::vector<double> JerkIndex;//!< vector of jerk computed by doing post processing over the path.
    double PowerConsumed;//!< describes the power consumed while moving along the path.
    double Smoothness;//!< describe the smoothness of the path
    std::string PROBTYPE;//!< represents type of problem, single query of multiple query problem.


    ob::StateSpacePtr stateSpacePtr; //!< state space pointer to KauthamDEStateSpace.
    oc::OpenDEEnvironmentPtr envPtr; //!< pointer to KauthamDE enviroment.
    oc::OpenDEStateSpace *stateSpace; //!< pointer to kauthamDEStatespace.
    oc::ControlSpacePtr csp;//!< pointer to ControlSpace.
    oc::OpenDESimpleSetup *ss;//!< pointer to Simple Setup.
    vector<State>  worldState;
    std::vector<Sample*> Rob;
    std::vector<Sample*> Obs;

    std::vector<double> lastState;
    //! The constructor will define all the necessary parameters for planning.
    KauthamDEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
    ~KauthamDEPlanner();
    virtual bool trySolve();//!< Compute the path and returns the boolean value.
    bool setParameters();//!< set the planning parameters.
    void KauthamOpenDEState2Robsmp(const ob::State *state, Sample* smp);
    void KauthamOpenDEState2Obssmp(const ob::State *state, Sample* smp,const oc::Control *control,const double duration);
    void moveAlongPath(unsigned int step);
    SoSeparator *getIvCspaceScene();//reimplemented
    void drawCspace(int numrob=0);
    void ComputeAction(const std::vector<ob::State*> &states, const std::vector<oc::Control*> &control, const std::vector<double> duration);
    void ComputeJerkIndex(const std::vector<ob::State*> &states, const std::vector<double> duration);
    void ComputePowerConsumed(const std::vector<ob::State*> &states,const std::vector<oc::Control*> &control, const std::vector<double> duration);
    void ComputePowerConsumed(const std::vector<ompl::control::Control *> &control, const std::vector<double> duration);
    bool computePath(oc::OpenDESimpleSetup *ssetup, ob::RealVectorBounds vb,ob::RealVectorBounds bounds, double x, double y,double planningTime);
    bool setKauthamOpenDEState(Sample *smp);
    std::vector<float> ComputeRn(const ob::State *state);
    ompl::control::PathControl *RectMotion();
    void callDrawStuffViewer(void );

    typedef struct
    {
        KthReal objectposition[3];
        KthReal objectorientation[4];
    }KauthamDEobject;

    typedef struct
    {
        std::vector<ob::State*> substates;
        std::vector<oc::Control*> control;
        std::vector<double> duration;
    }solutionStates;
    std::vector<solutionStates> sStates;

    typedef struct
    {
        std::vector<double> pose;
        std::string action;
        std::vector<double> f;
        unsigned int targetbody;
    }query;
    vector<KauthamDEobject> smp2KauthamOpenDEState(WorkSpace *wkSpace,Sample *smp);

};

}
/** @}   end of Doxygen module "Planner */
}

#endif  //_KauthamOpenDEplanner_H
#endif  //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

