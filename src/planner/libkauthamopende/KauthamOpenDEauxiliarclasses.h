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

#if !defined(_KauthamOpenDEauxiliarclasses_H)
#define _KauthamOpenDEauxiliarclasses_H
#define dDOUBLE
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
#include <libompl/omplcplanner.h>
#include "KauthamOpenDEEnvironment.h"
#include <sampling/sample.h>


#define _USE_MATH_DEFINES


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;




using namespace std;

namespace Kautham {

    namespace omplcplanner{

/*The KauthamDEGoal is mainly used to set as a derived class from a GoalRegion the operation distance which the planner is going to check in order to decide wether the goal has been reached or not.
In order to check the position and orientation of the objects at the goal configuration we define a method called smp2KauthamOpenDEState .
This smp2KauthamOpenDEState method moves the robot to its goal configuration and from them extracts the position and orientation of each element in the scene and fills the KauthamOdeobject structure.
We've added the onlyend bool for further improvements as we have considered interesting to set the possibility that the goal position to be just the position of the ending point (the tcp) instead of the position of each element.
the distance is considered to be the addition of the differences between each considered element's position and its goal position
*/
//! The KauthamDEGoal is mainly used to set as a derived class from a GoalRegion the operation distance which the planner is going to check in order to decide wether the goal has been reached or not.
class KauthamDEGoal : public ob::GoalRegion
{
public:
    KauthamDEGoal(const ob::SpaceInformationPtr &si, WorkSpace* ws, bool a,Sample *goal);
    ~KauthamDEGoal();
    virtual double distanceGoal(const ob::State *st) const; //!< This function compute the distance from the goal.
  
/*! This structure keeps the position and orientation of each element of the robot once the configuration has been moved GOAL.
 *   This information can be used to compute the distance of any configuration.
 */
   typedef struct
    {
        KthReal objectposition[3];
        KthReal objectorientation[4];
    }KauthamODEobject;
    //En aquesta estructura es guarda la posició i la orientació de cada element del robot una vegada aquest s'ha mogut a la configuració de 		goal.
    //Així aquesta informació la puc utilitzar a la hora de calcular la distancia de qualsevol configuració a la configuració del GOAL.

    
    vector<KauthamODEobject> smp2KauthamOpenDEState(WorkSpace* w,Sample *goal);//!< This function check the position and orientation of the objects at the goal configuration.

    vector<KauthamODEobject> Kauthamodebodies; //!<  Kauthamodebodies is a vector KauthamODEobject which keeps information on all the elements that make up the kinematic chain of the robot.

    //Kauthamodebodies es un vector de KauthamODEobject on es guarda la informació de tots els elements que formen la cadena cinemàtica del robot.

   
    bool onlyend; //!< It is meant to adapt to the possibility that the planner GOAL involves only the TCP.
    //Esta pensat per poder adaptar el planner a la possibilitat que el GOAL només involucri al TCP ja que podria ajudar en tasques en que l'important sigui la posició del manipulador.

};


/*! The KauthamDEStateSpace intherits from OpenDEStateSpace and just defines the method distance and the registerprojections.
 * An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 * position,linear and angular velocity and then a SO3 that represents the orientation
 */
class KauthamDEStateSpace : public oc::OpenDEStateSpace
{
public:
    KauthamDEStateSpace(const oc::OpenDEEnvironmentPtr &env);//!< Constructor
    ~KauthamDEStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.
};
/*! this class define how the state will be projected. this class inherit from the
 * ProjectionEvaluator and define the virtual functions.
 */
class KauthamDEStateProjectionEvaluator: public ob::ProjectionEvaluator
{
public:
    KauthamDEStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const;//!< This function calculate the projections
};

//this class reimplement the sampling mechanism as it is done in ompl for ode bodies
/*class KauthamDEObjectControlSampler : public oc::RealVectorControlUniformSampler
{
public:
    KauthamDEObjectControlSampler(const oc::ControlSpace *cm);
      ~KauthamDEObjectControlSampler();
    virtual void sampleNext(oc::Control *control, const oc::Control *previous);
    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*///);
//};

}

}

#endif  //_KauthamOpenDEauxiliarclasses_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
