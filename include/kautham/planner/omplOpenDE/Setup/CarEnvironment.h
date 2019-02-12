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

/* Author: Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_CarEnvironment_H)
#define _CarEnvironment_H
#define dDOUBLE
#define _USE_MATH_DEFINES


#include <iostream>
#include <ompl/base/ProjectionEvaluator.h>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;
namespace Kautham
{
/** \addtogroup Environment
 *  @{
 */
namespace omplcplanner
{
//!This class defines the pure virtual and virtual function of OpenDeEnviroment class for the CarEnvironment. It defines the control dimension for the robot,
//!control bounds, how the control will applied to the robot (such as in term of forces or velocities), how the robot will interact with
//!the environment (by defining isValidCollision), and the contact dynamics.
class CarEnvironment: public KauthamDEEnvironment
{
public:

    CarEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,
                   KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain);//!< Constructor define the 2D robot environment by calling the KauthamDEEnvironment.
    ~CarEnvironment(void);
    std::string robBase;
    std::string floor;
    virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.
    virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, minimum and maximum control range.
    virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities or torques.
    virtual bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact& /*contact*/) const ;//!< This function defines the validity of the collisions.
    virtual void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const; //!< This method set the parameters for the contact.

};
/////////////////////////////////////////////////////////////////////////////////
///                        Car State Space
/////////////////////////////////////////////////////////////////////////////////

/*! The CarStateSpace intherits from OpenDEStateSpace and defines the distance function and the registerprojections.
 * An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 * position,linear and angular velocity and then a SO3 that represents the orientation
 */
class CarStateSpace : public oc::OpenDEStateSpace
{
public:
    CarStateSpace(const oc::OpenDEEnvironmentPtr &env);//!< Constructor
    ~CarStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.
};
/////////////////////////////////////////////////////////////////////////////////
///                   Car Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
/*! this class define how the state will be projected. this class inherit from the
 * ProjectionEvaluator and define the virtual functions.
 */
class CarStateProjectionEvaluator: public ob::ProjectionEvaluator
{
public:
    CarStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, Kautham::VectorRef projection) const override;//!< This function calculate the projections
};
/*! Car control sampler will defines that hwo the control will be sampled.
 */
class CarControlSampler : public oc::RealVectorControlUniformSampler
{
public:

    CarControlSampler(const oc::ControlSpace *cm) ;
    virtual void sampleNext(oc::Control *control, const oc::Control *previous);//!< This function define the way of sampling controls.
    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/);

};
/*! Car control space will define the pointer to the defined control sampler
 */
class CarControlSpace : public oc::OpenDEControlSpace
{
public:
    CarControlSpace(const ob::StateSpacePtr &m);
    virtual oc::ControlSamplerPtr allocControlSampler(void) const;//!< This function set the pointer to the CarControlSampler.

};

}
/** @}   end of Doxygen module "Environment */
}
#endif  //CarEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

