/*************************************************************************\
   Copyright 2015 Institute of Industrial and Control Engineering (IOC)
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

//#if defined(KAUTHAM_USE_OMPL)
//#if defined(KAUTHAM_USE_ODE)

#if !defined(_PlanarChainEnvironment_H)
#define _PlanarChainEnvironment_H
#define dDOUBLE

#include <iostream>
#include<kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>

#define _USE_MATH_DEFINES


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;
namespace Kautham {

/** \addtogroup Environment
 *  @{
 */
namespace omplcplanner{

//!This class defines the pure virtual and virtual function of OpenDEEnviroment class for the PlanarChainEnvironment.
//!defines the control dimension for the robot,control bounds, how the control will applied to the robot (such as in
//!terms of forces or velocities), how the robot will interact with the environment (by defining isValidCollision),
//!and the contact dynamics.
class PlanarChainEnvironment: public KauthamDEEnvironment
{
public:
    std::vector<double> *configuration;

    PlanarChainEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain);//!< Constructor define the robot environment(i.e. table environment ) by calling the KauthamDEEnvironment.
    ~PlanarChainEnvironment(void);
    virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.
    virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, minimum and maximum control range.
    virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities or torques.
    virtual bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact& /*contact*/) const ;//!< This function defines the validity of the collisions.
    virtual void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const; //!< This method set the parameters for the contact.

};
////////////////////////////////////////////////////////////////////////////////
///                      Planar State Space
/////////////////////////////////////////////////////////////////////////////////

/*! The PlanarChainStateSpace intherits from KauthamDEStateSpace and defines the methods distance and the registerprojections.
 *  An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 *  position,linear and angular velocity and then a SO3 that represents the orientation
 */
class PlanarChainStateSpace : public oc::OpenDEStateSpace
{

public:
    PlanarChainStateSpace(const oc::OpenDEEnvironmentPtr &env);//!< Constructor
    ~PlanarChainStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.
};

/////////////////////////////////////////////////////////////////////////////////
///                Planar chain Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
/*! this class define how the state will be projected. this class inherit from the
 *  ProjectionEvaluator and define the virtual functions.
 */
class PlanarChainStateProjectionEvaluator: public ob::ProjectionEvaluator
{
public:
    PlanarChainStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, Kautham::VectorRef projection) const override;//!< This function defines that how the state will be projected
};
/*! PlanarChainControlSampler wil define the way the control will be applied
 */
class PlanarChainControlSampler : public oc::RealVectorControlUniformSampler
{
public:

    PlanarChainControlSampler( const oc::ControlSpace *cm);
    virtual void sampleNext(oc::Control *control, const oc::Control *previous);//!< This function define the way of sampling controls.
    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*/);

};
/*! Car control space will define the pointer to the defined control sampler
 */
class PlanarChainControlSpace : public oc::OpenDEControlSpace
{
public:

    PlanarChainControlSpace(const ob::StateSpacePtr &m);
    virtual oc::ControlSamplerPtr allocControlSampler(void) const;

};

}
/** @}   end of Doxygen module "Environment */

}

#endif  //PlanarChainEnvironment_H
//#endif //KAUTHAM_USE_ODE
//#endif // KAUTHAM_USE_OMPL
