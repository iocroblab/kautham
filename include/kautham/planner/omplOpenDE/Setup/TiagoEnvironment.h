
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

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_TiagoEnvironment_H)
#define _TiagoEnvironment_H
#define _USE_MATH_DEFINES
#define dDOUBLE
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>
#include <kautham/problem/workspace.h>
#include <kautham/planner/planner.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <random>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;
namespace Kautham {

/** \addtogroup Planner
 *  @{
 */
namespace omplcplanner{

//!This class defines the pure virtual and virtual function of OpenDEEnviroment class for the Kuka robot environment. It defines the control dimension for the robot,
//!control bounds, how the control will applied to the robot (such as in term of forces or velocities), how the robot will interact with
//!the environment (by defining isValidCollision), and the contact dynamics.
class TiagoEnvironment: public KauthamDEEnvironment
{
public:
    WorkSpace *wkspace;
    TiagoEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain);//!< Constructor define the robot environment(i.e. table environment ) by calling the KauthamDEEnvironment.

    virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.

    virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, minimum and maximum control range.

    virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities or torques.

    virtual bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact& /*contact*/) const ;//!< This function defines the validity of the collisions.

    virtual void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const; //!< This method set the parameters for the contact.

    float threshold;
    std::vector<float> jointValue;
    ~TiagoEnvironment(void);




};
////////////////////////////////////////////////////////////////////////////////
///                      Kuka State Validity Checker
/////////////////////////////////////////////////////////////////////////////////
class TiagoStateValidityChecker : public oc::OpenDEStateValidityChecker
{
public:
    TiagoStateValidityChecker(const oc::SpaceInformationPtr &si);
    virtual bool isValid(const ob::State *state) const;
};

//class KauthamRealVectorStateSampler : public ob::RealVectorStateSampler
//{
//public:

//    /** \brief Constructor */
//    KauthamRealVectorStateSampler(const ob::StateSpace *space) : ob::RealVectorStateSampler(space) {  }

//    /** \brief Destructor. This frees the added samplers as well. */
//    virtual ~KauthamRealVectorStateSampler(){  }
//    /** \brief State sampler for the R<sup>n</sup> state space */
//    virtual void RegionBaisedSampling(ob::State *state, const Region region);

//};


////////////////////////////////////////////////////////////////////////////////
///                      Kuka State Space
/////////////////////////////////////////////////////////////////////////////////

/*! The KukaStateSpace intherits from KauthamDEStateSpace and defines the methods distance and the registerprojections.
 *  An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 *  position,linear and angular velocity and then a SO3 that represents the orientation
 */
class TiagoStateSpace : public oc::OpenDEStateSpace
{

    // oc::OpenDEEnvironmentPtr envp;

public:
    std::vector<dJointID> jointValue;

    TiagoStateSpace(const oc::OpenDEEnvironmentPtr &env, Planner *planner );//!< Constructor
    ~TiagoStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.
    Planner *_planner;
};

/////////////////////////////////////////////////////////////////////////////////
///                Kuka Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
/*! this class define how the state will be projected. this class inherit from the
 *  ProjectionEvaluator and define the virtual functions.
 */
class TiagoStateProjectionEvaluator: public ob::ProjectionEvaluator
{
public:
    TiagoStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const;//!< This function defines that how the state will be projected
};


class TiagoControlSampler : public oc::RealVectorControlUniformSampler
{
private:

    Planner *_planner;
    ob::State *goal;
    unsigned int numcontrolssmp;
    oc::SpaceInformationPtr sii;
    double distanceThreshold;
public:
unsigned int stps;
    TiagoControlSampler(const oc::ControlSpace *cm, Planner *planner);
    virtual void sample(oc::Control *control);
};

class TiagoControlSpace : public oc::OpenDEControlSpace
{
public:

    TiagoControlSpace(const ob::StateSpacePtr &m, Planner *planner);
    virtual oc::ControlSamplerPtr allocControlSampler(void) const;
    Planner *_planner;
};

}
/** @}   end of Doxygen module "Planner */

}

#endif  //_KauthamOpenDE3RobotEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

