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

#if !defined(_TX90Environment_H)
#define _TX90Environment_H
#define dDOUBLE
#define _USE_MATH_DEFINES
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>
#include <kautham/problem/workspace.h>
#include <kautham/planner/planner.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;
namespace Kautham {

/** \addtogroup Planner
 *  @{
 */
namespace omplcplanner{

//! This class define the pure virtual methods of KauthamDEEnvironment and define how the control will be applied on the bodies.
 class YumiEnvironment: public KauthamDEEnvironment
 {
     public:

    YumiEnvironment(WorkSpace* ws, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain);//!< Constructor define the robot environment(i.e. table environment ) by calling the KauthamDEEnvironment.
     ~YumiEnvironment(void);

     virtual unsigned int getControlDimension(void) const;//!< describe the number of parameter used to describe control input.
     virtual void getControlBounds (std::vector< double > &lower, std::vector< double > &upper) const;//!< describe the control bounds, the bounding box to performe sampling
     virtual void applyControl (const double *control) const;//!< This function apply the control by setting the forces, velocities and torques.
     //virtual void SetPlanningParameters();//!< Set the planning parameters for planner.
     virtual bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact& /*contact*/) const;
     virtual void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const; //!< This method set the parameters for the contact.
     std::vector<float> jointValue;

 };
 ////////////////////////////////////////////////////////////////////////////////
 ///                      yumi State Validity Checker
 /////////////////////////////////////////////////////////////////////////////////
 class YumiStateValidityChecker : public oc::OpenDEStateValidityChecker
 {
 public:
     YumiStateValidityChecker(const oc::SpaceInformationPtr &si);
     virtual bool isValid(const ob::State *state) const;
 };
////////////////////////////////////////////////////////////////////////////////
///                     TX90 State Space
/////////////////////////////////////////////////////////////////////////////////
/*! The KauthamDEStateSpace intherits from KauthamDEStateSpace and just defines the method distance and the registerprojections.
 *  An OpenDEStateSpace inherits from a CompoundStateSpace where each body has three RealVectorSstateSpace representing the
 *  position,linear and angular velocity and then a SO3 that represents the orientation
 */
class YumiStateSpace : public oc::OpenDEStateSpace
{
private:
    std::vector<unsigned int> index;
    std::vector<std::string> linkname;
public:
    YumiStateSpace(const oc::OpenDEEnvironmentPtr &env);//!< Constructor
    ~YumiStateSpace();
    virtual double distance(const ob::State *s1, const ob::State *s2) const; //!< Define the method to compute the distance.
    virtual void registerProjections(void); //!< This function register the projetions for state space.

};

/////////////////////////////////////////////////////////////////////////////////
///                TX90 Projection Evaluator
/////////////////////////////////////////////////////////////////////////////////
/*! this class define how the state will be projected. this class inherit from the
 *  KauthamDEStateProjectionEvaluator and define the virtual functions.
 */
class YumiStateProjectionEvaluator: public ob::ProjectionEvaluator
{

public:
    char Planfor;
    YumiStateProjectionEvaluator(const ob::StateSpace *space); //!< Constructor
    virtual unsigned int getDimension(void) const; //!< This function returns the dimension of the projection.
    virtual void defaultCellSizes(void);//!< This function set the default dimension of the cell for projection.
    virtual void project(const ob::State *state, Kautham::VectorRef projection) const override;//!< This function calculate the projections
};

class YumiControlSampler : public oc::RealVectorControlUniformSampler
{
private:

    Planner *_planner;
    ob::State *goal;
    unsigned int numcontrolssmp;
    oc::SpaceInformationPtr sii;
    double distanceThreshold;
public:
unsigned int stps;
    YumiControlSampler(const oc::ControlSpace *cm, Planner *planner);
    virtual void sample(oc::Control *control);
    //unsigned int HSample(oc::Control *control, const oc::Control *previous, const ob::State* state);
};
class YumiControlSpace : public oc::OpenDEControlSpace
{
public:
    YumiControlSpace(const ob::StateSpacePtr &m, Planner *planner);
    virtual oc::ControlSamplerPtr allocControlSampler(void) const;
    Planner *_planner;
};

}
/** @}   end of Doxygen module "Planner */

}

#endif  //_KauthamOpenDE3RobotEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
