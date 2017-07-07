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

/* Author: Alexander Perez, Jan Rosell */

#if defined(KAUTHAM_USE_OMPL)
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <kautham/planner/omplc/omplcRRTdualdriveplanner.h>



namespace Kautham {
  namespace omplcplanner{


  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Class KinematicDualDriveModel
  /////////////////////////////////////////////////////////////////////////////////////////////////
  //! This class derives from KinematicRobotModel and reimplements the operator() function that
  //! defines the qdot = f(q, u) equations of a dualdrive
  class KinematicDualDriveModel : public KinematicRobotModel
  {
  public:

      /// Constructor
      KinematicDualDriveModel(const ob::StateSpacePtr space) : KinematicRobotModel(space)
      {
          param_.resize(2);
      }

      /// implement the function describing the robot motion: qdot = f(q, u)
      void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
      {
          //Call the parent operator to resize the dstate
          KinematicRobotModel::operator()(state,  control, dstate);

          //Get the control values
          const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

          //Get the subspace corresponding to robot 0
          ob::StateSpacePtr ssRobot0 = ((ob::StateSpacePtr) space_->as<ob::CompoundStateSpace>()->getSubspace(0));

          //Get the SE3 subspace of robot 0
          ob::StateSpacePtr ssRobot0SE3 =  ((ob::StateSpacePtr) ssRobot0->as<ob::CompoundStateSpace>()->getSubspace(0));

          //Retrieve the SE3 configuration
          ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobot0SE3);
          ob::ScopedState<ob::CompoundStateSpace> sstate(space_);
          sstate = *state;
          sstate >> pathscopedstatese3;

          //Get the current orientation (theta)
          mt::Rotation ori(pathscopedstatese3->rotation().x,
                           pathscopedstatese3->rotation().y,
                           pathscopedstatese3->rotation().z,
                           pathscopedstatese3->rotation().w);


          //computes dstate, the se3 incremental motion
          //translation
          //mt::Vector3 advance(0.0,u[0], 0.0);//we move along the y axis when angle is zero
          double v = R * (u[0] + u[1])/2;
          mt::Vector3 advance(v, 0.0, 0.0);//we move along the x axis when angle is zero
          mt::Vector3 v2 = ori(advance);
          dstate[0] = v2[0];
          dstate[1] = v2[1];
          dstate[2] = v2[2];
          /*
          mt::Unit3 axis;
          float theta;
          ori.getAxisAngle(axis, theta);
          //correct theta since axis may be (0,0,1) or (0,0,-1)!
          //this assures that theta is a rotation along the positive z-axis:
          theta = axis[2]*theta;
          dstate[0] = u[0] * cos(theta+M_PI/2);//the +M_PI/2 is to move along the y axis when angle is zero
          dstate[1] = u[0] * sin(theta+M_PI/2);
          dstate[2] = 0;
          */


          //rotation: axis-angle
          dstate[3] = 0.0; //rotation along the z-axis
          dstate[4] = 0.0;
          dstate[5] = 1.0;
          //
         
         dstate[6] = R *(u[0] - u[1])/(2*D);
      }
      	   

      /// Sets the dualdrive raduis
      void setDualDriveRadius(double r)
      {
          setParameter(0, r);
          R = r;
      }

      /// Gets the dualdrive radius
      double getDualDriveRadius()
      {
          return getParameter(0);
      }
      /// Sets the dualdrive distance
      void setDualDriveDistance(double d)
      {
          setParameter(0, d);
          D = d;
      }

      /// Gets the dualdrive distance
      double getDualDriveDistance()
      {
          return getParameter(0);
      }
   private:
      double R;
      double D;
  };


  /////////////////////////////////////////////////////////////////////////////////////////////////
  // omplcRRTdualdrivePlanner functions
  /////////////////////////////////////////////////////////////////////////////////////////////////
	//! Constructor
    omplcRRTdualdrivePlanner::omplcRRTdualdrivePlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, oc::SimpleSetup *ssptr):
              omplcPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl cRRT Planner";
        _idName = "omplcRRTdualdrive";

        // set the bounds for the control space
        _controlBound = 10;
        _onlyForward = 0;
        addParameter("ControlBound", _controlBound);
        addParameter("OnlyForward (0/1)", _onlyForward);

        if (ss.get() == NULL) {
            // create a control space
            int numcontrols = 2;
            spacec = ((oc::ControlSpacePtr) new oc::RealVectorControlSpace(space, numcontrols));
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(0, - _controlBound * (1 - _onlyForward));
            cbounds.setHigh(0, _controlBound);
            cbounds.setLow(1, - _controlBound * (1 - _onlyForward));
            cbounds.setHigh(1, _controlBound);
            spacec->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

            // define a simple setup class
            ss = ((oc::SimpleSetupPtr) new oc::SimpleSetup(spacec));

            //Add start states
            ss->clearStartStates();
            for (std::vector<Sample*>::const_iterator start(_init.begin());
                 start != _init.end(); ++start) {
                //Start state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> startompl(space);
                smp2omplScopedState(*start,&startompl);
                cout << "startompl:" << endl;
                startompl.print();
                ss->addStartState(startompl);
            }

            //Add goal states
            ob::GoalStates *goalStates(new ob::GoalStates(ss->getSpaceInformation()));
            for (std::vector<Sample*>::const_iterator goal(_goal.begin());
                 goal != _goal.end(); ++goal) {
                //Goal state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> goalompl(space);
                smp2omplScopedState(*goal,&goalompl);
                cout << "goalompl:" << endl;
                goalompl.print();
                goalStates->addState(goalompl);
            }
            ss->setGoal(ob::GoalPtr(goalStates));
        }




        // set state validity checking for this space
        ss->setStateValidityChecker(std::bind(&omplcplanner::isStateValid, ss->getSpaceInformation().get(), std::placeholders::_1, (Planner*)this));

        // set the propagation routine for this space
        oc::SpaceInformationPtr si=ss->getSpaceInformation();
        ss->setStatePropagator(oc::StatePropagatorPtr(new omplcPlannerStatePropagator<KinematicDualDriveModel>(si)));

        // propagation step size
        _propagationStepSize = 0.01;
        addParameter("PropagationStepSize", _propagationStepSize);
        static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->setIntegrationTimeStep(_propagationStepSize);
        si->setPropagationStepSize(_propagationStepSize);
        // propagation duration
        _durationMin = 1;
        _durationMax = 10;
        addParameter("MinDuration", _durationMin);
        addParameter("MaxDuration", _durationMax);
        si->setMinMaxControlDuration(_durationMin, _durationMax);

        //dualdriveRadius
        _dualdriveRadius = 0.2;
        addParameter("DualDriveRadius", _dualdriveRadius);
        static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->getIntegrator()->getOde()->setDualDriveRadius(_dualdriveRadius);

        //dualdriveDistance
        _dualdriveDistance = 0.2;
        addParameter("DualDriveDistance", _dualdriveDistance);
        static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->getIntegrator()->getOde()->setDualDriveDistance(_dualdriveDistance);

        // create a planner for the defined space
        ob::PlannerPtr planner(new oc::RRT(si));

        //set RRT Ggoal Bias
        _GoalBias=(planner->as<oc::RRT>())->getGoalBias();
        addParameter("Goal Bias", _GoalBias);
        planner->as<oc::RRT>()->setGoalBias(_GoalBias);

        //set the planner
        ss->setPlanner(planner);

        ss->setup();

    }

	//! void destructor
    omplcRRTdualdrivePlanner::~omplcRRTdualdrivePlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplcRRTdualdrivePlanner::setParameters(){

      omplcPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<oc::RRT>()->setGoalBias(_GoalBias);
        }
        else
          return false;

     it = _parameters.find("DualDriveRadius");
     if(it != _parameters.end()){
          _dualdriveRadius = it->second;
          static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->getIntegrator()->getOde()->setDualDriveRadius(_dualdriveRadius);
      }
      else
         return false;

    it = _parameters.find("DualDriveDistance");
     if(it != _parameters.end()){
          _dualdriveDistance = it->second;
          static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->getIntegrator()->getOde()->setDualDriveDistance(_dualdriveDistance);
      }
      else
         return false;

     it = _parameters.find("MinDuration");
     if(it != _parameters.end()){
         _durationMin = it->second;
         ss->getSpaceInformation()->setMinMaxControlDuration(_durationMin, _durationMax);
      }
      else
         return false;

     it = _parameters.find("MaxDuration");
     if(it != _parameters.end()){
         _durationMax = it->second;
         ss->getSpaceInformation()->setMinMaxControlDuration(_durationMin, _durationMax);
      }
      else
         return false;

       it = _parameters.find("PropagationStepSize");
       if(it != _parameters.end()){
           _propagationStepSize = it->second;
           static_cast<omplcPlannerStatePropagator<KinematicDualDriveModel>*>(ss->getStatePropagator().get())->setIntegrationTimeStep(_propagationStepSize);
           ss->getSpaceInformation()->setPropagationStepSize(_propagationStepSize);
        }
        else
           return false;

       if (spacec.get() != NULL) {
           it = _parameters.find("ControlBound");
           if(it != _parameters.end()){
               _controlBound = it->second;
               ob::RealVectorBounds cbounds(2);
               cbounds.setLow(0, - _controlBound * (1 - _onlyForward));
               cbounds.setHigh(0, _controlBound);
               cbounds.setLow(1, -_controlBound * (1 - _onlyForward));
               cbounds.setHigh(1, _controlBound);
               spacec->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
           }
           else
               return false;


           it = _parameters.find("OnlyForward (0/1)");
           if(it != _parameters.end()){
               if(it->second != 0) _onlyForward=1;
               else _onlyForward=0;
               ob::RealVectorBounds cbounds(2);
               cbounds.setLow(0, - _controlBound * (1 - _onlyForward));
               cbounds.setHigh(0, _controlBound);
               cbounds.setLow(1, -_controlBound * (1 - _onlyForward));
               cbounds.setHigh(1, _controlBound);
               spacec->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
           }
           else
               return false;
       }

      }catch(...){
        return false;
      }
      return true;
    }
  }
}

#endif // KAUTHAM_USE_OMPL
