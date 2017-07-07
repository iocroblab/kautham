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


#if defined(KAUTHAM_USE_OMPL)


#include <boost/bind/mem_fn.hpp>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoMaterial.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/goals/GoalStates.h>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/omplc/omplcplanner.h>
#include <kautham/planner/omplg/omplplanner.h>


namespace Kautham {

//! Namespace omplcplanner contains the planners based on the OMPL::control library
  namespace omplcplanner{

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // AUXILIAR functions
  /////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //! This function converts a state to a smp and tests if it is in collision or not
  bool isStateValid(const oc::SpaceInformation *si, const ob::State *state, Planner *p)
  {

      if(  si->satisfiesBounds(state)==false )
          return false;
      //create sample
      int d = p->wkSpace()->getNumRobControls();
      Sample *smp = new Sample(d);
      //copy the conf of the init smp. Needed to capture the home positions.
      smp->setMappedConf(p->initSamp()->getMappedConf());
      //load the RobConf of smp form the values of the ompl::state
      ((omplcPlanner*)p)->omplState2smp(state,smp);
      //collision-check
      if( p->wkSpace()->collisionCheck(smp) )
          return false;
      return true;
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  // KinematicRobotModel functions
  /////////////////////////////////////////////////////////////////////////////////////////////////

  /// KinematicRobotModel Constructor
  KinematicRobotModel::KinematicRobotModel(const ob::StateSpacePtr space) : space_(space)
  {
  }

   //! Implement the function describing the robot motion: qdot = f(q, u).
   //! This function is to be reimplemeted by derived classes defining different types of behavior
   //! The derived classes should first call this function, the parent operator() function, because it
   //! sets the dimension of the dstate.
   void KinematicRobotModel::operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
   {
          (void)state;//unused here
          (void)control;//unused here

          //compute the dimension of the compound statespace
          //loop for all the robots
          std::vector< ob::StateSpacePtr> sss = space_->as<ob::CompoundStateSpace>()->getSubspaces();
          std::stringstream sstm;

          int d=0;
          for(unsigned i=0;i<sss.size();i++)
          {
              //update robot i. Get the subspaces (SE3+Rn, or SE3 or Rn)
              std::vector< ob::StateSpacePtr> sssi = sss[i]->as<ob::CompoundStateSpace>()->getSubspaces();

              for(unsigned j=0;i<sssi.size();i++)
              {
                //SE3 subspace
                sstm.str("");
                sstm << "ssRobot" << i<<"_SE3";
                if(sssi[j]->getName() == sstm.str())
                {
                    //add the dimension of a se3 configuration
                    d +=7;
                }
                //Rn subspace
                sstm.str("");
                sstm << "ssRobot" << i<<"_Rn";
                if(sssi[j]->getName() == sstm.str())
                {
                    //add the dimension of the Rn configuration
                    d += sssi[j]->as<omplplanner::weigthedRealVectorStateSpace>()->getDimension();
                }
              }//end robot i
          }

          //resize the dstate
          dstate.resize(d);
          //do nothing. Derived classes should do it.
          for(int i=0;i<d; i++)
             dstate[i] = 0.0;
   }


   //! Implements y(n+1) = y(n) + d
   //! This function is called as ode_.update(...) in EulerIntegrator<KinematicRobotModel>::propagate function.
   void KinematicRobotModel::update(ob::State *state, const std::valarray<double> &dstate) const
   {
          //Create a scoped state in order to retreive each subspace in an easy way using the >> operators
          ob::ScopedState<ob::CompoundStateSpace> sstate(space_);
          sstate = *state;

          //create a scoped state to store the new state
          ob::ScopedState<ob::CompoundStateSpace> newsstate(space_);

          //loop for all the robots
          std::vector< ob::StateSpacePtr> sss = space_->as<ob::CompoundStateSpace>()->getSubspaces();
          std::stringstream sstm;

          int d=0;//index to access the increment stored in parameter dstate
          for(unsigned i=0;i<sss.size();i++)
          {
              //Update robot i.
              //Get the robot i subspaces (SE3+Rn, or SE3 or Rn)
              std::vector< ob::StateSpacePtr> sssi = sss[i]->as<ob::CompoundStateSpace>()->getSubspaces();

              //update the state of each subspace
              for(unsigned j=0;j<sssi.size();j++)
              {
                //SE3 subspace
                sstm.str("");
                sstm << "ssRobot" << i<<"_SE3";
                if(sssi[j]->getName() == sstm.str())
                {
                    //update SE3 conf
                    //retrieve the SE3 part
                    ob::ScopedState<ob::SE3StateSpace> scopedstatese3(sssi[j]);
                    sstate >> scopedstatese3;

                    //create the new SE3 part
                    ob::ScopedState<ob::SE3StateSpace> newscopedstatese3(sssi[j]);

                    //update the position
                    newscopedstatese3->setX(scopedstatese3->getX() + dstate[d++]);
                    newscopedstatese3->setY(scopedstatese3->getY() + dstate[d++]);
                    newscopedstatese3->setZ(scopedstatese3->getZ() + dstate[d++]);

                    //current orientation
                    mt::Rotation ori(scopedstatese3->rotation().x,
                                     scopedstatese3->rotation().y,
                                     scopedstatese3->rotation().z,
                                     scopedstatese3->rotation().w);
                    mt::Unit3 axis0;
                    float angle0;
                    ori.getAxisAngle(axis0, angle0);

                    //rotation to apply
                    double ax = dstate[d++];
                    double ay = dstate[d++];
                    double az = dstate[d++];
                    mt::Unit3 axis(ax,ay,az);//a unit vector is generated
                    float angle = dstate[d++];
                    mt::Rotation rot(axis, angle);

/*
                    if(axis[0]!=1.0 && axis[1]!=0.0 && axis[2]!=0.0)
                    {
                        int k;
                        k=0;
                    }
                    */

                    //update the orientation
                    mt::Rotation newori = rot*ori;
                    mt::Unit3 axis1;
                    float angle1;
                    newori.getAxisAngle(axis1, angle1);
                    newscopedstatese3->rotation().setAxisAngle(axis1[0],axis1[1],axis1[2],angle1);

                    /*
                    if(abs(axis0[0]-axis1[0])>0.001)
                    {
                        int k;
                        k=0;
                    }
                    */


                    //load the global scoped state with the info of the se3 data of robot i
                    newsstate<<newscopedstatese3;
                }

                //Rn subspace
                sstm.str("");
                sstm << "ssRobot" << i<<"_Rn";
                if(sssi[j]->getName() == sstm.str())
                {
                    //update Rn conf
                    //retrieve the Rn part
                    ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> scopedstatern(sssi[j]);
                    sstate >> scopedstatern;

                    //create the new Rn part
                    ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> newscopedstatern(sssi[j]);
                    //update the Rn part
                    for(unsigned k=0; k< sssi[j]->as<omplplanner::weigthedRealVectorStateSpace>()->getDimension();k++)
                        newscopedstatern->values[k] = scopedstatern->values[k] + dstate[d++];

                    //load the global scoped state with the info of the se3 data of robot i
                    newsstate << newscopedstatern;
                }
              }
              //end update robot i
           }
          //end loop for all robots

          //return the newstate in the parameter state
          space_->copyState(state, newsstate.get());
      }

   /// function to set parameters
   void KinematicRobotModel::setParameter(int i, double d)
   {
       param_[i] = d;
   }

   /// function to get parameters
   double KinematicRobotModel::getParameter(int i)
   {
       return param_[i];
   }



   /////////////////////////////////////////////////////////////////////////////////////////////////
   // KinematicRobotModel functions
   /////////////////////////////////////////////////////////////////////////////////////////////////

    //! Constructor
    omplcPlanner::omplcPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, oc::SimpleSetup *ssptr):
              Planner(stype, init, goal, samples, ws)
    {
        _family = OMPLCPLANNER;
        //set intial values from parent class data
        _speedFactor = 1;
        _solved = false;
        _guiName = "omplc Planner";
        _idName = "omplc Planner";

        //set own intial values
        _planningTime = 10;
        _incremental = 0;//by default makes a clear before any new call to solve in function trysolve().

        //add planner parameters
        addParameter("Incremental (0/1)",_incremental);
        addParameter("Max Planning Time", _planningTime);
        addParameter("Speed Factor", _speedFactor);


        if (ssptr == NULL) {
            //Construct the state space we are planning in. It is a compound state space composed of a compound state space for each robot
            //Each robot has a compound state space composed of a (oprional) SE3 state space and a (optional) Rn state space
            vector<ob::StateSpacePtr> spaceRn;
            vector<ob::StateSpacePtr> spaceSE3;
            vector<ob::StateSpacePtr> spaceRob;
            vector< double > weights;

            spaceRn.resize(_wkSpace->getNumRobots());
            spaceSE3.resize(_wkSpace->getNumRobots());
            spaceRob.resize(_wkSpace->getNumRobots());
            weights.resize(_wkSpace->getNumRobots());

            //loop for all robots
            for(unsigned i=0; i<_wkSpace->getNumRobots(); i++)
            {
                vector<ob::StateSpacePtr> compoundspaceRob;
                vector< double > weightsRob;
                std::stringstream sstm;

                //create state space SE3 for the mobile base, if necessary
                if(_wkSpace->getRobot(i)->isSE3Enabled())
                {
                    //create the SE3 state space
                    spaceSE3[i] = ((ob::StateSpacePtr) new ob::SE3StateSpace());
                    sstm.str("");
                    sstm << "ssRobot" << i<<"_SE3";
                    spaceSE3[i]->setName(sstm.str());

                    //set the bounds. If the bounds are equal or its difference is below a given epsilon value (0.001) then
                    //set the higher bound to the lower bound plus this eplsion
                    ob::RealVectorBounds bounds(3);

                    //x-direction
                    double low = _wkSpace->getRobot(i)->getLimits(0)[0];
                    double high = _wkSpace->getRobot(i)->getLimits(0)[1];
                    filterBounds(low, high, 0.001);
                    bounds.setLow(0, low);
                    bounds.setHigh(0, high);

                    //y-direction
                    low = _wkSpace->getRobot(i)->getLimits(1)[0];
                    high = _wkSpace->getRobot(i)->getLimits(1)[1];
                    filterBounds(low, high, 0.001);
                    bounds.setLow(1, low);
                    bounds.setHigh(1, high);

                    //z-direction
                    low = _wkSpace->getRobot(i)->getLimits(2)[0];
                    high = _wkSpace->getRobot(i)->getLimits(2)[1];
                    filterBounds(low, high, 0.001);
                    bounds.setLow(2, low);
                    bounds.setHigh(2, high);

                    spaceSE3[i]->as<ob::SE3StateSpace>()->setBounds(bounds);

                    //sets the weights between translation and rotation
                    spaceSE3[i]->as<ob::SE3StateSpace>()->setSubspaceWeight(0,_wkSpace->getRobot(i)->getWeightSE3()[0]);//translational weight
                    spaceSE3[i]->as<ob::SE3StateSpace>()->setSubspaceWeight(1,_wkSpace->getRobot(i)->getWeightSE3()[1]);//rotational weight

                    //load to the compound state space of robot i
                    compoundspaceRob.push_back(spaceSE3[i]);
                    weightsRob.push_back(1);
                }

                //create the Rn state space for the kinematic chain, if necessary
                int nj = _wkSpace->getRobot(i)->getNumJoints();
                if(nj>0)
                {
                    //create the Rn state space
                    spaceRn[i] = ((ob::StateSpacePtr) new omplplanner::weigthedRealVectorStateSpace(nj));
                    sstm.str("");
                    sstm << "ssRobot" << i<<"_Rn";
                    spaceRn[i]->setName(sstm.str());

                    // set the bounds and the weights
                    vector<KthReal> jointweights;
                    ob::RealVectorBounds bounds(nj);
                    double low, high;
                    for(int j=0; j<nj;j++)
                    {
                        //the limits of joint j between link j and link (j+1) are stroed in the data structure of link (j+1)
                        low = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(true);
                        high = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(false);
                        filterBounds(low, high, 0.001);
                        bounds.setLow(j, low);
                        bounds.setHigh(j, high);
                        //the weights
                        jointweights.push_back(_wkSpace->getRobot(i)->getLink(j+1)->getWeight());
                    }
                    spaceRn[i]->as<omplplanner::weigthedRealVectorStateSpace>()->setBounds(bounds);
                    spaceRn[i]->as<omplplanner::weigthedRealVectorStateSpace>()->setWeights(jointweights);

                    //load to the compound state space of robot i
                    compoundspaceRob.push_back(spaceRn[i]);
                    weightsRob.push_back(1);
                }
                //the compound state space for robot i is (SE3xRn), and either SE3 or Rn may be missing
                spaceRob[i] = ((ob::StateSpacePtr) new ob::CompoundStateSpace(compoundspaceRob,weightsRob));
                weights[i] = 1;
                sstm.str("");
                sstm << "ssRobot" << i;
                spaceRob[i]->setName(sstm.str());
            }
            //the state space for the set of robots. All the robots have the same weight.
            space = ((ob::StateSpacePtr) new ob::CompoundStateSpace(spaceRob,weights));
            ss = (oc::SimpleSetupPtr)ssptr;


        }
        else {
            ss = (oc::SimpleSetupPtr)ssptr;
            space = ss->getStateSpace();
        }
        space->setup();

        //The classes derived from this omplplanner class will create a planner,
        //the simplesetup and call the setStateValididyChecker function
    }

	//! void destructor
    omplcPlanner::~omplcPlanner(){
			
	}
	
    //! This function is used to verify that the low bound is below the high bound
    void omplcPlanner::filterBounds(double &l, double &h, double epsilon)
    {
        if((h - l) < epsilon) h = l + epsilon;
    }

	//! setParameters sets the parameters of the planner
    bool omplcPlanner::setParameters(){
      try{
        HASH_S_K::iterator it = _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

        it = _parameters.find("Max Planning Time");
        if(it != _parameters.end())
            _planningTime = it->second;
        else
          return false;

        it = _parameters.find("Incremental (0/1)");
        if (it != _parameters.end()) {
            _incremental = (it->second == 1);
        }
        else
            return false;

      }catch(...){
        return false;
      }
      return true;
    }


    //! This function sets the SoSeparator to draw a 2D configuration space
    SoSeparator *omplcPlanner::getIvCspaceScene()
    {
        //if(_wkSpace->getNumRobControls()<=3)
        //{
            _sceneCspace = new SoSeparator();
            _sceneCspace->ref();
        //}
        //else _sceneCspace=NULL;
        return Planner::getIvCspaceScene();
    }



    //! This routine allows to draw the roadmap or tree for a sigle robot with 2 dof
    void omplcPlanner::drawCspace()
    {
        if(_sceneCspace==NULL) return;

        //if(_wkSpace->getNumRobControls()<=3)
        //{
            if(_wkSpace->getRobot(0)->isSE3Enabled())
                drawCspaceSE3();
            else
                drawCspaceRn();
        //}
    }

    //! This routine allows to draw the roadmap or tree for a sigle robot with 2 translational dof
    void omplcPlanner::drawCspaceSE3()
    {
            //first delete whatever is already drawn
            while (_sceneCspace->getNumChildren() > 0)
            {
                _sceneCspace->removeChild(0);
            }

            //to draw points
            SoSeparator *psep = new SoSeparator();
            SoCoordinate3 *points  = new SoCoordinate3();
            SoPointSet *pset  = new SoPointSet();

            ob::ScopedState<ob::CompoundStateSpace> pathscopedstate(space);

            //get the SE3 subspace
            ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
            ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));

            //space bounds
            KthReal xmin=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().low[0];
            KthReal xmax=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().high[0];
            KthReal ymin=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().low[1];
            KthReal ymax=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().high[1];
            KthReal zmin=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().low[2];
            KthReal zmax=ssRobotiSE3->as<ob::SE3StateSpace>()->getBounds().high[2];

            KthReal x,y,z;
            //load the planner data to be drawn
            ob::PlannerDataPtr pdata;
            pdata = ((ob::PlannerDataPtr) new ob::PlannerData(ss->getSpaceInformation()));
            ss->getPlanner()->getPlannerData(*pdata);

            //loop for all vertices of the roadmap or tree and create the coin3D points
            for(unsigned i=0;i<pdata->numVertices();i++)
            {
                pathscopedstate = pdata->getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>();
                ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                pathscopedstate >> pathscopedstatese3;
                x = pathscopedstatese3->getX();
                y = pathscopedstatese3->getY();
                z = pathscopedstatese3->getZ();

                points->point.set1Value(i,x,y,z);
            }
            SoDrawStyle *pstyle = new SoDrawStyle;
            pstyle->pointSize = 2;
            SoMaterial *color = new SoMaterial;
            color->diffuseColor.setValue(0.2,0.8,0.2);

            //draw the points
            psep->addChild(color);
            psep->addChild(points);
            psep->addChild(pstyle);
            psep->addChild(pset);
            _sceneCspace->addChild(psep);

            //draw edges:
            SoSeparator *lsep = new SoSeparator();
            int numOutgoingEdges;
            std::vector< unsigned int > outgoingVertices;

            //loop for all nodes
            for(unsigned i=0;i<pdata->numVertices();i++)
            {
                 numOutgoingEdges = pdata->getEdges (i, outgoingVertices);

                 //for each node loop for all the outgoing edges
                 for ( int j=0; j<numOutgoingEdges; j++ ){

                    SoCoordinate3 *edgepoints  = new SoCoordinate3();

                    //initial edgepoint
                    float x1,y1,z1,x2,y2,z2;
                    pathscopedstate = pdata->getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>();
                    ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                    pathscopedstate >> pathscopedstatese3;
                    x1 = pathscopedstatese3->getX();
                    y1 = pathscopedstatese3->getY();
                    z1 = pathscopedstatese3->getZ();
                    edgepoints->point.set1Value(0,x1,y1,z1);

                    //final edgepoint
                    pathscopedstate = pdata->getVertex(outgoingVertices.at(j)).getState()->as<ob::CompoundStateSpace::StateType>();
                     pathscopedstate >> pathscopedstatese3;
                    x2 = pathscopedstatese3->getX();
                    y2 = pathscopedstatese3->getY();
                    z2 = pathscopedstatese3->getZ();
                    edgepoints->point.set1Value(1,x2,y2,z2);

                    //the edge
                    lsep->addChild(edgepoints);
                    SoLineSet *ls = new SoLineSet;
                    ls->numVertices.set1Value(0,2);//two values
                    lsep->addChild(ls);
                  }
            }
            _sceneCspace->addChild(lsep);

            //draw path:
            if(_solved)
            {
                //separator for the solution path
                SoSeparator *pathsep = new SoSeparator();
                //get the states of the solution path
                std::vector< ob::State * > & pathstates = ss->getSolutionPath().getStates();

                //loop for al the states of the solution path
                for(unsigned i=0; i<ss->getSolutionPath().getStateCount()-1; i++)
                {
                    //initial edgepoint
                    SoCoordinate3 *edgepoints  = new SoCoordinate3();
                    pathscopedstate = pathstates[i]->as<ob::CompoundStateSpace::StateType>();
                    ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                    pathscopedstate >> pathscopedstatese3;
                    x = pathscopedstatese3->getX();
                    y = pathscopedstatese3->getY();
                    z = pathscopedstatese3->getZ();
                    edgepoints->point.set1Value(0,x,y,z);

                    //final edgepoint
                    pathscopedstate = pathstates[i+1]->as<ob::CompoundStateSpace::StateType>();
                    pathscopedstate >> pathscopedstatese3;
                    x = pathscopedstatese3->getX();
                    y = pathscopedstatese3->getY();
                    z = pathscopedstatese3->getZ();
                    edgepoints->point.set1Value(1,x,y,z);

                    //edge of the path
                    pathsep->addChild(edgepoints);
                    SoLineSet *ls = new SoLineSet;
                    ls->numVertices.set1Value(0,2);//two values
                    SoDrawStyle *lstyle = new SoDrawStyle;
                    lstyle->lineWidth=2;
                    SoMaterial *path_color = new SoMaterial;
                    path_color->diffuseColor.setValue(0.8,0.2,0.2);
                    pathsep->addChild(path_color);
                    pathsep->addChild(lstyle);
                    pathsep->addChild(ls);
                }
                _sceneCspace->addChild(pathsep);
            }


            //draw floor
            SoSeparator *floorsep = new SoSeparator();
            SoCube *cs = new SoCube();
            cs->width = xmax-xmin;
            cs->height = ymax-ymin;
            if(zmax==zmin) cs->depth = (xmax-xmin)/50.0;
            else cs->depth = zmax-zmin;

            SoTransform *cub_transf = new SoTransform;
            SbVec3f centre;
            centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2, zmin+(zmax-zmin)/2);
            cub_transf->translation.setValue(centre);
            cub_transf->recenter(centre);

            SoMaterial *cub_color = new SoMaterial;
            cub_color->transparency.setValue(0.8);
            //cub_color->diffuseColor.setValue(0.2,0.2,0.2);


            floorsep->addChild(cub_color);
            floorsep->addChild(cub_transf);
            floorsep->addChild(cs);
            _sceneCspace->addChild(floorsep);
    }


    //! This routine allows to draw the roadmap or tree for a single robot with 2 joints
    void omplcPlanner::drawCspaceRn()
    {
        //first delete whatever is already drawn
        while (_sceneCspace->getNumChildren() > 0)
        {
            _sceneCspace->removeChild(0);
        }

        //to draw points
        SoSeparator *psep = new SoSeparator();
        SoCoordinate3 *points  = new SoCoordinate3();
        SoPointSet *pset  = new SoPointSet();

        ob::ScopedState<ob::CompoundStateSpace> pathscopedstate(space);

        //get the SE3 subspace
        ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
        ob::StateSpacePtr ssRobotiRn =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));

        //space bounds
        KthReal xmin=ssRobotiRn->as<omplplanner::weigthedRealVectorStateSpace>()->getBounds().low[0];
        KthReal xmax=ssRobotiRn->as<omplplanner::weigthedRealVectorStateSpace>()->getBounds().high[0];
        KthReal ymin=ssRobotiRn->as<omplplanner::weigthedRealVectorStateSpace>()->getBounds().low[1];
        KthReal ymax=ssRobotiRn->as<omplplanner::weigthedRealVectorStateSpace>()->getBounds().high[1];

        KthReal x,y;
        //load the planner data to be drawn
        ob::PlannerDataPtr pdata;
        pdata = ((ob::PlannerDataPtr) new ob::PlannerData(ss->getSpaceInformation()));
        ss->getPlanner()->getPlannerData(*pdata);

        //loop for all vertices of the roadmap or tree and create the coin3D points
        for(unsigned i=0;i<pdata->numVertices();i++)
        {
            pathscopedstate = pdata->getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>();
            ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> pathscopedstatern(ssRobotiRn);
            pathscopedstate >> pathscopedstatern;
            x = pathscopedstatern->values[0];
            y = pathscopedstatern->values[1];

            points->point.set1Value(i,x,y,0);
        }
        SoDrawStyle *pstyle = new SoDrawStyle;
        pstyle->pointSize = 2;
        SoMaterial *color = new SoMaterial;
        color->diffuseColor.setValue(0.2,0.8,0.2);

        //draw the points
        psep->addChild(color);
        psep->addChild(points);
        psep->addChild(pstyle);
        psep->addChild(pset);
        _sceneCspace->addChild(psep);

        //draw edges:
        SoSeparator *lsep = new SoSeparator();
        int numOutgoingEdges;
        std::vector< unsigned int > outgoingVertices;

        //loop for all nodes
        for(unsigned i=0;i<pdata->numVertices();i++)
        {
             numOutgoingEdges = pdata->getEdges (i, outgoingVertices);

             //for each node loop for all the outgoing edges
             for ( int j=0; j<numOutgoingEdges; j++ ){

                SoCoordinate3 *edgepoints  = new SoCoordinate3();

                //initial edgepoint
                float x1,y1,x2,y2,z;
                pathscopedstate = pdata->getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>();
                ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> pathscopedstatern(ssRobotiRn);
                pathscopedstate >> pathscopedstatern;
                x1 = pathscopedstatern->values[0];
                y1 = pathscopedstatern->values[1];
                z=0.0;
                edgepoints->point.set1Value(0,x1,y1,z);

                //final edgepoint
                pathscopedstate = pdata->getVertex(outgoingVertices.at(j)).getState()->as<ob::CompoundStateSpace::StateType>();
                 pathscopedstate >> pathscopedstatern;
                 x2 = pathscopedstatern->values[0];
                 y2 = pathscopedstatern->values[1];
                edgepoints->point.set1Value(1,x2,y2,z);

                //the edge
                lsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                lsep->addChild(ls);
              }
        }
        _sceneCspace->addChild(lsep);

        //draw path:
        if(_solved)
        {
            //separator for the solution path
            SoSeparator *pathsep = new SoSeparator();
            //get the states of the solution path
            std::vector< ob::State * > & pathstates = ss->getSolutionPath().getStates();

            //loop for al the states of the solution path
            for(unsigned i=0; i<ss->getSolutionPath().getStateCount()-1; i++)
            {
                //initial edgepoint
                SoCoordinate3 *edgepoints  = new SoCoordinate3();
                pathscopedstate = pathstates[i]->as<ob::CompoundStateSpace::StateType>();
                ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> pathscopedstatern(ssRobotiRn);
                pathscopedstate >> pathscopedstatern;
                x = pathscopedstatern->values[0];
                y = pathscopedstatern->values[1];
                edgepoints->point.set1Value(0,x,y,0);

                //final edgepoint
                pathscopedstate = pathstates[i+1]->as<ob::CompoundStateSpace::StateType>();
                pathscopedstate >> pathscopedstatern;
                x = pathscopedstatern->values[0];
                y = pathscopedstatern->values[1];
                edgepoints->point.set1Value(1,x,y,0);

                //edge of the path
                pathsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                SoDrawStyle *lstyle = new SoDrawStyle;
                lstyle->lineWidth=2;
                SoMaterial *path_color = new SoMaterial;
                path_color->diffuseColor.setValue(0.8,0.2,0.2);
                pathsep->addChild(path_color);
                pathsep->addChild(lstyle);
                pathsep->addChild(ls);
            }
            _sceneCspace->addChild(pathsep);
        }

        //draw floor
        SoSeparator *floorsep = new SoSeparator();
        SoCube *cs = new SoCube();
        cs->width = xmax-xmin;
        cs->depth = (xmax-xmin)/50.0;
        cs->height = ymax-ymin;

        SoTransform *cub_transf = new SoTransform;
        SbVec3f centre;
        centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,-cs->depth.getValue());
        cub_transf->translation.setValue(centre);
        cub_transf->recenter(centre);

        SoMaterial *cub_color = new SoMaterial;
        cub_color->diffuseColor.setValue(0.2,0.2,0.2);

        floorsep->addChild(cub_color);
        floorsep->addChild(cub_transf);
        floorsep->addChild(cs);
        _sceneCspace->addChild(floorsep);
    }


    //! This function converts a Kautham sample to an ompl scoped state.
    void omplcPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate)
    {
        //Extract the mapped configuration of the sample. It is a vector with as many components as robots.
        //each component has the RobConf of the robot (the SE3 and the Rn configurations)
        if(smp->getMappedConf().size()==0)
        {
            _wkSpace->moveRobotsTo(smp); // to set the mapped configuration
        }
        std::vector<RobConf>& smpRobotsConf = smp->getMappedConf();

        //loop for all the robots
        for(unsigned i=0; i<_wkSpace->getNumRobots(); i++)
        {
            int k=0; //counter of subspaces contained in subspace of robot i

            //get the subspace of robot i
            ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(i));
            string ssRobotiname = ssRoboti->getName();

            //if it has se3 part
            if(_wkSpace->getRobot(i)->isSE3Enabled())
            {
                //get the kautham SE3 configuration
                SE3Conf c = smpRobotsConf.at(i).getSE3();
                vector<KthReal>& pp = c.getPos();
                vector<KthReal>& aa = c.getAxisAngle();

                //set the ompl SE3 configuration
                ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));
                string ssRobotiSE3name = ssRobotiSE3->getName();

                ob::ScopedState<ob::SE3StateSpace> cstart(ssRobotiSE3);
                cstart->setX(pp[0]);
                cstart->setY(pp[1]);
                cstart->setZ(pp[2]);
                cstart->rotation().setAxisAngle(aa[0],aa[1],aa[2],aa[3]);

                //load the global scoped state with the info of the se3 data of robot i
                (*sstate)<<cstart;
                k++;
            }

            //has Rn part
            if(_wkSpace->getRobot(i)->getNumJoints()>0)
            {
                //get the kautham Rn configuration
                RnConf r = smpRobotsConf.at(i).getRn();

                //set the ompl Rn configuration
                ob::StateSpacePtr ssRobotiRn =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));
                ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> rstart(ssRobotiRn);

                for(unsigned j=0; j<_wkSpace->getRobot(i)->getNumJoints();j++)
                    rstart->values[j] = r.getCoordinate(j);

                //load the global scoped state with the info of the Rn data of robot i
                (*sstate) << rstart;
                k++;//dummy
            }
        }
    }


    //! This member function converts an ompl State to a Kautham sample
    void omplcPlanner::omplState2smp(const ob::State *state, Sample* smp)
    {
        ob::ScopedState<ob::CompoundStateSpace> sstate(space);
        sstate = *state;
        omplScopedState2smp( sstate, smp);
    }

    //! This member function converts an ompl ScopedState to a Kautham sample
    void omplcPlanner::omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp)
    {
        vector<RobConf> rc;

        //loop for all the robots
        for(unsigned i=0; i<_wkSpace->getNumRobots(); i++)
        {
            //RobConf to store the robots configurations read form the ompl state
            RobConf *rcj = new RobConf;

            //Get the subspace corresponding to robot i
            ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(i));

            //Get the SE3 subspace of robot i, if it exisits, and extract the SE3 configuration
            int k=0; //counter of subspaces of robot i
            if(_wkSpace->getRobot(i)->isSE3Enabled())
            {
                //Get the SE3 subspace of robot i
                 ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));

                 //create a SE3 scoped state and load it with the data extracted from the global scoped state
                 ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                 sstate >> pathscopedstatese3;

                 //convert to a vector of 7 components
                 vector<KthReal> se3coords;
                 se3coords.resize(7);
                 se3coords[0] = pathscopedstatese3->getX();
                 se3coords[1] = pathscopedstatese3->getY();
                 se3coords[2] = pathscopedstatese3->getZ();
                 se3coords[3] = pathscopedstatese3->rotation().x;
                 se3coords[4] = pathscopedstatese3->rotation().y;
                 se3coords[5] = pathscopedstatese3->rotation().z;
                 se3coords[6] = pathscopedstatese3->rotation().w;
                 //create the sample
                 SE3Conf se3;
                 se3.setCoordinates(se3coords);
                 rcj->setSE3(se3);
                 k++;
             }
             //If the robot does not have movile SE3 dofs then the SE3 configuration of the sample is maintained
             else
             {
                 if(smp->getMappedConf().size()==0)
                     throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                 else
                     rcj->setSE3(smp->getMappedConf()[i].getSE3());
             }


            //Get the Rn subspace of robot i, if it exisits, and extract the Rn configuration
             if(_wkSpace->getRobot(i)->getNumJoints()>0)
             {
                 //Get the Rn subspace of robot i
                 ob::StateSpacePtr ssRobotiRn =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));

                 //create a Rn scoped state and load it with the data extracted from the global scoped state
                 ob::ScopedState<omplplanner::weigthedRealVectorStateSpace> pathscopedstateRn(ssRobotiRn);
                 sstate >> pathscopedstateRn;

                 //convert to a vector of n components
                 vector<KthReal> coords;
                 for(unsigned j=0;j<_wkSpace->getRobot(i)->getNumJoints();j++) coords.push_back(pathscopedstateRn->values[j]);
                 rcj->setRn(coords);
                 k++;//dummy
             }
             //If the robot does not have movile Rn dofs then the Rn configuration of the sample is maintained
             else
             {
                 if(smp->getMappedConf().size()==0)
                     throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                 else
                     rcj->setRn(smp->getMappedConf()[i].getRn());
             }
             //load the RobConf with the data of robot i
             rc.push_back(*rcj);
        }
        //create the sample with the RobConf
        //the coords (controls) of the sample are kept void
        smp->setMappedConf(rc);
    }

	//! function to find a solution path
    bool omplcPlanner::trySolve()
    {
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
        goalStates->setThreshold(0.01);
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

        //Remove previous solutions, if any
        if (_incremental) {
            ss->getProblemDefinition()->clearSolutionPaths();
        } else {
            ss->clear();
            ss->getPlanner()->clear();
        }
        //ss->clear();//to remove previous solutions, if any
        //ss->getPlanner()->clear();

        // attempt to solve the problem within _planningTime seconds of planning time
        ob::PlannerStatus solved = ss->solve(_planningTime);
        //UNKNOWN = 0,
        //INVALID_START,
        //INVALID_GOAL,
        //UNRECOGNIZED_GOAL_TYPE,
        //TIMEOUT,
        //APPROXIMATE_SOLUTION,
        //EXACT_SOLUTION,
        //CRASH,
        //TYPE_COUNT

         ss->print();

        //retrieve all the states. Load the SampleSet _samples
        ob::PlannerData data(ss->getSpaceInformation());
        ss->getPlannerData(data);
        /*
        int imax = data.numVertices();
        for(int i=0; i<imax;i++)
        {
             smp=new Sample(_wkSpace->getNumRobControls());
             smp->setMappedConf(_init->getMappedConf());//copy the conf of the start smp
             omplState2smp(data.getVertex(i).getState(), smp);
             _samples->add(smp);
        }
        */

        if (solved)
        {
             std::cout << "Found solution (solved=<<"<<solved.asString()<<"):" << std::endl;
             // print the path to screen
             std::cout << "Geomeric Path solution:" << std::endl;
             ss->getSolutionPath().asGeometric().print(std::cout);
             std::cout << "Control Path solution:" << std::endl;
             ss->getSolutionPath().print(std::cout);

             Sample *smp;
             _path.clear();
             clearSimulationPath();
             int l = ss->getSolutionPath().asGeometric().getStateCount();

             //load the kautham _path variable from the ompl solution
             for(int j=0;j<l;j++){
                 //create a smp and load the RobConf of the init configuration (to have the same if the state does not change it)
                 smp=new Sample(_wkSpace->getNumRobControls());
                 smp->setMappedConf(_init.at(0)->getMappedConf());
                 //convert form state to smp
                 omplState2smp(ss->getSolutionPath().asGeometric().getState(j)->as<ob::CompoundStateSpace::StateType>(), smp);

                 _path.push_back(smp);
                 _samples->add(smp);
               }
               _solved = true;
               drawCspace();
               return _solved;
            }
            else{
                std::cout << "No solution found (solved=<<"<<solved.asString()<<")" << std::endl;
                _solved = false;
                drawCspace();
                return _solved;
            }
		}
    }
}

#endif // KAUTHAM_USE_OMPL


