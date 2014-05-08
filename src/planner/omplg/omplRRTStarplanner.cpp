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
#include <problem/workspace.h>
#include <sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "omplRRTStarplanner.h"
#include "omplValidityChecker.h"


#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include "omplPCAalignmentOptimizationObjective.h"

//for myRRTstar
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <boost/math/constants/constants.hpp>
//#include <ompl/geometric/planners/PlannerIncludes.h>
//#include <ompl/datastructures/NearestNeighbors.h>
//#include <limits>
//#include <vector>
//#include <utility>

namespace Kautham {
  namespace omplplanner{

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  class myRRTstar:public og::RRTstar
  {
  private:
      bool _optimize; //!< flag that allows to disable optimization when set to false. Defaults to true.
      double _factor_k_rrg;//!< Value that modifies the number K of nearest neighbors (multiplication factor)

  public:

      myRRTstar(const ob::SpaceInformationPtr &si):RRTstar(si)
      {
          setName("myRRTstar");
          _optimize = true;
          _factor_k_rrg = 1.0;
      }

      bool getOptimize(){ return _optimize;}; //!< Returns the _optimize flag
      void setOptimize(bool s){_optimize=s;}; //!< Sets the _optimize flag
      bool getNeighFactor(){ return _factor_k_rrg;}; //!< Returns _factor_k_rrg that modifies the number K of nearest neighbors
      void setNeighFactor(double f){_factor_k_rrg=f;}; //!< Sets  _factor_k_rrg to modify the number K of nearest neighbors


      /** \brief Compute distance between motions (actually distance between contained states) */
      double distanceFunction(const Motion* a, const Motion* b) const
      {
          return si_->distance(a->state, b->state);
          //return (opt_->motionCost(a->state, b->state).v);
      }



      ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc)
      {
          checkValidity();
          ob::Goal                  *goal   = pdef_->getGoal().get();
          ob::GoalSampleableRegion  *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

          bool symDist = si_->getStateSpace()->hasSymmetricDistance();
          bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
          bool symCost = opt_->isSymmetric();

          while (const ob::State *st = pis_.nextStart())
          {
              Motion *motion = new Motion(si_);
              si_->copyState(motion->state, st);
              motion->cost = opt_->identityCost();
              nn_->add(motion);
          }

          if (nn_->size() == 0)
          {
              OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
              return ob::PlannerStatus::INVALID_START;
          }


          if (!sampler_)
              sampler_ = si_->allocStateSampler();

          OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

          Motion *solution       = lastGoalMotion_;

          // \TODO Make this variable unnecessary, or at least have it
          // persist across solve runs
          ob::Cost bestCost    = opt_->infiniteCost();

          Motion *approximation  = NULL;
          double approximatedist = std::numeric_limits<double>::infinity();
          bool sufficientlyShort = false;

          Motion *rmotion        = new Motion(si_);
          ob::State *rstate    = rmotion->state;
          ob::State *xstate    = si_->allocState();

          // e+e/d.  K-nearest RRT*
          double k_rrg;
          if(_optimize){
              k_rrg = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());
              k_rrg = _factor_k_rrg*k_rrg;
          }
          else
              k_rrg = 0.0; //When set to zero RRTStar behavies as a standard RRT

          std::vector<Motion*>       nbh;

          std::vector<ob::Cost>    costs;
          std::vector<ob::Cost>    incCosts;
          std::vector<std::size_t>   sortedCostIndices;

          std::vector<int>           valid;
          unsigned int               rewireTest = 0;
          unsigned int               statesGenerated = 0;

          if (solution)
              OMPL_INFORM("%s: Starting with existing solution of cost %.5f", getName().c_str(), solution->cost.v);
          OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size()+1))));


          // our functor for sorting nearest neighbors
          CostIndexCompare compareFn(costs, *opt_);

          while (ptc == false)
          {
              iterations_++;
              // sample random state (with goal biasing)
              // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
              if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
                  goal_s->sampleGoal(rstate);
              else
                  sampler_->sampleUniform(rstate);

              // find closest state in the tree
              Motion *nmotion = nn_->nearest(rmotion);

              ob::State *dstate = rstate;

              // find state to add to the tree
              double d = si_->distance(nmotion->state, rstate);
              if (d > maxDistance_)
              {
                  si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
                  dstate = xstate;
              }

              // Check if the motion between the nearest state and the state to add is valid
              ++collisionChecks_;
              if (si_->checkMotion(nmotion->state, dstate))
              {
                  // create a motion
                  Motion *motion = new Motion(si_);
                  si_->copyState(motion->state, dstate);
                  motion->parent = nmotion;

                  // This sounds crazy but for asymmetric distance functions this is necessary
                  // For this case, it has to be FROM every other point TO our new point
                  // NOTE THE ORDER OF THE boost::bind PARAMETERS
                  if (!symDist)
                      nn_->setDistanceFunction(boost::bind(&myRRTstar::distanceFunction, this, _1, _2));

                  // Find nearby neighbors of the new motion - k-nearest RRT*
                  unsigned int k = std::ceil(k_rrg * log((double)(nn_->size()+1)));
                  nn_->nearestK(motion, k, nbh);
                  rewireTest += nbh.size();
                  statesGenerated++;

                  // cache for distance computations
                  //
                  // Our cost caches only increase in size, so they're only
                  // resized if they can't fit the current neighborhood
                  if (costs.size() < nbh.size())
                  {
                      costs.resize(nbh.size());
                      incCosts.resize(nbh.size());
                      sortedCostIndices.resize(nbh.size());
                  }

                  // cache for motion validity (only useful in a symmetric space)
                  //
                  // Our validity caches only increase in size, so they're
                  // only resized if they can't fit the current neighborhood
                  if (symDist && symInterp)
                  {
                      if (valid.size() < nbh.size())
                          valid.resize(nbh.size());
                      std::fill(valid.begin(), valid.begin()+nbh.size(), 0);
                  }


                  // Finding the nearest neighbor to connect to
                  // By default, neighborhood states are sorted by cost, and collision checking
                  // is performed in increasing order of cost
                  if (delayCC_)
                  {
                      // calculate all costs and distances
                      for (std::size_t i = 0 ; i < nbh.size(); ++i)
                      {

                          if(opt_->getDescription()=="PMD alignment"){
                              ob::State *s0;
                              if(nbh[i]->parent==NULL) s0=NULL;
                              else s0=nbh[i]->parent->state;
                              incCosts[i] = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nbh[i]->state, motion->state);
                          }
                          else
                              incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                          costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                      }

                      // sort the nodes
                      //
                      // we're using index-value pairs so that we can get at
                      // original, unsorted indices
                      for (std::size_t i = 0; i < nbh.size(); ++i)
                          sortedCostIndices[i] = i;
                      std::sort(sortedCostIndices.begin(), sortedCostIndices.begin()+nbh.size(),
                                compareFn);

                      // collision check until a valid motion is found
                      for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                           i != sortedCostIndices.begin()+nbh.size();
                           ++i)
                      {
                          if (nbh[*i] != nmotion)
                              ++collisionChecks_;

                          if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
                          {
                              motion->incCost = incCosts[*i];
                              motion->cost = costs[*i];
                              motion->parent = nbh[*i];
                              if (symDist && symInterp)
                                  valid[*i] = 1;
                              break;
                          }
                          else if (symDist && symInterp)
                              valid[*i] = -1;
                      }
                  }
                  else // if not delayCC
                  {

                      if(opt_->getDescription()=="PMD alignment"){
                          ob::State *s0;
                          if(nmotion->parent==NULL) s0=NULL;
                          else s0=nmotion->parent->state;
                          motion->incCost = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nmotion->state, motion->state);
                      }
                      else
                          motion->incCost = opt_->motionCost(nmotion->state, motion->state);

                      motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                      // find which one we connect the new state to
                      for (std::size_t i = 0 ; i < nbh.size(); ++i)
                      {
                          if (nbh[i] != nmotion)
                          {

                              if(opt_->getDescription()=="PMD alignment"){
                                  ob::State *s0;
                                  if(nbh[i]->parent==NULL) s0=NULL;
                                  else s0=nbh[i]->parent->state;
                                  incCosts[i] = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,nbh[i]->state, motion->state);
                              }
                              else
                                  incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);

                              costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                              if (opt_->isCostBetterThan(costs[i], motion->cost))
                              {
                                  ++collisionChecks_;
                                  if (si_->checkMotion(nbh[i]->state, motion->state))
                                  {
                                      motion->incCost = incCosts[i];
                                      motion->cost = costs[i];
                                      motion->parent = nbh[i];
                                      if (symDist && symInterp)
                                          valid[i] = 1;
                                  }
                                  else if (symDist && symInterp)
                                      valid[i] = -1;
                              }
                          }
                          else
                          {
                              incCosts[i] = motion->incCost;
                              costs[i] = motion->cost;
                              if (symDist && symInterp)
                                  valid[i] = 1;
                          }
                      }
                  }

                  // add motion to the tree
                  nn_->add(motion);
                  motion->parent->children.push_back(motion);

                  bool checkForSolution = false;
                  // rewire tree if needed
                  //
                  // This sounds crazy but for asymmetric distance functions this is necessary
                  // For this case, it has to be FROM our new point TO each other point
                  // NOTE THE ORDER OF THE boost::bind PARAMETERS
                  if (!symDist)
                  {
                      nn_->setDistanceFunction(boost::bind(&myRRTstar::distanceFunction, this, _2, _1));
                      nn_->nearestK(motion, k, nbh);
                      rewireTest += nbh.size();
                  }

                  for (std::size_t i = 0; i < nbh.size(); ++i)
                  {
                      if (nbh[i] != motion->parent)
                      {
                          ob::Cost nbhIncCost;
                          if (symDist && symCost)
                              nbhIncCost = incCosts[i];
                          else{
                              if(opt_->getDescription()=="PMD alignment"){
                                  ob::State *s0;
                                  if(motion->parent==NULL) s0=NULL;
                                  else s0=motion->parent->state;
                                  nbhIncCost = ((PMDalignmentOptimizationObjective*)opt_.get())->motionCost(s0,motion->state, nbh[i]->state);
                              }
                              else
                                  nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                          }

                          ob::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                          if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                          {
                              bool motionValid;
                              if (symDist && symInterp)
                              {
                                  if (valid[i] == 0)
                                  {
                                      ++collisionChecks_;
                                      motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                                  }
                                  else
                                      motionValid = (valid[i] == 1);
                              }
                              else
                              {
                                  ++collisionChecks_;
                                  motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                              }
                              if (motionValid)
                              {
                                  // Remove this node from its parent list
                                  removeFromParent (nbh[i]);

                                  // Add this node to the new parent
                                  nbh[i]->parent = motion;
                                  nbh[i]->incCost = nbhIncCost;
                                  nbh[i]->cost = nbhNewCost;
                                  nbh[i]->parent->children.push_back(nbh[i]);

                                  // Update the costs of the node's children
                                  updateChildCosts(nbh[i]);

                                  checkForSolution = true;
                              }
                          }
                      }
                  }

                  // Add the new motion to the goalMotion_ list, if it satisfies the goal
                  double distanceFromGoal;
                  if (goal->isSatisfied(motion->state, &distanceFromGoal))
                  {
                      goalMotions_.push_back(motion);
                      checkForSolution = true;
                  }

                  // Checking for solution or iterative improvement
                  if (checkForSolution)
                  {
                      for (size_t i = 0; i < goalMotions_.size(); ++i)
                      {
                          if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                          {
                              bestCost = goalMotions_[i]->cost;
                              bestCost_ = bestCost;
                          }

                          sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
                          if (sufficientlyShort)
                          {
                              solution = goalMotions_[i];
                              break;
                          }
                          else if (!solution ||
                                   opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
                              solution = goalMotions_[i];
                      }
                  }

                  // Checking for approximate solution (closest state found to the goal)
                  if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
                  {
                      approximation = motion;
                      approximatedist = distanceFromGoal;
                  }
              }

              // terminate if a sufficient solution is found
              if (solution && sufficientlyShort)
                  break;
          }

          bool approximate = (solution == 0);
          bool addedSolution = false;
          if (approximate)
              solution = approximation;
          else
              lastGoalMotion_ = solution;

          if (solution != 0)
          {
              // construct the solution path
              std::vector<Motion*> mpath;
              while (solution != 0)
              {
                  mpath.push_back(solution);
                  solution = solution->parent;
              }

              // set the solution path
              og::PathGeometric *geoPath = new og::PathGeometric(si_);
              for (int i = mpath.size() - 1 ; i >= 0 ; --i)
                  geoPath->append(mpath[i]->state);

              ob::PathPtr path(geoPath);
              // Add the solution path, whether it is approximate (not reaching the goal), and the
              // distance from the end of the path to the goal (-1 if satisfying the goal).
              ob::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0);
              // Does the solution satisfy the optimization objective?
              psol.optimized_ = sufficientlyShort;

              pdef_->addSolutionPath (psol);

              addedSolution = true;
          }

          si_->freeState(xstate);
          if (rmotion->state)
              si_->freeState(rmotion->state);
          delete rmotion;

          OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

          return ob::PlannerStatus(addedSolution, approximate);
      }

  };




  /*!
   * createOptimizationObjectivePMD creates a PMD alignment Optimization Objective.
   * If there is only a single robot, it is basicly corresponds to the PMDs of the hand synergies.
   * If there are more than one robot it corresponds to the coupling in the x,y,z coordinates of the base.
   * \return the pointer to the OptimizationObjective
   */
  ob::OptimizationObjectivePtr omplRRTStarPlanner::createOptimizationObjectivePMD()
  {
      if(wkSpace()->getNumRobots() == 1)
      {
          //////////////////////////////////////////////////////////////////////////////
          // 3) single robot handPMD alignment optimization criteria
          int robotindex = 0;
          int numPMD = 0;

          string listcontrolsname = wkSpace()->getRobControlsName();
          vector<string*> controlname;
          string *newcontrol = new string;
          for(int i=0; i<listcontrolsname.length();i++)
          {
              if(listcontrolsname[i]=='|')
              {
                  controlname.push_back(newcontrol);
                  newcontrol = new string;
              }
              else
                  newcontrol->push_back(listcontrolsname[i]);
          }
          //add last control (since listcontrolsname does not end with a |)
          controlname.push_back(newcontrol);

          std::size_t foundlabel;
          vector<int> handpmdcolumns;
          for(int i=0;i<controlname.size();i++)
          {
              if(controlname[i]->find("PMD") != string::npos)
              {
                  //store the columns of the matrix of controls that will contain the handpmd control
                  handpmdcolumns.push_back(i);
              }
          }
          numPMD = handpmdcolumns.size();

          //be careful, if no PMD control is provides return the distance optimization function
          if(numPMD==0)
          {
              cout<<"Error configuring the PMD matrix. No PMD controls provided. Returning distance optimization objective)"<<endl;
              return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
          }


          int numDOF = wkSpace()->getRobot(robotindex)->getNumJoints();
          ob::ProjectionMatrix PMD;
          PMD.mat = ob::ProjectionMatrix::Matrix(numDOF,numPMD);
          KthReal **mm2 = wkSpace()->getRobot(robotindex)->getMapMatrix();
          double kk;
          int col;
          for(int j=0;j<numPMD;j++)//columns
          {
              for(int i=0;i<numDOF;i++)//rows
              {
                  col = handpmdcolumns[j];
                  kk = mm2[6+i][handpmdcolumns[j]];
                  PMD.mat(i,j) = mm2[6+i][handpmdcolumns[j]];
              }
          }

          ob::OptimizationObjectivePtr _handpmdalignmentopti;
          _handpmdalignmentopti = ob::OptimizationObjectivePtr(new singleRobotPMDalignmentOptimizationObjective(robotindex, ss->getSpaceInformation(),PMD));
          _handpmdalignmentopti->setCostThreshold(ob::Cost(0.0));
          return _handpmdalignmentopti;
      }

      else
      {
          //////////////////////////////////////////////////////////////////////////////
          // 4) multi robot se3PMD alignment optimization criteria
          int numPMD = 0;
          int numDOF = 3*wkSpace()->getNumRobots(); //the coupling is allowed with the x,y,z of each robot

          std::map< string, vector< pair<int,int> >  > pmdMap;
          std::map< string, vector< pair<int,int> >  >::iterator itpmdMap;


          for(int k=0;k<wkSpace()->getNumRobots(); k++)
          {
              //find the controls that are coupled, i.e. those that have the PMD letters in their name
              string listcontrolsname = wkSpace()->getRobControlsName();
              vector<string*> controlname;
              string *newcontrol = new string;
              //split the list of controls to obtain the control names
              for(int i=0; i<listcontrolsname.length();i++)
              {
                  if(listcontrolsname[i]=='|')
                  {
                      controlname.push_back(newcontrol);
                      newcontrol = new string;
                  }
                  else
                      newcontrol->push_back(listcontrolsname[i]);
              }
              //add last control (since listcontrolsname does not end with a |)
              controlname.push_back(newcontrol);

              //verify those control names that have the leters PMD in their name
              std::size_t foundlabel;
              for(int i=0;i<controlname.size();i++)
              {
                  if(controlname[i]->find("PMD") != string::npos)
                  {
                      //if the pmd has not yet been found (the Key does not exists), create it
                      itpmdMap=pmdMap.find(*controlname[i]);
                      if(itpmdMap == pmdMap.end())//not found
                      {
                          pair< string, vector< pair<int,int> > > pmdfound;
                          pmdfound.first = *controlname[i]; //string

                          vector< pair<int,int> > *pmdcolumns = new vector< pair<int,int> >;
                          pmdfound.second = *pmdcolumns; //indices

                          pmdMap.insert(pmdfound);
                      }

                      //add the pair indices to the map
                      pair<int,int> pmdfoundindex;
                      pmdfoundindex.first = k; //robot index
                      pmdfoundindex.second = i; //pmd index
                      pmdMap[*controlname[i]].push_back( pmdfoundindex );
                  }
              }
          }
          numPMD = pmdMap.size();
          //check correctness
          for (itpmdMap=pmdMap.begin(); itpmdMap!=pmdMap.end(); ++itpmdMap)
          {
              if(itpmdMap->second.size() != wkSpace()->getNumRobots())
              {
                  cout<<"Error configuring the PMD matrix. The same number of coupled controls (PMDs) is required per robot (and with the same name!)"<<endl;
                  return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
              }
          }
          //be careful, if no PMD control is provides return the distance optimization function
          if(numPMD==0)
          {
              cout<<"Error configuring the PMD matrix. No PMD controls provided. Returning distance optimization objective)"<<endl;
              return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
          }

          //Now load the PMD matrix
          ob::ProjectionMatrix PMD;
          PMD.mat = ob::ProjectionMatrix::Matrix(numDOF,numPMD);

          int ic=0; //column counter
          for (itpmdMap=pmdMap.begin(); itpmdMap!=pmdMap.end(); ++itpmdMap)//columns
          {
              int ir=0; //row counter
              for(int j=0; j< itpmdMap->second.size(); j++) //this loops as many times as robots
              {
                  int robotindex = itpmdMap->second[j].first; //the robot
                  int pmdcolumn = itpmdMap->second[j].second; //the columns in the mapMatrix of robot robotindex
                  KthReal **mm2 = wkSpace()->getRobot(robotindex)->getMapMatrix(); //the mapMatrix of robot robotindex

                  //copy the first three rows of the mapMatrix of robot robotindex, that correspond to the x,y and z coordinates
                  for(int ii=0; ii< 3; ii++)
                  {
                      double kk = mm2[ii][pmdcolumn];
                      PMD.mat(ir,ic) = mm2[ii][pmdcolumn];
                      ir++;//next row in PMD matrix
                  }
              }
              ic++;
          }

          ob::OptimizationObjectivePtr _multise3pmdalignmentopti;
          _multise3pmdalignmentopti = ob::OptimizationObjectivePtr(new multiRobotSE3PMDalignmentOptimizationObjective(ss->getSpaceInformation(),PMD));
          _multise3pmdalignmentopti->setCostThreshold(ob::Cost(0.0));
          return _multise3pmdalignmentopti;
      }
  }



    //! Constructor
    omplRRTStarPlanner::omplRRTStarPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl RRT Star Planner";
        _idName = "omplRRTStar";


        //alloc valid state sampler
        si->setValidStateSamplerAllocator(boost::bind(&omplplanner::allocValidStateSampler, _1, (Planner*)this));
        //alloc state sampler
        space->setStateSamplerAllocator(boost::bind(&omplplanner::allocStateSampler, _1, (Planner*)this));

        //create planner
        ob::PlannerPtr planner(new myRRTstar(si));
        //ob::PlannerPtr planner(new og::RRTstar(si));

        //set planner parameters: range, goalbias, delay collision checking and optimization option
        _Range = 0.05;
        _GoalBias = (planner->as<og::RRTstar>())->getGoalBias();
        _DelayCC = (planner->as<og::RRTstar>())->getDelayCC();
        _opti = 0; //optimize path lenght by default
        addParameter("Range", _Range);
        addParameter("Goal Bias", _GoalBias);
        addParameter("DelayCC (0/1)", _DelayCC);
        addParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)", _opti);
        planner->as<og::RRTstar>()->setRange(_Range);
        planner->as<og::RRTstar>()->setGoalBias(_GoalBias);
        planner->as<og::RRTstar>()->setDelayCC(_DelayCC);


        //set the parameter that modifies the number K of near neighbors to search in the RRT* solve
        _KneighFactor = 1.0;
        addParameter("K-Neigh Factor", _KneighFactor);
        planner->as<myRRTstar>()->setNeighFactor(_KneighFactor);

        //////////////////////////////////////////////////////////////////////////////
        //START optimization criteria:
        ob::ProblemDefinitionPtr pdefPtr = ((ob::ProblemDefinitionPtr) new ob::ProblemDefinition(si));

        //////////////////////////////////////////////////////////////////////////////
        // 1) Length optimization criteria
        _lengthopti = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));

        //////////////////////////////////////////////////////////////////////////////
        // 2) clearance optimization criteria
        _clearanceopti = ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(ss->getSpaceInformation()));

        //////////////////////////////////////////////////////////////////////////////
        // 3) pmd alignment optimization criteria
        _pmdalignmentopti = createOptimizationObjectivePMD();

        //////////////////////////////////////////////////////////////////////////////
        _lengthweight = 0.1;
        addParameter("lengthweight(0..1)", _lengthweight);
        _penalizationweight = 1.0;
        addParameter("penalizationweight", _penalizationweight);
        _orientationweight = 1.0;
        addParameter("orientationweight", _orientationweight);


        //_multiopti = ob::OptimizationObjectivePtr(new ob::MultiOptimizationObjective(ss->getSpaceInformation()));
        //((ob::MultiOptimizationObjective*)_multiopti.get())->addObjective(_lengthopti,_lengthweight);
        //((ob::MultiOptimizationObjective*)_multiopti.get())->addObjective(_pcaalignmentopti,1.0-_lengthweight);
        //_pcaalignmentopti2 = ob::OptimizationObjectivePtr(new PCAalignmentOptimizationObjective2(ss->getSpaceInformation(),dimpca));
        //_pcaalignmentopti3 = ob::OptimizationObjectivePtr(new PCAalignmentOptimizationObjective3(ss->getSpaceInformation(),dimpca));



        if(_opti==0){
            _optiselected = _lengthopti; //dummy
            planner->as<myRRTstar>()->setOptimize(false); //Disable optimization
        }
        else if(_opti==1){
            _optiselected = _lengthopti; //length optimization
            planner->as<myRRTstar>()->setOptimize(true);
        }
        else if(_opti==2){
            _optiselected = _clearanceopti;//clearnace optimization
            planner->as<myRRTstar>()->setOptimize(true);
        }
        else if(_opti==3){
            _optiselected = _pmdalignmentopti; //pmd alignment optimization
            planner->as<myRRTstar>()->setOptimize(true);
        }
        else { //default
            _opti=1;
            _optiselected = _lengthopti; //length optimization
            setParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)", _simplify);
            planner->as<myRRTstar>()->setOptimize(true);
        }

        pdefPtr->setOptimizationObjective(_optiselected);


        planner->setProblemDefinition(pdefPtr);
        planner->setup();

        //set the planner
        ss->setPlanner(planner);

        //for the RRT type of planners we do not want to constrain the samppling in the PMD subspace, then disable these controls
        _disablePMDControlsFromSampling=1;
        addParameter("disablePMDControlsFromSampling", _disablePMDControlsFromSampling);
        disablePMDControlsFromSampling();
    }

	//! void destructor
    omplRRTStarPlanner::~omplRRTStarPlanner(){
			
	}
	

    //! function to find a solution path
    bool omplRRTStarPlanner::trySolve()
    {
        bool ret;
        //solve
        ret = omplPlanner::trySolve();
        //evaluate path
        ob::Cost pathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
        cout<<"Path cost = "<<pathcost.v<<endl;
        if(_opti==3)
        {
            //store the values
            double ow = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getOrientationWeight();
            double dw = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getDistanceWeight();
            double pw = ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->getOrientationPenalization();
            //eval only the alignment with the PMDs
            _orientationweight = 1.0;
            _lengthweight = 0.0;
            _penalizationweight = 0.0;
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);

            ob::Cost alignmentpathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
            cout<<"Path alignment cost = "<<alignmentpathcost.v<<endl;

            //eval only the distance
            _orientationweight = 0.0;
            _lengthweight = 1.0;
            _penalizationweight = 0.0;
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);
            ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);

             ob::Cost distancepathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(_optiselected);
             cout<<"Path distance cost = "<<distancepathcost.v<<endl;

             cout<<"Alignment cost per unitary distance traveled = "<< (alignmentpathcost.v/distancepathcost.v)<<endl;

             //restore the values
             ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(ow);
             ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(dw);
             ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(pw);
        }

        return ret;

     }

	//! setParameters sets the parameters of the planner
    bool omplRRTStarPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("Range");
        if(it != _parameters.end()){
          _Range = it->second;
          ss->getPlanner()->as<og::RRTstar>()->setRange(_Range);
         }
        else
          return false;

        it = _parameters.find("Optimize none(0)/dist(1)/clear(2)/PMD(3)");
        if(it != _parameters.end()){
            _opti = it->second;
            ob::ProblemDefinitionPtr pdefPtr = ss->getPlanner()->getProblemDefinition();

            if(_opti==0){
                _optiselected = _lengthopti; //dummy
                ss->getPlanner()->as<myRRTstar>()->setOptimize(false); //Disable optimization
            }
            else if(_opti==1){
                _optiselected = _lengthopti; //length optimization
                ss->getPlanner()->as<myRRTstar>()->setOptimize(true);
            }
            else if(_opti==2){
                _optiselected = _clearanceopti;//clearnace optimization
                ss->getPlanner()->as<myRRTstar>()->setOptimize(true);
            }
            else if(_opti==3){
                _optiselected = _pmdalignmentopti; //pmd alignment optimization
                ss->getPlanner()->as<myRRTstar>()->setOptimize(true);
            }
            else { //default
                _opti=1;
                _optiselected = _lengthopti; //length optimization
                setParameter("Optimize none(0)/dist(1)/clear(2)/PMD(3)", _simplify);
                ss->getPlanner()->as<myRRTstar>()->setOptimize(true);
            }

            pdefPtr->setOptimizationObjective(_optiselected);

            ss->getPlanner()->setup();
        }
        else
          return false;



        it = _parameters.find("disablePMDControlsFromSampling");
        if(it != _parameters.end()){
            _disablePMDControlsFromSampling = it->second;
            if(_disablePMDControlsFromSampling==0)
            {
                disablePMDControlsFromSampling(true);//enable all controls
            }
            else
            {
                disablePMDControlsFromSampling(false);//disable those named PMD
                //setParameter("disablePMDControlsFromSampling", 1); //force to 1
            }
        }
        else
          return false;


        it = _parameters.find("Goal Bias");
        if(it != _parameters.end()){
            _GoalBias = it->second;
            ss->getPlanner()->as<og::RRTstar>()->setGoalBias(_GoalBias);
        }
        else
          return false;

        it = _parameters.find("K-Neigh Factor");
        if(it != _parameters.end()){
            _KneighFactor = it->second;
            ss->getPlanner()->as<myRRTstar>()->setNeighFactor(_KneighFactor);
        }
        else
          return false;

        it = _parameters.find("DelayCC (0/1)");
        if(it != _parameters.end()){
            if(it->second == 0) _DelayCC = false;
            else _DelayCC = true;
            ss->getPlanner()->as<og::RRTstar>()->setDelayCC(_DelayCC);
        }
        else
          return false;

        it = _parameters.find("lengthweight(0..1)");
        if(it != _parameters.end()){
            if(it->second >=0.0 && it->second<=1.0) _lengthweight = it->second;
            else _lengthweight = 0.5;

            if(_opti==3)
                ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setDistanceWeight(_lengthweight);
        }
        else
          return false;

        it = _parameters.find("penalizationweight");
        if(it != _parameters.end()){
            _penalizationweight = it->second;

            if(_opti==3)
                ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationPenalization(_penalizationweight);
        }
        else
          return false;

        it = _parameters.find("orientationweight");
        if(it != _parameters.end()){
            _orientationweight = it->second;

            if(_opti==3)
                ((PMDalignmentOptimizationObjective*)_pmdalignmentopti.get())->setOrientationWeight(_orientationweight);
        }
        else
          return false;

      }catch(...){
        return false;
      }
      return true;
    }
  }
}


#endif // KAUTHAM_USE_OMPL
