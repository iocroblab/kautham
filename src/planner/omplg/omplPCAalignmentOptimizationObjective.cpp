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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "omplPCAalignmentOptimizationObjective.h"
#include "omplplanner.h" //for the definition of weightedRealVectorStateSpace class


#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <ompl/tools/config/MagicConstants.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>



using namespace boost::numeric::ublas;
namespace ob = ompl::base;
namespace om = ompl::magic;


namespace Kautham {
  namespace omplplanner{


  /*! Constructor.
   *  \param roboti is the index of the robot.
   *  \param si is the space information of the problem
   *  \param M is the PMD matrix (it has as many columns as controls and as many rows as DOF). It is taken from the robot mapMatrix.
   */
      PMDalignmentOptimizationObjective::PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M) :
          ob::OptimizationObjective(si)
      {
          description_ = "PMD alignment"; //This label is used in myRRTstar - be careful not to change it!

          wpenalization = 1.0;
          wdistance = 0.1;
          worientation = 1.0;
          epsilon = 0.1;

          numDOF = M.mat.size1();//rows
          numPMD = M.mat.size2();//columns
          lambda.resize(numPMD);
          PMD.mat.resize(numDOF,numPMD);
          setPCAdata(M);
      }

      /*! void destructor
       */
      PMDalignmentOptimizationObjective::~PMDalignmentOptimizationObjective(){

      }


  /*! The function setPCAdata loads the PMD matrix.
   */
  void PMDalignmentOptimizationObjective::setPCAdata(ob::ProjectionMatrix M)
  {
      double modul;

      for(int j=0;j<numPMD;j++)//column
      {
          modul = 0.0;
          for(int i=0;i<numDOF;i++)//row
          {
              PMD.mat(i,j) = M.mat(i,j);
              modul += M.mat(i,j)*M.mat(i,j);
          }
          lambda[j] = sqrt(modul);
          for(int i=0;i<numDOF;i++) //columns vectors must be unitary vectors
              PMD.mat(i,j) /= lambda[j];
      }
      cout<<"PMD matrix: "<<endl;
      PMD.print();
  }

  /*!
   * This function is only used in the drawcspace function, because the RRTstar calls the motionCost(s0,s1,s2) to
   * include the penalization for the changes in orientation. In the drawcspace function this is not possible since
   * we loose the information of the previous state (s0).
   */
  ob::Cost PMDalignmentOptimizationObjective::motionCost(const ob::State *s1, const ob::State *s2) const
  {
      return motionCost(NULL,s1,s2);
  }

  /*!
   * This function computes the cost of a path. It reimplements the computation of the cost of a path in order to take into account the orientation cost correctly
   */

  ob::Cost  PMDalignmentOptimizationObjective::getCost(const ob::Path &path) const
  {
      // Cast path down to a PathGeometric
      const og::PathGeometric *pathGeom = dynamic_cast<const og::PathGeometric*>(&path);

      // Give up if this isn't a PathGeometric or if the path is empty.
      if (!pathGeom)
      {
          OMPL_ERROR("Error: Cost computation is only implemented for paths of type PathGeometric.");
          return this->identityCost();
      }
      else
      {
          std::size_t numStates = pathGeom->getStateCount();
          if (numStates == 0)
          {
              OMPL_ERROR("Cannot compute cost of an empty path.");
              return this->identityCost();
          }
          else
          {
              // Compute path cost by accumulating the cost along the path
              ob::Cost cost(this->identityCost());
              cout<<"getCost: :";
              for (std::size_t i = 1; i < numStates; ++i)
              {
                  const ob::State *s0;
                  if(i==1) s0 = NULL;
                  else s0 = pathGeom->getState(i-2);
                  const ob::State *s1 = pathGeom->getState(i-1);
                  const ob::State *s2 = pathGeom->getState(i);
                  cost = this->combineCosts(cost, motionCost(s0, s1, s2));

                  cout<<" "<<cost.v;
              }
              cout<<endl;

              return cost;
          }
      }
  }




  /*! \class singleRobotPMDalignmentOptimizationObjective
   *  For a given robot, this objective function aims the obtaintion of a path as much aligned to the main PMDs as possible.
   *  The PMDs are assumed to be defined on the Rn part of the robot.
   *  It defines a cost composed of three weighted components:
   *    1) distance cost: It measures the lenght of each edge of the path
   *    2) alignment cost: this cost evaluates the alignment of the path edge along the mian PMDs. Each edge is projected onto the eignevectors
   *         of the PMDs and scaled according to its eigenvalues; the module of this projection is computed. Then this cost is
   *          (edge_module/projection_module - 1). It is zero when the edge is aligned along the main PMD.
   *    3) penalization cost: this cost increases when the path has many turns. It is evaluated by measurng the angle between consecutive edges,
   *        the bigger the angle the higher the cost is.
   *  The cost is associated to a single robot, and the PMDs are assumed to be defined for the copupling of the Rn part (the chain). If the robot
   *  has mobile edge, this contributes to the cost as the (weighted) distance of the SE3 part.
   *
   *  IMPORTANT: The controls of the robot that can take part in this objective funtion must have the word PMD in its name, e.g. handPMD1.
   */
    //////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

  /*! Constructor.
   *  \param roboti is the index of the robot.
   *  \param si is the space information of the problem
   *  \param M is the PMD matrix (it has as many columns as controls and as many rows as DOF). It is taken from the robot mapMatrix.
   */
      singleRobotPMDalignmentOptimizationObjective::singleRobotPMDalignmentOptimizationObjective(int roboti, const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M) :
          PMDalignmentOptimizationObjective(si, M)
      {
          //do not overwrite description_ that is set in the constructor of PMDalignmentOptimizationObjective class

          robotindex = roboti;
          weightSE3 = 1.0;
          weightRn = 1.0;
      }

      /*! void destructor
       */
      singleRobotPMDalignmentOptimizationObjective::~singleRobotPMDalignmentOptimizationObjective(){

      }



      /*! The function motionCost computes the cost of an edge s1s2. The function extracts the subspace of the robot and weights the
        * cost of the SE3 part (if it exisits) and of the Rn part (if it exisits), returned by funtions  motionCostSE3 and motionCostRn, respectively.
        *  \param s0 is the tail node of the previous edge. It can be null.
        *  \param s1 is the tail of the edge
        *  \param s2 is the head of the edge
       */
      ob::Cost singleRobotPMDalignmentOptimizationObjective::motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const
      {
          double cost=0.0;
          ob::StateSpacePtr space = getSpaceInformation()->getStateSpace();

          ob::ScopedState<ob::CompoundStateSpace> sstate1(space);
          ob::ScopedState<ob::CompoundStateSpace> sstate2(space);
          sstate1 = *s1;
          sstate2 = *s2;

          //get subspace of robot robotindex
          ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(robotindex));
          for(unsigned j=0; j < ssRoboti->as<ob::CompoundStateSpace>()->getSubspaceCount(); j++)
          {
             ob::StateSpacePtr ssRobotij =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(j));
             if(ssRobotij->getType()==ob::STATE_SPACE_SE3)
             {
                 ob::ScopedState<ob::SE3StateSpace> robotise3_s1(ssRobotij);
                 ob::ScopedState<ob::SE3StateSpace> robotise3_s2(ssRobotij);
                 sstate1 >> robotise3_s1;
                 sstate2 >> robotise3_s2;
                 ob::Cost costSE3 = motionCostSE3(robotise3_s1.get(),robotise3_s2.get());
                 cost += weightSE3 * costSE3.v;
             }
             else
             {
                 if(s0!=NULL)
                 {
                    ob::ScopedState<ob::CompoundStateSpace> sstate0(space);
                    sstate0 = *s0;
                    ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s0(ssRobotij);
                    sstate0 >> robotiRn_s0;

                    ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s1(ssRobotij);
                    ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s2(ssRobotij);
                    sstate1 >> robotiRn_s1;
                    sstate2 >> robotiRn_s2;
                    ob::Cost costRn = motionCostRn(robotiRn_s0.get(),robotiRn_s1.get(),robotiRn_s2.get());
                    cost += weightRn * costRn.v;
                 }
                 else
                 {
                    ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s1(ssRobotij);
                    ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s2(ssRobotij);
                    sstate1 >> robotiRn_s1;
                    sstate2 >> robotiRn_s2;
                    ob::Cost costRn = motionCostRn(NULL,robotiRn_s1.get(),robotiRn_s2.get());
                    cost += weightRn * costRn.v;
                 }
              }
          }
          return ob::Cost(cost);
      }

      /*! The function motionCostSE3 computes the cost of the SE3 part of edge s1s2, as the SE3 distance.
        *  \param s1 is the tail of the edge (a state of a SE3 State Space)
        *  \param s2 is the head of the edge (a state of a SE3 State Space)
       */
      ob::Cost singleRobotPMDalignmentOptimizationObjective::motionCostSE3(const ob::State *s1, const ob::State *s2) const
      {
          return ob::Cost(distance(s1,s2));
      }


      /*! The function motionCostRn computes the cost of the Rn part of edge s1s2.
        *  \param s0 is the tail of the previous edge (a state of a weigthedRealVectorStateSpace). It can be null.
        *  \param s1 is the tail of the edge (a state of a weigthedRealVectorStateSpace)
        *  \param s2 is the head of the edge (a state of a weigthedRealVectorStateSpace)
        * The cost has three weighted components:
        *    1) distance cost: It measures the lenght of each edge of the path
        *    2) alignment cost: this cost evaluates the alignment of the path edge along the mian PMDs. Each edge is projected onto the eignevectors
        *         of the PMDs and scaled according to its eigenvalues; the module of this projection is computed. Then this cost is
        *          (edge_module/projection_module - 1). It is zero when the edge is aligned along the main PMD.
        *    3) penalization cost: this cost increases when the path has many turns. It is evaluated by measurng the angle between consecutive edges,
        *        the bigger the angle the higher the cost is.
       */
      ob::Cost singleRobotPMDalignmentOptimizationObjective::motionCostRn(const ob::State *s0, const ob::State *s1, const ob::State *s2) const
      {
          std::vector<double> s1_coords;
          std::vector<double> s2_coords;
          s1_coords.resize(numDOF);
          s2_coords.resize(numDOF);
          for(int i=0;i<numDOF;i++)
          {
              s1_coords[i] = s1->as<weigthedRealVectorStateSpace::StateType>()->values[i];
              s2_coords[i] = s2->as<weigthedRealVectorStateSpace::StateType>()->values[i];
          }

          ////////////////
          //DISTANCE COST:
          ////////////////
          //vector from s1 to s2 in state space: from12 = s2-s1
          std::vector<double> edge12(numDOF);
          double modul12=0.0;
          for(int i=0; i<numDOF;i++)
          {
              edge12[i] = s2_coords[i] - s1_coords[i];
              modul12 += edge12[i]*edge12[i];
          }
          modul12 = sqrt(modul12);
          double distcost = wdistance*modul12;
          ////////////////
          //ALIGNMENT COST:
          ////////////////
          //normalize the lambdas (eignvalues).
          std::vector<double> lambdanorm;
          lambdanorm.resize(numPMD);
          double lambdamax=lambda[0];
          double lambdamin=lambda[0];
          //The first one is always the largest (it should be!). Check it, perhaps they are not ordered....
          for(int i=1; i<numPMD;i++){
             if(lambda[i]>lambdamax) lambdamax=lambda[i];
             if(lambda[i]<lambdamin) lambdamin=lambda[i];
          }
          for(int i=0; i<numPMD;i++)
              lambdanorm[i] = lambda[i] / lambdamax;

          //The projection of the edge onto each PMD (using the dot product) weighted by the lambda:
          std::vector<double> projectionPMD(numPMD);
          for(int j=0;j<numPMD;j++)//columns
          {
              projectionPMD[j] = 0.0;
              for(int i=0;i<numDOF;i++)//rows
                    projectionPMD[j] += edge12[i]*PMD.mat(i,j);
              projectionPMD[j] *= lambdanorm[j];
          }
          //The new edge resulting from the weighted projections has the following module:
          double newmodul12=0.0;
          for(int j=0;j<numPMD;j++)//columns
          {
              newmodul12 += projectionPMD[j]*projectionPMD[j];
          }
          newmodul12 = sqrt(newmodul12);
          double threshold = modul12*lambdamin/100.0;
          if(newmodul12<threshold) newmodul12 = threshold;//if the edge is perpendicular to all the PMDs then the newmodul12 obtained is zero.
                                                         //set it to a small value in order not to have an infinite cost.

          //The value alpha is capture the variation in modules. The more aligned the edge is with the main PMDs then
          //the smaller is alpha because then newmodul12 i approximately equal to modul12. When the edge is exactly
          //aligned with the main PMD then alpha is equal to zero.
          double alpha = modul12/newmodul12;
          //double alpha = log10(modul12/newmodul12);
          //double alpha = (modul12/newmodul12)-1.0;
          //modul12 should always be larger than newmodul12  because the lambdas are normalized. Then alpha must be >= 0
          if(alpha<0)
          {
              alpha=0.0; //this should never happen
          }
          //Finally the alignment cost is:
          double orientcost=alpha*worientation*modul12;
          //double orientcost=alpha*worientation*epsilon;

          ////////////////
          //PENALIZATION COST
          ////////////////
          //Compute now the possible penalization due to a big change in orientation
          double orientationpenalization=0.0;
          if(s0!=NULL) //only avalilable if state s0 (the one prior to s1) is given.
          {
              std::vector<double> s0_coords;
              s0_coords.resize(numDOF);
              for(int i=0;i<numDOF;i++)
                  s0_coords[i] = s0->as<weigthedRealVectorStateSpace::StateType>()->values[i];

              //vector from s0 to s1 in state space: from01 = s1-s0
              std::vector<double> edge01(numDOF);
              double modul01=0.0;
              for(int i=0; i<numDOF;i++)
              {
                  edge01[i] = s1_coords[i] - s0_coords[i];
                  modul01 += edge01[i]*edge01[i];
              }
              modul01 = sqrt(modul01);

              double escalarProduct=0;
              for(int i=0; i<numDOF;i++)
              {
                  escalarProduct += edge01[i]*edge12[i];
              }
              double cosbeta = escalarProduct/(modul01*modul12);
              if(cosbeta>1.0) cosbeta=1.0; //should not be necessary...
              if(cosbeta<-1.0) cosbeta=-1.0;

              //orientationpenalization = acos(cosbeta)*wpenalization*modul12;
              orientationpenalization = acos(cosbeta)*wpenalization*epsilon;
          }

          //std::cout<<" d="<<distcost<<" o="<<orientcost<<" p="<<orientationpenalization<<" "<<std::endl;
          return ob::Cost(distcost+orientcost+orientationpenalization);
      }



      //////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////

    /*! Constructor.
     *  \param si is the space information of the problem
     *  \param M is the PMD matrix (it has as many columns as controls and as many rows as DOF). It is taken from the robots mapMatrix.
     */
      //la matriu M conte els PMDs. Les columnes son els controls que estan acoblats entre robots
      //les files son els gdl de tots els robots
        multiRobotSE3PMDalignmentOptimizationObjective::multiRobotSE3PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M) :
            PMDalignmentOptimizationObjective(si, M)
        {
            //do not overwrite description_ that is set in the constructor of PMDalignmentOptimizationObjective class
        }

        /*! void destructor
         */
        multiRobotSE3PMDalignmentOptimizationObjective::~multiRobotSE3PMDalignmentOptimizationObjective(){

        }



        /*! The function motionCost computes the cost of an edge s1s2.
          *  \param s0 is the tail node of the previous edge. It can be null.
          *  \param s1 is the tail of the edge
          *  \param s2 is the head of the edge
         */
        ob::Cost multiRobotSE3PMDalignmentOptimizationObjective::motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const
        {
            ob::StateSpacePtr space = getSpaceInformation()->getStateSpace();

            ob::ScopedState<ob::CompoundStateSpace> sstate1(space);
            ob::ScopedState<ob::CompoundStateSpace> sstate2(space);
            sstate1 = *s1;
            sstate2 = *s2;

            unsigned int numrobots = space->as<ob::CompoundStateSpace>()->getSubspaceCount();
            std::vector<double> s1_coords;
            std::vector<double> s2_coords;
            for(unsigned i=0; i<numrobots; i++)
            {
                //get subspace of robot i
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(i));
                ob::StateSpacePtr ssRobotise3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));
                //load s1 data
                ob::ScopedState<ob::SE3StateSpace> robotse3_s1(ssRobotise3);
                sstate1 >> robotse3_s1;
                s1_coords.push_back(robotse3_s1->getX());
                s1_coords.push_back(robotse3_s1->getY());
                s1_coords.push_back(robotse3_s1->getZ());
                //load s2 data
                ob::ScopedState<ob::SE3StateSpace> robotse3_s2(ssRobotise3);
                sstate2 >> robotse3_s2;
                s2_coords.push_back(robotse3_s2->getX());
                s2_coords.push_back(robotse3_s2->getY());
                s2_coords.push_back(robotse3_s2->getZ());
            }

            ////////////////
            //DISTANCE COST:
            ////////////////
            //vector from s1 to s2 in state space: from12 = s2-s1
            std::vector<double> edge12(numDOF);
            double modul12=0.0;
            for(int i=0; i<numDOF;i++)
            {
                edge12[i] = s2_coords[i] - s1_coords[i];
                modul12 += edge12[i]*edge12[i];
            }
            modul12 = sqrt(modul12);
            double distcost = wdistance*modul12;
            ////////////////
            //ALIGNMENT COST:
            ////////////////
            //normalize the lambdas (eignvalues).
            std::vector<double> lambdanorm;
            lambdanorm.resize(numPMD);
            double lambdamax=lambda[0];
            double lambdamin=lambda[0];
            //The first one is always the largest (it should be!). Check it, perhaps they are not ordered....
            for(int i=1; i<numPMD;i++){
               if(lambda[i]>lambdamax) lambdamax=lambda[i];
               if(lambda[i]<lambdamin) lambdamin=lambda[i];
            }
            for(int i=0; i<numPMD;i++)
                lambdanorm[i] = lambda[i] / lambdamax;

            //The projection of the edge onto each PMD (using the dot product) weighted by the lambda:
            std::vector<double> projectionPMD(numPMD);
            for(int j=0;j<numPMD;j++)//columns
            {
                projectionPMD[j] = 0.0;
                for(int i=0;i<numDOF;i++)//rows
                      projectionPMD[j] += edge12[i]*PMD.mat(i,j);
                projectionPMD[j] *= lambdanorm[j];
            }
            //The new edge resulting from the weighted projections has the following module:
            double newmodul12=0.0;
            for(int j=0;j<numPMD;j++)//columns
            {
                newmodul12 += projectionPMD[j]*projectionPMD[j];
            }
            newmodul12 = sqrt(newmodul12);
            if(newmodul12<lambdamin/100.0) newmodul12 = lambdamin/100.0;//if the edge is perpendicular to all the PMDs then the newmodul12 obtained is zero.
                                                           //set it to a small value in order not to have an infinite cost.

            //The value alpha is capture the variation in modules. The more aligned the edge is with the main PMDs then
            //the smaller is alpha because then newmodul12 i approximately equal to modul12. When the edge is exactly
            //aligned with the main PMD then alpha is equal to zero.
            double alpha = modul12/newmodul12;
            //double alpha = log10(modul12/newmodul12);
            //double alpha = (modul12/newmodul12)-1.0;
            //modul12 should always be larger than newmodul12  because the lambdas are normalized. Then alpha must be >= 0
            if(alpha<0)
            {
                alpha=0.0; //this should never happen
            }
            //Finally the alignment cost is:
            double orientcost=alpha*worientation*modul12;

            ////////////////
            //PENALIZATION COST
            ////////////////
            //Compute now the possible penalization due to a big change in orientation
            double orientationpenalization=0.0;
            if(s0!=NULL) //only avalilable if state s0 (the one prior to s1) is given.
            {
                ob::ScopedState<ob::CompoundStateSpace> sstate0(space);
                sstate0 = *s0;
                std::vector<double> s0_coords;
                for(unsigned i=0; i<numrobots; i++)
                {
                    //get subspace of robot i
                    ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(i));
                    ob::StateSpacePtr ssRobotise3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));
                    //load s1 data
                    ob::ScopedState<ob::SE3StateSpace> robotse3_s0(ssRobotise3);
                    sstate0 >> robotse3_s0;
                    s0_coords.push_back(robotse3_s0->getX());
                    s0_coords.push_back(robotse3_s0->getY());
                    s0_coords.push_back(robotse3_s0->getZ());
                }

                //vector from s0 to s1 in state space: from01 = s1-s0
                std::vector<double> edge01(numDOF);
                double modul01=0.0;
                for(int i=0; i<numDOF;i++)
                {
                    edge01[i] = s1_coords[i] - s0_coords[i];
                    modul01 += edge01[i]*edge01[i];
                }
                modul01 = sqrt(modul01);

                double escalarProduct=0;
                for(int i=0; i<numDOF;i++)
                {
                    escalarProduct += edge01[i]*edge12[i];
                }
                double cosbeta = escalarProduct/(modul01*modul12);
                if(cosbeta>1.0) cosbeta=1.0; //should not be necessary...
                if(cosbeta<-1.0) cosbeta=-1.0;

                //orientationpenalization = acos(cosbeta)*wpenalization*modul12;
                orientationpenalization = acos(cosbeta)*wpenalization*epsilon;
            }

            //std::cout<<" d="<<distcost<<" o="<<orientcost<<" p="<<orientationpenalization<<" "<<std::endl;


            return ob::Cost(distcost+orientcost+orientationpenalization);
        }








      /////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////

      /////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////

      /*
        //http://savingyoutime.wordpress.com/2009/09/21/c-matrix-inversion-boostublas/
        // Matrix inversion routine.
        //    Uses lu_factorize and lu_substitute in uBLAS to invert a matrix
        template<class T>
            bool InvertMatrix(const matrix<T>& input, matrix<T>& inverse)
            {
            typedef permutation_matrix<std::size_t> pmatrix;

            // create a working copy of the input
            matrix<T> A(input);

            // create a permutation matrix for the LU-factorization
            pmatrix pm(A.size1());

            // perform LU-factorization
            int res = lu_factorize(A, pm);
            if (res != 0)
                return false;

            // create identity matrix of "inverse"
            inverse.assign(identity_matrix<T> (A.size1()));

            // backsubstitute to get the inverse
            lu_substitute(A, pm, inverse);

            return true;
        }
            */

    /*
      //! Constructor
        PCAalignmentOptimizationObjective::PCAalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, int dim, ob::ProjectionMatrix M) :
        ob::OptimizationObjective(si)
        {
            description_ = "PCA alignment";
            PCAdataset=false;
            dimension = dim;
            wpenalization = 1.0;
            wdistance = 0.1;
            worientation = 1.0;

            lambda.resize(dimension);
            pcaM.mat.resize(dimension,dimension);
            setPCAdata(M);
        }

        //! void destructor
        PCAalignmentOptimizationObjective::~PCAalignmentOptimizationObjective(){

        }



        //void PCAalignmentOptimizationObjective::setPCAdata(int option)//ob::ProjectionMatrix M, ob::EuclideanProjection v){
        void PCAalignmentOptimizationObjective::setPCAdata(ob::ProjectionMatrix M)
        {
            Matrix pca(dimension,dimension);
            Matrix invpca(dimension,dimension);

            double modul;
            for(int j=0;j<dimension;j++)//column
            {
                modul = 0.0;
                for(int i=0;i<dimension;i++)//row
                {
                    pca(i,j) = M.mat(i,j);
                    modul += M.mat(i,j)*M.mat(i,j);
                }
                lambda[j] = sqrt(modul);
                for(int i=0;i<dimension;i++) //columns vectors must be unitary vectors
                    pca(i,j) /= lambda[j];
            }
            pcaM.mat = pca;
            pcaM.print();

            InvertMatrix(pca, invpca);
            pcaMinv.mat = invpca;
            pcaMinv.print();

            PCAdataset=true;
        }


        ob::Cost PCAalignmentOptimizationObjective::motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const
        {
            if(wdistance==0.0 && worientation==0 && wpenalization==0) return ob::Cost(1.0);



            ob::StateSpacePtr space = getSpaceInformation()->getStateSpace();

            ob::ScopedState<ob::CompoundStateSpace> ss1(space);
            ob::ScopedState<ob::CompoundStateSpace> ss2(space);
            ss1 = *s1;
            ss2 = *s2;

            //Get the SE3 subspace of robot 0
            ob::StateSpacePtr ssRobot0 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
            ob::StateSpacePtr ssRobot0rn =  ((ob::StateSpacePtr) ssRobot0->as<ob::CompoundStateSpace>()->getSubspace(0));
            ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s1(ssRobot0rn);
            ss1 >> robotiRn_s1;
            ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s2(ssRobot0rn);
            ss2 >> robotiRn_s2;
            std::vector<double> s1_se3coords;
            std::vector<double> s2_se3coords;
            s1_se3coords.resize(2);
            s2_se3coords.resize(2);
            s1_se3coords[0] = robotiRn_s1->values[0];
            s1_se3coords[1] = robotiRn_s1->values[1];
            s2_se3coords[0] = robotiRn_s2->values[0];
            s2_se3coords[1] = robotiRn_s2->values[1];


            //vector from s1 to s2 in state space: from12 = s2-s1
            std::vector<double> from12(dimension);
            double modul12=0.0;
            for(int i=0; i<dimension;i++)
            {
                from12[i] = s2_se3coords[i] - s1_se3coords[i];
                modul12 += from12[i]*from12[i];
            }
            modul12 = sqrt(modul12);

            //vector from s1 to s2 using the pca reference frame: vpca = M*v
            ob::EuclideanProjection to12(dimension);
            pcaMinv.project(&from12[0],to12);


            //normalize the lambdas (eignvalues).
            std::vector<double> lambdanorm;
            lambdanorm.resize(dimension);
            double lambdamax=0.0;
            //The first one is always the largest (it should be!). Chack it, perhaps they are not ordered....
            for(int i=0; i<dimension;i++)
            {
                if(lambda[i]>lambdamax) lambdamax=lambda[i];
            }
            for(int i=0; i<dimension;i++)
            {
                lambdanorm[i] = lambda[i] / lambdamax;
            }

            double modulto12=0.0;
            for(int i=0; i<dimension;i++)
            {
                modulto12 += to12[i]*to12[i]*lambdanorm[i]*lambdanorm[i];
            }
            modulto12 = sqrt(modulto12);




            //std::cout<<alpha<<" "<<modul12/modulto12<<std::endl;
            //when aligned with the main PMD the cost is zero ((1/lambdanorm1)-1 = 0)
            //when aligned with the second PMD the cost is ((1/lambdanorm2)-1)
            double alpha = (modul12/modulto12)-1.0;
            //modul12 should always be larger than modulto12 because the lambdas are normalized. Then alpha must be >= 0
            if(alpha<0)
            {
                alpha=0.0; //this should not happen
            }
            double orientcost=alpha*worientation*modul12;

            //Compute now the possible penalization due to a big change in orientation
            //double orientationpenalization=1.0;
            double orientationpenalization=0.0;
            if(s0!=NULL)
            {
                ob::ScopedState<ob::CompoundStateSpace> ss0(space);
                ss0 = *s0;
                ob::ScopedState<weigthedRealVectorStateSpace> robotiRn_s0(ssRobot0rn);
                ss0 >> robotiRn_s0;
                std::vector<double> s0_se3coords;
                s0_se3coords.resize(2);
                s0_se3coords[0] = robotiRn_s0->values[0];
                s0_se3coords[1] = robotiRn_s0->values[1];



                //vector from s0 to s1 in state space: from01 = s1-s0
                std::vector<double> from01(dimension);
                double modul01=0.0;
                for(int i=0; i<dimension;i++)
                {
                    from01[i] = s1_se3coords[i] - s0_se3coords[i];
                    modul01 += from01[i]*from01[i];
                }
                modul01 = sqrt(modul01);

                double cosbeta = (from01[0]*from12[0]+from01[1]*from12[1])/(modul01*modul12);
                orientationpenalization = acos(cosbeta)*wpenalization*modul12;
            }

            double distcost = wdistance*modul12;
            //std::cout<<" d="<<distcost<<" o="<<orientcost<<" p="<<orientationpenalization<<" "<<std::endl;
            return ob::Cost(distcost+orientcost+orientationpenalization);
        }


        ob::Cost PCAalignmentOptimizationObjective::motionCost(const ob::State *s1, const ob::State *s2) const
        {
            motionCost(NULL,s1,s2);
        }
        */




  /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
    /*

  //! Constructor
    PCAalignmentOptimizationObjective3::PCAalignmentOptimizationObjective3(const ob::SpaceInformationPtr &si, int dim) :
    ob::MechanicalWorkOptimizationObjective(si)
    {
        description_ = "PCA alignment2";
        PCAdataset=false;
        dimension = dim;

        //de moment ho fixem a 2
        dimension = 2;
        setPCAdata(0);
    }

    //! void destructor
    PCAalignmentOptimizationObjective3::~PCAalignmentOptimizationObjective3(){

    }

    void PCAalignmentOptimizationObjective3::setPCAdata(int option)//ob::ProjectionMatrix M, ob::EuclideanProjection v){
    {
        //de moment ho fixo a ma
        Matrix pca(dimension,dimension);
        Matrix invpca(dimension,dimension);
        ob::EuclideanProjection v(dimension);
        bari.resize(dimension);

        if(option==0)
        {
            pca(0,0) = sqrt(2.0)/2.0;
            pca(0,1) = sqrt(2.0)/2.0;
            pca(1,0) = -sqrt(2.0)/2.0;
            pca(1,1) = sqrt(2.0)/2.0;
            InvertMatrix(pca, invpca);
            pcaM.mat = invpca;
            pcaM.print();
        }
        else
        {
            pca(0,0) = sqrt(2.0)/2.0;
            pca(0,1) = -sqrt(2.0)/2.0;
            pca(1,0) = sqrt(2.0)/2.0;
            pca(1,1) = sqrt(2.0)/2.0;
            InvertMatrix(pca, invpca);
            pcaM.mat = invpca;
            pcaM.print();
        }

        v[0] = 1.0;
        v[1] = 0.1;
        lambda = v;

        bari[0]=0.5;
        bari[1]=0.5;
        PCAdataset=true;
    }

    ob::Cost PCAalignmentOptimizationObjective3::motionCost(const ob::State *s1, const ob::State *s2) const
    {
      // Only accrue positive changes in cost
      double positiveCostAccrued = std::max(stateCost(s2).v - stateCost(s1).v, 0.0);
      return ob::Cost(wfix*positiveCostAccrued/si_->distance(s1,s2) + wdistance*si_->distance(s1,s2));
    }

    ob::Cost PCAalignmentOptimizationObjective3::stateCost(const ob::State *s1) const
    {
        ob::StateSpacePtr space = getSpaceInformation()->getStateSpace();

        ob::ScopedState<ob::CompoundStateSpace> ss1(space);
        ss1 = *s1;

        //Get the SE3 subspace of robot 0
        ob::StateSpacePtr ssRobot0 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
        ob::StateSpacePtr ssRobot0SE3 =  ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0));
        ob::ScopedState<ob::SE3StateSpace> s1se3(ssRobot0SE3);
        ss1 >> s1se3;

        //convert to a vector of 7 components
        vector<double> s1_se3coords;
        s1_se3coords.resize(2);
        s1_se3coords[0] = s1se3->getX();
        s1_se3coords[1] = s1se3->getY();

        vector<double> from;
        from.resize(dimension);
        for(int i=0; i<dimension;i++)
        {
            from[i] = s1_se3coords[i] - bari[i];
        }

        //vector from s1 to s2 using the pca reference frame: vpca = M*v
        ob::EuclideanProjection to(dimension);
        pcaM.project(&from[0],to);

        double dist=0.0;
        for(int i=0; i<dimension;i++)
        {
            dist += (to[i]/lambda[i])*(to[i]/lambda[i]);
        }
        dist = sqrt(dist);
        return ob::Cost(dist);
    }
  */
  }
}


#endif // KAUTHAM_USE_OMPL
