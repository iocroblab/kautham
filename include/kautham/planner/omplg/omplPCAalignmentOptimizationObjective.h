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

#if !defined(_omplPCAOBJECTIVE_H)
#define _omplPCAOBJECTIVE_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/Path.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <Eigen/Core>


namespace ob = ompl::base;


namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */
  namespace omplplanner{

  class PMDalignmentOptimizationObjective:public ob::OptimizationObjective {
      protected:
      ob::ProjectionMatrix PMD; //!< The copupling matrix
      Eigen::VectorXd lambda;//!< The eignevalues of the PMDs.
      int numPMD; //!< Number of PMDs considered for the hand. It corresponds to the number of columns of te copupling matrix
      int numDOF; //!< Number of DOF coupled. It corresponds to the number of rows of the coupling matrix

      double wpenalization; //!< To penalize changes in orientation between consecutive edges of a path
      double wdistance;//!< To weight the distance
      double worientation;//!< To weight the alignment with the PMDs
      double epsilon;//!< advance step of the RRT, used to equilize the three parts of the cost function

  public:
      PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~PMDalignmentOptimizationObjective();

      virtual ob::Cost stateCost(const ob::State *s) const {(void)s; return ob::Cost();}
      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const = 0;
      virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
      ob::Cost getCost(const ob::Path &path) const;
      void setPCAdata(ob::ProjectionMatrix M);
      inline double getOrientationPenalization(){return wpenalization;}
      inline void setOrientationPenalization(double w){wpenalization=w;}
      inline double getDistanceWeight(){return wdistance;}
      inline void setDistanceWeight(double w){wdistance=w;}
      inline void setOrientationWeight(double w){worientation=w;}
      inline double getOrientationWeight(){return worientation;}
      inline double getEpsilon(){return epsilon;}
      inline void setEpsilon(double e){epsilon=e;}
    };

  class singleRobotPMDalignmentOptimizationObjective:public PMDalignmentOptimizationObjective {
      private:
      double weightSE3;//!< To weight the costs in Rn subspace (cost=distcost+alignmentcost+penalization)
      double weightRn;//!< To weight the cost in SE3 subspace (cost=distance)
      int robotindex;

  public:
      singleRobotPMDalignmentOptimizationObjective(int roboti, const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~singleRobotPMDalignmentOptimizationObjective();

      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
      ob::Cost motionCostRn(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
      ob::Cost motionCostSE3(const ob::State *s1, const ob::State *s2) const;
    };


  class multiRobotSE3PMDalignmentOptimizationObjective:public PMDalignmentOptimizationObjective {
  public:
      multiRobotSE3PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~multiRobotSE3PMDalignmentOptimizationObjective();

      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
    };
  }
  /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplPCAOBJECTIVE_H
