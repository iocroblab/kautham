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

/* Author: Jan Rosell, Nestor Garcia Hidalgo */

#if !defined(_omplMyOBJECTIVE_H)
#define _omplMyOBJECTIVE_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/Cost.h>

#include "omplplanner.h"
namespace ob = ompl::base;


namespace Kautham {
/** \addtogroup Planner
 *  @{
 */
  namespace omplplanner {

  class myMWOptimizationObjective : public ob::MechanicalWorkOptimizationObjective {
  private:
      std::vector<std::vector<double> > controlpoints;
      std::vector<std::pair<double,double> > costParams;
      omplPlanner *pl;
  public:
      myMWOptimizationObjective(const ob::SpaceInformationPtr &si, omplPlanner *p,
                                double pathLengthWeight = 0.00001);
      void setControlPoints(std::vector<std::vector<double> > *cp);
      void setCostParams(std::vector<std::pair<double,double> > *cp);
      void setPathLengthWeight(double weight) {pathLengthWeight_ = weight;}
      bool isSymmetric() {return false;}
      virtual ob::Cost stateCost(const ob::State *s) const;
  };

  class myICOptimizationObjective : public ob::MechanicalWorkOptimizationObjective {
  private:
      std::vector< std::vector<double> > controlpoints;
      std::vector< std::pair<double,double> > costParams;
      omplPlanner *pl;
      double pathLengthWeight_;
      bool interpolateMotionCost_;

      ob::Cost trapezoid(ob::Cost c1, ob::Cost c2, double dist) const {
          return ob::Cost(0.5*dist*(c1.v+c2.v));
      }
  public:
      myICOptimizationObjective(const ob::SpaceInformationPtr &si, omplPlanner *p,
                                double pathLengthWeight = 0.00001,
                                bool enableMotionCostInterpolation = false);
      void setControlPoints(std::vector< std::vector<double> > *cp);
      void setCostParams(std::vector<std::pair<double, double> > *cp);
      void setPathLengthWeight(double weight) {pathLengthWeight_ = weight;}
      bool isSymmetric() {return ob::OptimizationObjective::isSymmetric();}
      bool isMotionCostInterpolationEnabled() const {return interpolateMotionCost_;}
      virtual ob::Cost stateCost(const ob::State *s) const;
      virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
    };
  }
  /** @}   end of Doxygen module "Planner */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplMyOBJECTIVE_H

