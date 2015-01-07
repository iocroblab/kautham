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

/* Author: Jan Rosell */

#if !defined(_omplMyOBJECTIVE_H)
#define _omplMyOBJECTIVE_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>


#include "omplplanner.h"
namespace ob = ompl::base;


namespace Kautham {
/** \addtogroup Planner
 *  @{
 */
  namespace omplplanner{

  class myOptimizationObjective:public ob::StateCostIntegralObjective {

  private:
      std::vector< std::vector<double> > controlpoints;
      double diffusion;
      omplPlanner *pl;
  public:
      myOptimizationObjective(const ob::SpaceInformationPtr &si, omplPlanner *p, bool enableMotionCostInterpolation=false);
      ~myOptimizationObjective();

      void setControlPoints(std::vector< std::vector<double> > *cp);

      inline void setDiffusion(double d){diffusion = d;};
      inline double getDiffusion(){return diffusion;};
      ob::Cost 	stateCost (const ob::State *s) const;
    };




  }
  /** @}   end of Doxygen module "Planner */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplMyOBJECTIVE_H

