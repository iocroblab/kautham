/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <boost/numeric/ublas/matrix.hpp> // for Matrix, or eigen for matrix

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace Kautham {
  namespace omplplanner {
    class PCARRT : public og::RRT {
    private:
      double radius;
      unsigned int kPCA; //number of neighbors to compute the PCA
      unsigned int nDOF;

    public:
      PCARRT(const ob::SpaceInformationPtr &si, int n) : RRT(si) {
        nDOF = 2;
        n = nDOF;
        kPCA = 3*nDOF;//kPCA must be at least equal to nDOF.
        radius = 1.;
      }

      ompl::base::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);
    };
  }
}
