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

/* Author: Enrique Ajenjo, Ely Repiso */

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <boost/numeric/ublas/matrix.hpp> // for Matrix, or eigen for matrix
#include <eigen3/Eigen/Dense>

////////////////////////////////////////////////////////////////////////////////////////////////////////
  // This is a class derived form the class ompl::RRT. Its purpose is to slightly change its behavior, by using the PCA for grow the RRT
 // !!!!! IMPORTANT: change its next definitions of the behavior of the PCARR, by its real behavior, not the behavior of the PRM description copied for follow the same way for of creation for the PCARRT class.
  //      1) Making the ratio fo the steps grow and expand variable. It is done in the reimplementation of the solve function
  //      2) The function expandRoadmap is changed by myexpandRoadmap. For now they are equal, but possible changes include
  //          the distance threshold in the edges of the bounce motions and changing the number of bounce steps

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace Kautham {
  namespace omplplanner{

 class PCARRT:public og::RRT
  {
  private:
         //definir aqui los parámetros que requiera el PCARRT, como por ejemplo el radio de vecindad del qnear para
         //calcular el pca
         double radius;
         int kPCA; //number of neighbors to compute the PCA
         int nDOF;

  public:

      PCARRT(const ob::SpaceInformationPtr &si, int n):RRT(si)
      {
          nDOF=2;
          n = nDOF;
          kPCA = 3*nDOF;//kPCA must be at least equal to nDOF.
          radius = 1.0;
      }

      ompl::base::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);

  };
//Final class PCARRT

  }
}
