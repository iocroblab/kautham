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

 

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include "omplMyOptimizationObjective.h"
#include <math.h>


namespace Kautham {
  namespace omplplanner{

  /*! Constructor.
   *  \param si is the space information of the problem
   *  \param enableMotionCostInterpolation is a flag set false by default.
   */
  myOptimizationObjective::myOptimizationObjective(const ob::SpaceInformationPtr &si,
                                                   omplPlanner *p,
                                                   bool enableMotionCostInterpolation) :
      ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
  {
    pl = p;
  }

  /*! void destructor
   */
  myOptimizationObjective::~myOptimizationObjective(){

  }


  void myOptimizationObjective::setControlPoints(std::vector< std::vector<double> > *cp)
  {
      controlpoints.resize(cp->size());//number of control points
      for(int i=0; i<cp->size(); i++)
      {
          controlpoints[i].resize(cp->at(i).size());//dimension of the space
          for(int j=0;j<cp->at(i).size();j++)
              controlpoints[i].at(j) = cp->at(i).at(j);
      }
  }

  void myOptimizationObjective::setCostParams(std::vector< std::pair<double,double> > *cp) {
      costParams.resize(cp->size());
      for (int i = 0; i < cp->size(); ++i) {
          costParams[i].first = cp->at(i).first;
          costParams[i].second = cp->at(i).second;
      }
  }

  /*! stateCost.
   *  Computes the cost of the state s
   */
  ob::Cost 	myOptimizationObjective::stateCost(const ob::State *s) const
  {

      /*
      for(int i=0;i<controlpoints.size();i++){
          std::cout<<"obstacle["<<i<<"] = ( ";
          for(int j=0;j<controlpoints[i].size();j++)
              std::cout<<controlpoints[i].at(j)<<" ";
          std::cout<<")"<<std::endl;
      }
      std::cout<<std::endl;
      */


      Sample *smp = new Sample(3);
      //copy the conf of the init smp. Needed to capture the home positions.
      smp->setMappedConf(pl->initSamp()->getMappedConf());
      pl->omplState2smp(s, smp);
      double x = smp->getMappedConf()[0].getSE3().getPos().at(0);
      double y = smp->getMappedConf()[0].getSE3().getPos().at(1);
      double z = smp->getMappedConf()[0].getSE3().getPos().at(2);

      //std::cout<<"The state is: ("<<x<<", "<<y<<", "<<z<<")"<<std::endl;

      double xdist, ydist, zdist, dist, c;
      double totalcost = 0.0;
      double maxdist = sqrt(100.0*100.0+100.0*100.0+100.0*100.0);

      for(int i=0;i<controlpoints.size();i++){
          xdist = (x-controlpoints[i].at(0));
          ydist = (y-controlpoints[i].at(1));
          zdist = (z-controlpoints[i].at(2));
          dist = (xdist*xdist+ydist*ydist+zdist*zdist)/maxdist;
          c = costParams.at(i).first*exp(-costParams.at(i).second*dist);
          totalcost += c;
          //std::cout<< "Distance " << i<<" is "<<dist<<std::endl;
          //std::cout<< "Cost " << i<<" is "<<c<<std::endl;
      }
      //std::cout<< "The totalcost is:" << totalcost<<std::endl;
      return ob::Cost(totalcost);
  }


  }
}


#endif // KAUTHAM_USE_OMPL

