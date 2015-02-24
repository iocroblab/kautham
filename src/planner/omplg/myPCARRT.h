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
#include "pcakdtree.h"

namespace Kautham {
namespace omplplanner {
class myPCARRT:public ompl::geometric::RRT {
protected:
    PCAkdtree *tree_;
    double timeStep_;
    double pmdBias_;

    //! New qRand
    arma::vec new_qRand(arma::vec qr, arma::vec qn);

public:
    myPCARRT(const ompl::base::SpaceInformationPtr &si):RRT(si) {
        name_ = "myPCARRT";
        tree_ = NULL;
        timeStep_ = 1.;
        pmdBias_ = 1.;
        Planner::declareParam<double>("timeStep",this,&myPCARRT::setTimeStep,&myPCARRT::getTimeStep,"0.:.001:1000.");
        Planner::declareParam<double>("pmdBias",this,&myPCARRT::setPMDbias,&myPCARRT::getPMDbias,"0.:.01:1.");
    }

    void setTimeStep(double timeStep) {
        timeStep_ = timeStep;
    }

    double getTimeStep() const {
        return timeStep_;
    }

    void setPMDbias(double pmdBias) {
        pmdBias_ = pmdBias;
    }

    double getPMDbias() const {
        return pmdBias_;
    }

    void setPCAkdtree(PCAkdtree * tree) {
        tree_ = tree;
    }

    PCAkdtree *getPCAkdtree() const {
        return tree_;
    }

    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
};
}
}
