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
#include "pcaresult.h"

namespace Kautham {
namespace omplplanner {
class myPCARRT:public ompl::geometric::RRT {
protected:
    std::vector<VelocityPCAResult*> *pmdSet_;
    double alfa_;
    double pmdBias_;

    //! New qRand
    arma::vec new_qRand(arma::vec qr, arma::vec qn, unsigned int index);

    unsigned int indexOf(arma::vec qn) {
        //return qn[0] <= 500.;

        unsigned int n = std::floor(std::min(qn[1],999.)/125.);
        bool t = std::abs(std::abs(qn[0]-500.)-250.) > 125. &&
                (std::abs(qn[1]-500.) < 375. ||
                 std::abs(qn[0]-500.) > 375.);
        if (t)  {
            double xb = floor(qn[0]/500.+0.5)*500.;
            double yb;
            if (std::abs(qn[0]-500.) < 125.) {
                yb = (n+n%2)*125.;
            } else {
                yb = (n+(n+1)%2)*125.;
            }
            t = (std::abs(qn[0]-xb) + std::abs(qn[1]-yb)) < 125.;
        }
        if (t) {
            if (qn[0] > 500.) {
                return 0;
            } else {
                return 1;
            }
        } else {
            if (n % 2) {
                return 2;
            } else {
                return 3;
            }
        }
    }

public:
    myPCARRT(const ompl::base::SpaceInformationPtr &si):RRT(si) {
        name_ = "myPCARRT";
        pmdSet_ = NULL;
        alfa_ = 0.8;
        pmdBias_ = 0.8;
        Planner::declareParam<double>("alfa",this,&myPCARRT::setAlfa,&myPCARRT::getAlfa,"0.:.01:1.");
        Planner::declareParam<double>("pmdBias",this,&myPCARRT::setPMDbias,&myPCARRT::getPMDbias,"0.:.01:1.");
    }

    void setAlfa(double alfa) {
        alfa_ = alfa;
    }

    double getAlfa() const {
        return alfa_;
    }

    void setPMDbias(double pmdBias) {
        pmdBias_ = pmdBias;
    }

    double getPMDbias() const {
        return pmdBias_;
    }

    void setPMDset(std::vector<VelocityPCAResult*> *pmdSet) {
        pmdSet_ = pmdSet;
    }

    std::vector<VelocityPCAResult*> *getPMDset() const {
        return pmdSet_;
    }

    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
};
}
}
