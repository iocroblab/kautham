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
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/Cost.h>

#include <kautham/planner/omplg/synergy_tree.h>
#include <kautham/planner/omplg/omplplanner.h>

namespace ob = ompl::base;


namespace Kautham {
    /** \addtogroup GeometricPlanners
     *  @{
     */
    namespace omplplanner {
        struct Segment {
            Point3 p0, p1;
            Segment(const Point3 startPoint, const Point3 endPoint)
                : p0(startPoint),p1(endPoint) {

            }
            double radius;
        };
        struct RobotSegment {
            Robot *robot;
            Link *link;
            Point3 p0, p1;
            double radius;
            Segment segment() const {
                mt::Transform t(mt::Rotation(link->getElement()->getOrientation()[0],
                                link->getElement()->getOrientation()[1],
                        link->getElement()->getOrientation()[2],
                        link->getElement()->getOrientation()[3]),
                        mt::Point3(link->getElement()->getPosition()[0],
                        link->getElement()->getPosition()[1],
                        link->getElement()->getPosition()[2]));

                return Segment(t*p0,t*p1);
            }
        };

        class myMWOptimizationObjective : public ob::MechanicalWorkOptimizationObjective {
        public:
            myMWOptimizationObjective(const ob::SpaceInformationPtr &si, omplPlanner *p,
                                      double pathLengthWeight = 0.00001);
            bool setPotentialCost(std::string filename);
            void setPathLengthWeight(double weight) {pathLengthWeight_ = weight;}
            bool isSymmetric() {return false;}
            virtual ob::Cost stateCost(const ob::State *s) const;
            virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;

        private:
            std::vector<mt::Point3> point;
            std::vector<Segment> segment;
            std::vector<std::vector<mt::Point3> > pointSet;
            std::vector<std::vector<Segment> > segmentSet;
            std::vector<double> pointRadius;
            std::vector<double> segmentRadius;
            std::vector<std::vector<double> > pointSetRadius;
            std::vector<std::vector<double> > segmentSetRadius;
            std::vector<std::pair<double,double> > pointCost;
            std::vector<std::pair<double,double> > segmentCost;
            std::vector<std::pair<double,double> > pointSetCost;
            std::vector<std::pair<double,double> > segmentSetCost;
            std::vector<RobotSegment> robotSegments;
            omplPlanner *pl;
        };

        class myICOptimizationObjective : public ob::StateCostIntegralObjective {
        public:
            myICOptimizationObjective(const ob::SpaceInformationPtr &si, omplPlanner *p,
                                      double kP = 1., double kI = 1., double kD = 1.);
            bool setPotentialCost(std::string filename);
            void setKP(double kP) {kP_ = kP;}
            void setKI(double kI) {kI_ = kI;}
            void setKD(double kD) {kD_ = kD;}
            double getKP() {return kP_;}
            double getKI() {return kI_;}
            double getKD() {return kD_;}
            bool isSymmetric() {return ob::OptimizationObjective::isSymmetric();}
            virtual ob::Cost stateCost(const ob::State *s) const;
            virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;

        private:
            std::vector<mt::Point3> point;
            std::vector<Segment> segment;
            std::vector<std::vector<mt::Point3> > pointSet;
            std::vector<std::vector<Segment> > segmentSet;
            std::vector<double> pointRadius;
            std::vector<double> segmentRadius;
            std::vector<std::vector<double> > pointSetRadius;
            std::vector<std::vector<double> > segmentSetRadius;
            std::vector<std::pair<double,double> > pointCost;
            std::vector<std::pair<double,double> > segmentCost;
            std::vector<std::pair<double,double> > pointSetCost;
            std::vector<std::pair<double,double> > segmentSetCost;
            omplPlanner *pl;
            double kP_;
            double kI_;
            double kD_;

            ob::Cost costPID(ob::Cost c1, ob::Cost c2, double d12) const {
                double cP(d12);
                double cI(0.5*(c1.value()+c2.value())*d12);
                double cD(fabs(c2.value()-c1.value()));

                return ob::Cost(kP_*cP+kI_*cI+kD_*cD);
            }
        };
    }
    /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplMyOBJECTIVE_H

