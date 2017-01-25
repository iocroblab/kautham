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
#include <kautham/planner/omplg/omplMyOptimizationObjective.h>
#include <math.h>
#include <pugixml.hpp>


#define TOL 1e-8


namespace Kautham {
    namespace omplplanner {
        double distance(const Point3 p, const Segment s) {
            Vector3 v = s.p1 - s.p0;
            Vector3 w = p - s.p0;

            double c1 = dot(w,v);
            if (c1 <= 0.) return length(p-s.p0);

            double c2 = dot(v,v);
            if (c2 <= c1) return length(p-s.p1);

            Point3 pb = s.p0 + (c1/c2)*v;
            return length(p-pb);
        }


        double distance(Segment s1, Segment s2) {
            Vector3 u = s1.p1 - s1.p0;
            Vector3 v = s2.p1 - s2.p0;
            Vector3 w = s1.p0 - s2.p0;
            double a = dot(u,u);// always >= 0
            double b = dot(u,v);
            double c = dot(v,v);// always >= 0
            double d = dot(u,w);
            double e = dot(v,w);
            double D = a*c - b*b;// always >= 0
            double sc, sN, sD = D;// sc = sN / sD, default sD = D >= 0
            double tc, tN, tD = D;// tc = tN / tD, default tD = D >= 0

            // compute the line parameters of the two closest points
            if (D < TOL) {
                // the lines are almost parallel
                // force using point p0 on segment s1
                // to prevent possible division by 0.0 later
                sN = 0.;
                sD = 1.;
                tN = e;
                tD = c;
            } else {
                // get the closest points on the infinite lines
                sN = b*e - c*d;
                tN = a*e - b*d;
                if (sN < 0.) {
                    // sc < 0 => the s=0 edge is visible
                    sN = 0.;
                    tN = e;
                    tD = c;
                } else if (sN > sD) {
                    // sc > 1  => the s=1 edge is visible
                    sN = sD;
                    tN = e + b;
                    tD = c;
                }
            }

            if (tN < 0.) {
                // tc < 0 => the t=0 edge is visible
                tN = 0.;
                // recompute sc for this edge
                if (-d < 0.) {
                    sN = 0.;
                } else if (-d > a) {
                    sN = sD;
                } else {
                    sN = -d;
                    sD = a;
                }
            } else if (tN > tD) {
                // tc > 1  => the t=1 edge is visible
                tN = tD;
                // recompute sc for this edge
                if ((-d + b) < 0.)
                    sN = 0.;
                else if ((-d + b) > a)
                    sN = sD;
                else {
                    sN = -d + b;
                    sD = a;
                }
            }
            // finally do the division to get sc and tc
            sc = (fabs(sN) < TOL ? 0. : sN/sD);
            tc = (fabs(tN) < TOL ? 0. : tN/tD);

            // get the difference of the two closest points
            return length(w + (sc * u) - (tc * v));// =  S1(sc) - S2(tc)
        }


        mt::Point3 getPoint(std::string str) {
            mt::Point3 point;
            std::istringstream strs(str);
            int chars_to_read = strs.str().size();
            unsigned int num_read = 0;
            while (chars_to_read > 0 && num_read < 3) {
                getline(strs,str,' ');
                if (str.size() > 0) {
                    point[num_read] = atof(str.c_str());
                    num_read++;
                }
                chars_to_read -= str.size() + 1;
            }
            if (num_read != 3 || chars_to_read != -1) throw;

            return point;
        }


        myMWOptimizationObjective::myMWOptimizationObjective(const ob::SpaceInformationPtr &si,
                                                             omplPlanner *p,
                                                             double pathLengthWeight):
            ob::MechanicalWorkOptimizationObjective(si,pathLengthWeight),pl(p) {

        }


        bool myMWOptimizationObjective::setPotentialCost(std::string filename) {
            try {
                pugi::xml_document doc;
                pugi::xml_parse_result result = doc.load_file(filename.c_str());
                if (result) {
                    point.clear();
                    pointRadius.clear();
                    pointCost.clear();

                    segment.clear();
                    segmentRadius.clear();
                    segmentCost.clear();

                    pointSet.clear();
                    pointSetRadius.clear();
                    pointSetCost.clear();

                    segmentSet.clear();
                    segmentSetRadius.clear();
                    segmentSetCost.clear();

                    for (pugi::xml_node_iterator it = doc.child("Potential").begin();
                         it != doc.child("Potential").end(); ++it) {
                        if (std::string(it->name()) == "Point") {
                            pointCost.push_back(std::pair<double,double>
                                                (it->attribute("repulse").as_double(0.),
                                                 it->attribute("diffusion").as_double(0.)));
                            point.push_back(getPoint(it->attribute("p").as_string("0 0 0")));
                            pointRadius.push_back(it->attribute("radius").as_double(0.));
                        } else if (std::string(it->name()) == "Segment") {
                            segmentCost.push_back(std::pair<double,double>
                                                  (it->attribute("repulse").as_double(0.),
                                                   it->attribute("diffusion").as_double(0.)));
                            segment.push_back(Segment(getPoint(it->attribute("p0").as_string("0 0 0")),
                                                      getPoint(it->attribute("p1").as_string("0 0 0"))));
                            segmentRadius.push_back(it->attribute("radius").as_double(0.));
                        } else if (std::string(it->name()) == "PointSet") {
                            pointSetCost.push_back(std::pair<double,double>
                                                   (it->attribute("repulse").as_double(0.),
                                                    it->attribute("diffusion").as_double(0.)));
                            std::vector<mt::Point3> points;
                            std::vector<double> radii;
                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                points.push_back(getPoint(it2->attribute("p").as_string("0 0 0")));
                                radii.push_back(it2->attribute("radius").as_double(0.));
                            }
                            pointSet.push_back(points);
                            pointSetRadius.push_back(radii);
                        } else if (std::string(it->name()) == "SegmentSet") {
                            segmentSetCost.push_back(std::pair<double,double>
                                                     (it->attribute("repulse").as_double(0.),
                                                      it->attribute("diffusion").as_double(0.)));
                            std::vector<Segment> segments;
                            std::vector<double> radii;
                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                segments.push_back(Segment(getPoint(it2->attribute("p0").as_string("0 0 0")),
                                                           getPoint(it2->attribute("p1").as_string("0 0 0"))));
                                radii.push_back(it2->attribute("radius").as_double(0.));
                            }
                            segmentSet.push_back(segments);
                            segmentSetRadius.push_back(radii);
                        }
                    }

                    robotSegments.clear();
                    Robot *robot;
                    Link *link;
                    for (pugi::xml_node_iterator itRobot = doc.child("Robots").begin();
                         itRobot != doc.child("Robots").end(); ++itRobot) {
                        robot = pl->wkSpace()->getRobot(itRobot->attribute("name").as_string());
                        if (robot) {
                            for (pugi::xml_node_iterator itLink = itRobot->begin();
                                 itLink != itRobot->end(); ++itLink) {
                                link = robot->getLink(itLink->attribute("name").as_string());
                                if (link) {
                                    for (pugi::xml_node_iterator it = itLink->begin();
                                         it != itLink->end(); ++it) {
                                        /*if (std::string(it->name()) == "Point") {
                                            pointCost.push_back(std::pair<double,double>
                                                                (it->attribute("repulse").as_double(0.),
                                                                 it->attribute("diffusion").as_double(0.)));
                                            point.push_back(getPoint(it->attribute("p").as_string("0 0 0")));
                                            pointRadius.push_back(it->attribute("radius").as_double(0.));
                                        } else*/ if (std::string(it->name()) == "Segment") {
                                            RobotSegment robotSegment;
                                            robotSegment.robot = robot;
                                            robotSegment.link = link;
                                            robotSegment.p0 = getPoint(it->attribute("p0").as_string("0 0 0"));
                                            robotSegment.p1 = getPoint(it->attribute("p1").as_string("0 0 0"));
                                            robotSegment.radius = it->attribute("radius").as_double(0.);
                                            robotSegments.push_back(robotSegment);
                                        } /*else if (std::string(it->name()) == "PointSet") {
                                            pointSetCost.push_back(std::pair<double,double>
                                                                   (it->attribute("repulse").as_double(0.),
                                                                    it->attribute("diffusion").as_double(0.)));
                                            std::vector<mt::Point3> points;
                                            std::vector<double> radii;
                                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                                points.push_back(getPoint(it2->attribute("p").as_string("0 0 0")));
                                                radii.push_back(it2->attribute("radius").as_double(0.));
                                            }
                                            pointSet.push_back(points);
                                            pointSetRadius.push_back(radii);
                                        } else if (std::string(it->name()) == "SegmentSet") {
                                            segmentSetCost.push_back(std::pair<double,double>
                                                                     (it->attribute("repulse").as_double(0.),
                                                                      it->attribute("diffusion").as_double(0.)));
                                            std::vector<Segment> segments;
                                            std::vector<double> radii;
                                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                                segments.push_back(Segment(getPoint(it2->attribute("p0").as_string("0 0 0")),
                                                                           getPoint(it2->attribute("p1").as_string("0 0 0"))));
                                                radii.push_back(it2->attribute("radius").as_double(0.));
                                            }
                                            segmentSet.push_back(segments);
                                            segmentSetRadius.push_back(radii);
                                        }*/
                                    }
                                }
                            }
                        }
                    }

                    return true;
                } else {
                    std::cout << filename << " " << result.description() << std::endl;

                    return false;
                }
            } catch(...) {
                return false;
            }
        }


        ob::Cost myMWOptimizationObjective::stateCost(const ob::State *s) const {
            Sample *smp = new Sample(3);
            //copy the conf of the init smp. Needed to capture the home positions.
            smp->setMappedConf(pl->initSamp()->getMappedConf());
            pl->omplState2smp(s,smp);

            double sqDist, cost;
            double totalCost = 0.;

            if (robotSegments.size() == 0) {
                mt::Point3 p(smp->getMappedConf()[0].getSE3().getPos().at(0),
                        smp->getMappedConf()[0].getSE3().getPos().at(1),
                        smp->getMappedConf()[0].getSE3().getPos().at(2));
                //std::cout << "The state is: " << p << std::endl;



                //Points cost
                //std::cout << "Points:" << std::endl;
                for (unsigned int i = 0; i < point.size(); ++i) {
                    sqDist = pow(std::max(0.,Vector3(p-point.at(i)).length()-pointRadius.at(i)),2.);
                    cost = std::max(-pointCost.at(i).first,0.)+pointCost.at(i).first*exp(-pointCost.at(i).second*sqDist);
                    totalCost += cost;
                    //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                    //std::cout << "Cost " << i << " is " << cost << std::endl;
                }

                //Segments cost
                //std::cout << "Segments:" << std::endl;
                for (unsigned int i = 0; i < segment.size(); ++i) {
                    sqDist = pow(std::max(0.,distance(p,segment.at(i))-segmentRadius.at(i)),2.);
                    cost = std::max(-segmentCost.at(i).first,0.)+segmentCost.at(i).first*exp(-segmentCost.at(i).second*sqDist);
                    totalCost += cost;
                    //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                    //std::cout << "Cost " << i << " is " << cost << std::endl;
                }

                //Point sets cost
                //std::cout << "Point sets:" << std::endl;
                for (unsigned int i = 0; i < pointSet.size(); ++i) {
                    sqDist = DBL_MAX;
                    for (unsigned int j = 0; j < pointSet.at(i).size(); ++j) {
                        sqDist = std::min(sqDist,std::max(0.,Vector3(p-pointSet.at(i).at(j)).length()-pointSetRadius.at(i).at(j)));
                    }
                    sqDist *= sqDist;
                    cost = std::max(-pointSetCost.at(i).first,0.)+pointSetCost.at(i).first*exp(-pointSetCost.at(i).second*sqDist);
                    totalCost += cost;
                    //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                    //std::cout << "Cost " << i << " is " << cost << std::endl;
                }

                //Segment sets cost
                //std::cout << "Segement sets:" << std::endl;
                for (unsigned int i = 0; i < segmentSet.size(); ++i) {
                    sqDist = DBL_MAX;
                    for (unsigned int j = 0; j < segmentSet.at(i).size(); ++j) {
                        sqDist = std::min(sqDist,std::max(0.,distance(p,segmentSet.at(i).at(j))-segmentSetRadius.at(i).at(j)));
                    }
                    sqDist *= sqDist;
                    cost = std::max(-segmentSetCost.at(i).first,0.)+segmentSetCost.at(i).first*exp(-segmentSetCost.at(i).second*sqDist);
                    totalCost += cost;
                    //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                    //std::cout << "Cost " << i << " is " << cost << std::endl;
                }
            } else {
                pl->wkSpace()->moveRobotsTo(smp);
                /*double minDist = DBL_MAX;
                for (size_t i = 0; i < robotSegments.size(); ++i) {
                    Segment segment = robotSegments.at(i).segment();
                    for (size_t j = 0; j < pointSet.at(0).size(); ++j) {
                        minDist = std::min(minDist,std::max(0.,distance(pointSet.at(0).at(j),segment)-pointSetRadius.at(0).at(j)-robotSegments.at(i).radius));
                    }
                }
                sqDist = minDist*minDist;

                totalCost = std::max(-pointSetCost.at(0).first,0.)+pointSetCost.at(0).first*exp(-pointSetCost.at(0).second*sqDist);*/

                for (size_t k = 0; k < robotSegments.size(); ++k) {
                    Segment s = robotSegments.at(k).segment();
                    double r = robotSegments.at(k).radius;

                    //Points cost
                    //std::cout << "Points:" << std::endl;
                    for (unsigned int i = 0; i < point.size(); ++i) {
                        sqDist = pow(std::max(0.,distance(point.at(i),s)-pointRadius.at(i)-r),2.);
                        cost = std::max(-pointCost.at(i).first,0.)+pointCost.at(i).first*exp(-pointCost.at(i).second*sqDist);
                        totalCost += cost;
                        //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                        //std::cout << "Cost " << i << " is " << cost << std::endl;
                    }

                    //Segments cost
                    //std::cout << "Segments:" << std::endl;
                    for (unsigned int i = 0; i < segment.size(); ++i) {
                        sqDist = pow(std::max(0.,distance(segment.at(i),s)-segmentRadius.at(i)-r),2.);
                        cost = std::max(-segmentCost.at(i).first,0.)+segmentCost.at(i).first*exp(-segmentCost.at(i).second*sqDist);
                        totalCost += cost;
                        //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                        //std::cout << "Cost " << i << " is " << cost << std::endl;
                    }

                    //Point sets cost
                    //std::cout << "Point sets:" << std::endl;
                    for (unsigned int i = 0; i < pointSet.size(); ++i) {
                        sqDist = DBL_MAX;
                        for (unsigned int j = 0; j < pointSet.at(i).size(); ++j) {
                            sqDist = std::min(sqDist,std::max(0.,distance(pointSet.at(i).at(j),s)-pointSetRadius.at(i).at(j)-r));
                        }
                        sqDist *= sqDist;
                        cost = std::max(-pointSetCost.at(i).first,0.)+pointSetCost.at(i).first*exp(-pointSetCost.at(i).second*sqDist);
                        totalCost += cost;
                        //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                        //std::cout << "Cost " << i << " is " << cost << std::endl;
                    }

                    //Segment sets cost
                    //std::cout << "Segement sets:" << std::endl;
                    for (unsigned int i = 0; i < segmentSet.size(); ++i) {
                        sqDist = DBL_MAX;
                        for (unsigned int j = 0; j < segmentSet.at(i).size(); ++j) {
                            sqDist = std::min(sqDist,std::max(0.,distance(segmentSet.at(i).at(j),s)-segmentSetRadius.at(i).at(j)-r));
                        }
                        sqDist *= sqDist;
                        cost = std::max(-segmentSetCost.at(i).first,0.)+segmentSetCost.at(i).first*exp(-segmentSetCost.at(i).second*sqDist);
                        totalCost += cost;
                        //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                        //std::cout << "Cost " << i << " is " << cost << std::endl;
                    }

                    //Other robot segments
                    for (size_t l = k+1; l < robotSegments.size(); ++l) {
                        if (robotSegments.at(k).robot != robotSegments.at(l).robot) {
                            sqDist = pow(std::max(0.,distance(robotSegments.at(l).segment(),s)-robotSegments.at(l).radius-r),2.);
                            cost = std::max(-segmentCost.at(0).first,0.)+segmentCost.at(0).first*exp(-segmentCost.at(0).second*sqDist);
                            totalCost += cost;
                            //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                            //std::cout << "Cost " << i << " is " << cost << std::endl;
                        }
                    }
                }
            }


            delete smp;
            //std::cout << "Totalcost is: " << totalcost << std::endl;
            return ob::Cost(totalCost);
        }


        ob::Cost myMWOptimizationObjective::motionCost(const ompl::base::State *s1,
                                                       const ompl::base::State *s2) const {
            return ob::Cost(std::fabs(stateCost(s2).value()-stateCost(s1).value()) +
                            pathLengthWeight_*si_->distance(s1,s2));
        }


        myICOptimizationObjective::myICOptimizationObjective(const ob::SpaceInformationPtr &si,
                                                             omplPlanner *p,
                                                             double kP, double kI, double kD) :
            ob::StateCostIntegralObjective(si,true),pl(p),kP_(kP),kI_(kI),kD_(kD) {

        }


        bool myICOptimizationObjective::setPotentialCost(std::string filename) {
            try {
                pugi::xml_document doc;
                pugi::xml_parse_result result = doc.load_file(filename.c_str());
                if (result) {
                    point.clear();
                    pointRadius.clear();
                    pointCost.clear();

                    segment.clear();
                    segmentRadius.clear();
                    segmentCost.clear();

                    pointSet.clear();
                    pointSetRadius.clear();
                    pointSetCost.clear();

                    segmentSet.clear();
                    segmentSetRadius.clear();
                    segmentSetCost.clear();

                    for (pugi::xml_node_iterator it = doc.child("Potential").begin();
                         it != doc.child("Potential").end(); ++it) {
                        if (std::string(it->name()) == "Point") {
                            pointCost.push_back(std::pair<double,double>
                                                (it->attribute("repulse").as_double(0.),
                                                 it->attribute("diffusion").as_double(0.)));
                            point.push_back(getPoint(it->attribute("p").as_string("0 0 0")));
                            pointRadius.push_back(it->attribute("radius").as_double(0.));
                        } else if (std::string(it->name()) == "Segment") {
                            segmentCost.push_back(std::pair<double,double>
                                                  (it->attribute("repulse").as_double(0.),
                                                   it->attribute("diffusion").as_double(0.)));
                            segment.push_back(Segment(getPoint(it->attribute("p0").as_string("0 0 0")),
                                                      getPoint(it->attribute("p1").as_string("0 0 0"))));
                            segmentRadius.push_back(it->attribute("radius").as_double(0.));
                        } else if (std::string(it->name()) == "PointSet") {
                            pointSetCost.push_back(std::pair<double,double>
                                                   (it->attribute("repulse").as_double(0.),
                                                    it->attribute("diffusion").as_double(0.)));
                            std::vector<mt::Point3> points;
                            std::vector<double> radii;
                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                points.push_back(getPoint(it2->attribute("p").as_string("0 0 0")));
                                radii.push_back(it2->attribute("radius").as_double(0.));
                            }
                            pointSet.push_back(points);
                            pointSetRadius.push_back(radii);
                        } else if (std::string(it->name()) == "SegmentSet") {
                            segmentSetCost.push_back(std::pair<double,double>
                                                     (it->attribute("repulse").as_double(0.),
                                                      it->attribute("diffusion").as_double(0.)));
                            std::vector<Segment> segments;
                            std::vector<double> radii;
                            for (pugi::xml_node_iterator it2 = it->begin(); it2 != it->end(); ++it2) {
                                segments.push_back(Segment(getPoint(it2->attribute("p0").as_string("0 0 0")),
                                                           getPoint(it2->attribute("p1").as_string("0 0 0"))));
                                radii.push_back(it2->attribute("radius").as_double(0.));
                            }
                            segmentSet.push_back(segments);
                            segmentSetRadius.push_back(radii);
                        }
                    }

                    return true;
                } else {
                    std::cout << filename << " " << result.description() << std::endl;

                    return false;
                }
            } catch(...) {
                return false;
            }
        }


        ob::Cost myICOptimizationObjective::stateCost(const ob::State *s) const {
            Sample *smp = new Sample(3);
            //copy the conf of the init smp. Needed to capture the home positions.
            smp->setMappedConf(pl->initSamp()->getMappedConf());
            pl->omplState2smp(s,smp);
            mt::Point3 p(smp->getMappedConf()[0].getSE3().getPos().at(0),
                    smp->getMappedConf()[0].getSE3().getPos().at(1),
                    smp->getMappedConf()[0].getSE3().getPos().at(2));
            //std::cout << "The state is: " << p << std::endl;
            delete smp;

            double sqDist, cost;
            double totalCost = 0.;

            //Points cost
            //std::cout << "Points:" << std::endl;
            for (unsigned int i = 0; i < point.size(); ++i) {
                sqDist = pow(std::max(0.,Vector3(p-point.at(i)).length()-pointRadius.at(i)),2.);
                cost = std::max(-pointCost.at(i).first,0.)+pointCost.at(i).first*exp(-pointCost.at(i).second*sqDist);
                totalCost += cost;
                //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                //std::cout << "Cost " << i << " is " << cost << std::endl;
            }

            //Segments cost
            //std::cout << "Segments:" << std::endl;
            for (unsigned int i = 0; i < segment.size(); ++i) {
                sqDist = pow(std::max(0.,distance(p,segment.at(i))-segmentRadius.at(i)),2.);
                cost = std::max(-segmentCost.at(i).first,0.)+segmentCost.at(i).first*exp(-segmentCost.at(i).second*sqDist);
                totalCost += cost;
                //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                //std::cout << "Cost " << i << " is " << cost << std::endl;
            }

            //Point sets cost
            //std::cout << "Point sets:" << std::endl;
            for (unsigned int i = 0; i < pointSet.size(); ++i) {
                sqDist = DBL_MAX;
                for (unsigned int j = 0; j < pointSet.at(i).size(); ++j) {
                    sqDist = std::min(sqDist,std::max(0.,Vector3(p-pointSet.at(i).at(j)).length()-pointSetRadius.at(i).at(j)));
                }
                sqDist *= sqDist;
                cost = std::max(-pointSetCost.at(i).first,0.)+pointSetCost.at(i).first*exp(-pointSetCost.at(i).second*sqDist);
                totalCost += cost;
                //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                //std::cout << "Cost " << i << " is " << cost << std::endl;
            }

            //Segment sets cost
            //std::cout << "Segement sets:" << std::endl;
            for (unsigned int i = 0; i < segmentSet.size(); ++i) {
                sqDist = DBL_MAX;
                for (unsigned int j = 0; j < segmentSet.at(i).size(); ++j) {
                    sqDist = std::min(sqDist,std::max(0.,distance(p,segmentSet.at(i).at(j))-segmentSetRadius.at(i).at(j)));
                }
                sqDist *= sqDist;
                cost = std::max(-segmentSetCost.at(i).first,0.)+segmentSetCost.at(i).first*exp(-segmentSetCost.at(i).second*sqDist);
                totalCost += cost;
                //std::cout << "Distance " << i << " is " << sqrt(sqDist) << std::endl;
                //std::cout << "Cost " << i << " is " << cost << std::endl;
            }


            //std::cout << "Totalcost is: " << totalcost << std::endl;
            return ob::Cost(totalCost);
        }


        ob::Cost myICOptimizationObjective::motionCost(const ob::State *s1, const ob::State *s2) const {
            ob::State *test1(si_->cloneState(s1));

            ob::Cost prevStateCost(stateCost(test1));
            ob::Cost nextStateCost;
            ob::Cost totalCost(identityCost());

            unsigned int nd = si_->getStateSpace()->validSegmentCount(s1,s2);
            if (nd > 1 && interpolateMotionCost_) {
                ob::State *test2(si_->allocState());

                for (unsigned int j = 1; j < nd; ++j) {
                    si_->getStateSpace()->interpolate(s1,s2,double(j)/double(nd),test2);
                    nextStateCost = stateCost(test2);
                    totalCost = combineCosts(totalCost,costPID(prevStateCost,nextStateCost,si_->distance(test1,test2)));
                    std::swap(test1,test2);
                    prevStateCost = nextStateCost;
                }

                si_->freeState(test2);
            }

            // Lastly, add s2
            totalCost = combineCosts(totalCost,costPID(prevStateCost,stateCost(s2),si_->distance(test1,s2)));

            si_->freeState(test1);

            return totalCost;
        }
    }
}

#endif // KAUTHAM_USE_OMPL
