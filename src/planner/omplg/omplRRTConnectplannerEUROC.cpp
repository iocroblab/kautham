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

 
#if defined(KAUTHAM_USE_OMPL)

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplRRTConnectplannerEUROC.h>
#include <kautham/planner/omplg/omplValidityChecker.h>



namespace Kautham {
  namespace omplplanner{

	//! Constructor
    omplRRTConnectPlannerEUROC::omplRRTConnectPlannerEUROC(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl RRTConnect Planner EUROC";
        _idName = "omplRRTConnectEUROC";

        //create planner
        ob::PlannerPtr planner(new og::RRTConnect(si));

        //set planner parameters: range
        _Range=0.05;
        addParameter("Range", _Range);
        planner->as<og::RRTConnect>()->setRange(_Range);
        if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
            space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
            space->setup();
        }

        //activate the filtering of samples done in validity check class
        _filtersamples = 1;
        addParameter("FilterSamples(0/1)", _filtersamples);

        //set the planner
        ss->setPlanner(planner);
    }

	//! void destructor
    omplRRTConnectPlannerEUROC::~omplRRTConnectPlannerEUROC(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplRRTConnectPlannerEUROC::setParameters(){

      omplPlanner::setParameters();
      try{
          HASH_S_K::iterator it = _parameters.find("Range");
          if(it != _parameters.end()){
              _Range = it->second;
              ss->getPlanner()->as<og::RRTConnect>()->setRange(_Range);
              if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                  space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
                  space->setup();
              }
          }
          else
              return false;

          it = _parameters.find("FilterSamples(0/1)");
          if(it != _parameters.end()){
              if(it->second == 0) _filtersamples = 0;
              else _filtersamples = 1;
          }
          else
              return false;

      }catch(...){
        return false;
      }
      return true;
    }

    //! trySolve is reiplemented to add the chance to try again without having the
    //! filtering of samples activated
    bool omplRRTConnectPlannerEUROC::trySolve()//reimplemented
    {
        //if filter eliminates init or goal, then do not activate the filter!!
        if(filtersample(_init.at(0)) || filtersample(_goal.at(0)))
        {
            //disable filtersample
            std::cout<<"Disabling filtering because either init or goal do not satisfy filter restrictions\n";
            _filtersamples = 0;
            //solve
            return omplPlanner::trySolve();
        }
        else
        {
            //do as programmed (with ot without filter)
            if(omplPlanner::trySolve())
                return true;
            else
            {
                //try again without the filtering activated
                if(_filtersamples==1)
                {
                    std::cout<<"Retrying trySolve without sample filtering\n";
                    //disable filtersample
                    _filtersamples = 0;
                    //try again
                    bool ret = omplPlanner::trySolve();
                    //restore things
                    _filtersamples = 1;

                    return ret;
                } else {
                    return false;
                }
            }
        }
    }

    bool omplRRTConnectPlannerEUROC::filtersample(Sample* smp)
    {
        //Filtering not activated
        if(_filtersamples == 0)
            return false;


        //If init and goal have J2 and J4 in the same halve ranges then
        //all samples shouuld also have this way
        if(_init.at(0)->getCoords()[3]>0.5 && _goal.at(0)->getCoords()[3]>0.5 &&
                _init.at(0)->getCoords()[5]<0.5 && _goal.at(0)->getCoords()[5]<0.5) {
            if (smp->getCoords()[3]<0.5 || smp->getCoords()[5]>0.5) return true;//do not match,then filter
        } else if(_init.at(0)->getCoords()[3]<0.5 && _goal.at(0)->getCoords()[3]<0.5 &&
                _init.at(0)->getCoords()[5]>0.5 && _goal.at(0)->getCoords()[5]>0.5) {
            if (smp->getCoords()[3]>0.5 || smp->getCoords()[5]<0.5) return true;//do not match,then filter
        }
        //else continue...

        //Filtering activated
        //std::cout<<"Filtering\n";

        //Filter those samples that make the LWR to pass near the home position
        /*
        int d=wkSpace()->getNumRobControls();
        double th2 = abs(smp->getCoords()[2] - 0.5);
        double th4 = abs(smp->getCoords()[4] - 0.5);
        if(th2<0.15 && th4<0.15){
            std::cout<<"NOT accepted sample. Robot in too vertical position "<<std::endl;
            return true; //sample must be rejected
        }
        */


        wkSpace()->moveRobotsTo(smp);
        mt::Transform& t6 = wkSpace()->getRobot(0)->getLinkTransform(6);
        mt::Transform& t7 = wkSpace()->getRobot(0)->getLinkTransform(7);


        /*
        //Filter those sample that take the TCP too high
        double zposT7 = t7.getTranslation()[2];
        //std::cout<<"zpos TCP: "<<zposT7<<std::endl;
        if(zposT7>900)
        {
            //std::cout<<"NOT accepted sample. TCP too high "<<zposT7<<std::endl;
            return true;//sample must be rejected
        }
        */


        //Filter those samples that make the axis of the 6 and 7 joints form a plane
        //parallel to the floor. In these configurations the joint 6 has many chances to
        //exceed the maximum torque when the robot has a grasped object.
        mt::Vector3& zvectorT6 = t6.getRotation().getMatrix().transpose().at(2);
        mt::Vector3& zvectorT7 = t7.getRotation().getMatrix().transpose().at(2);

        //std::cout<<"z6 = ("<<zvectorT6[0]<<","<<zvectorT6[1]<<","<<zvectorT6[2]<<std::endl;
        //std::cout<<"z7 = ("<<zvectorT7[0]<<","<<zvectorT7[1]<<","<<zvectorT7[2]<<std::endl;

        //crossproduct
        mt::Vector3 zzcross =  zvectorT6.cross(zvectorT7);
        //std::cout<<"zzcross = ("<<zzcross[0]<<","<<zzcross[1]<<","<<zzcross[2]<<std::endl;

        Vector3 zworld(0.0,0.0,1.0);
        double cosangle = zzcross.dot(zworld);
        if(fabs(cosangle) > cos(mt::PI/6))
        {
            //std::cout<<"NOT accepted sample. Angle between (z6xz7) and zworld is "<<(180/mt::PI)*acos(fabs(cosangle))<<std::endl;
            return true;//sample must be rejected
        }
        else{
            //std::cout<<"Accepted sample. zTCP = "<<zposT7<<" Angle between (z6xz7) and zworld = "<<(180/mt::PI)*acos(fabs(cosangle))<<std::endl;
            return false;//sample must be accepted
        }
    }


  }
}

#endif // KAUTHAM_USE_OMPL

