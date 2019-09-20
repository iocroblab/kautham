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

#include <kautham/planner/omplg/omplRRTConnectplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>



namespace Kautham {
  namespace omplplanner{

	//! Constructor
    omplRRTConnectPlanner::omplRRTConnectPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
	{
        _guiName = "ompl RRTConnect Planner";
        _idName = "omplRRTConnect";

        //create planner
        ob::PlannerPtr planner(new og::RRTConnect(si));

        //set planner parameters: range
        _Range=0.05;
        addParameter("Range", _Range);
        planner->as<og::RRTConnect>()->setRange(_Range);



std::cout<<"................................................."<<std::endl;
std::cout<<"................................................."<<std::endl;
std::cout<<"_Range: "<<_Range<<std::endl;
std::cout<<"_validSegmentCount: "<<_validSegmentCount<<std::endl;
std::cout<<"space->getLongestValidSegmentLength(): "<<space->getLongestValidSegmentLength()<<std::endl;
std::cout<<"( _validSegmentCount-1)*space->getLongestValidSegmentLength() "<<( _validSegmentCount-1)*space->getLongestValidSegmentLength()<<std::endl;
std::cout<<"space->getMaximumExtent(): "<<space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<_Range/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<10.0/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<space->getLongestValidSegmentFraction()<<std::endl;
std::cout<<"................................................."<<std::endl;
std::cout<<"................................................."<<std::endl;



        if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
            //space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());

            //space->setLongestValidSegmentFraction(10.0/_validSegmentCount/space->getMaximumExtent());

std::cout<<"***********************************************"<<std::endl;
std::cout<<"***********************************************"<<std::endl;
std::cout<<"***********************************************"<<std::endl;
std::cout<<"_Range: "<<_Range<<std::endl;
std::cout<<"_validSegmentCount: "<<_validSegmentCount<<std::endl;
std::cout<<"space->getLongestValidSegmentLength(): "<<space->getLongestValidSegmentLength()<<std::endl;
std::cout<<"space->getMaximumExtent(): "<<space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<_Range/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<10.0/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"***********************************************"<<std::endl;
std::cout<<"***********************************************"<<std::endl;
std::cout<<"***********************************************"<<std::endl;


            space->setup();
        }

        //set the planner
        ss->setPlanner(planner);
    }

	//! void destructor
    omplRRTConnectPlanner::~omplRRTConnectPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplRRTConnectPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
          HASH_S_K::iterator it = _parameters.find("Range");
          if(it != _parameters.end()){
            _Range = it->second;
            ss->getPlanner()->as<og::RRTConnect>()->setRange(_Range);
std::cout<<"++++++6666666666666666666666666666666++++++++++++++++++++++++"<<std::endl;
std::cout<<"_Range: "<<_Range<<std::endl;
std::cout<<"_validSegmentCount: "<<_validSegmentCount<<std::endl;
std::cout<<"space->getLongestValidSegmentLength(): "<<space->getLongestValidSegmentLength()<<std::endl;
std::cout<<"( _validSegmentCount-1)*space->getLongestValidSegmentLength() "<<( _validSegmentCount-1)*space->getLongestValidSegmentLength() <<" " <<std::endl;
            if (_Range <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {


std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
std::cout<<"_Range: "<<_Range<<std::endl;
std::cout<<"_validSegmentCount: "<<_validSegmentCount<<std::endl;
std::cout<<"space->getLongestValidSegmentLength(): "<<space->getLongestValidSegmentLength()<<std::endl;
std::cout<<"space->getMaximumExtent(): "<<space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<_Range/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"setLongestValidSegmentFraction: "<<10.0/_validSegmentCount/space->getMaximumExtent()<<std::endl;
std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;


                //space->setLongestValidSegmentFraction(_Range/_validSegmentCount/space->getMaximumExtent());
                space->setup();
            }
           }
        else
          return false;

      }catch(...){
        return false;
      }
      return true;
    }
  }
}

#endif // KAUTHAM_USE_OMPL

