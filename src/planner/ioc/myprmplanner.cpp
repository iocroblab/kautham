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

/* Author: Alexander Perez, Jan Rosell */
 

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/localplanner.h>
#include <kautham/planner/ioc/myprmplanner.h>


namespace Kautham {
  namespace IOC{
	//! Constructor
    MyPRMPlanner::MyPRMPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, WorkSpace *ws):
              PRMPlanner(stype, init, goal, samples, sampler, ws)
	{
		//set intial values
		_firstParameter = 10;
		_secondParameter = 0.5;
		_thirdParameter = 0.1;

		//set intial values from parent class data
		_speedFactor = 1;
        _solved = false;
	  
        _guiName = "My PRM Planner";
		addParameter("Speed Factor", _speedFactor);
		addParameter("First parameter", _firstParameter);
		addParameter("Second parameter", _secondParameter);
		addParameter("Third parameter", _thirdParameter);

    }

	//! void destructor
	MyPRMPlanner::~MyPRMPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool MyPRMPlanner::setParameters(){
      try{
        HASH_S_K::iterator it =  _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

		it = _parameters.find("First parameter");
        if(it != _parameters.end())
			_firstParameter = it->second;
        else
          return false;

        it = _parameters.find("Second parameter");
        if(it != _parameters.end())
          _secondParameter = it->second;
        else
          return false;

        it = _parameters.find("Third parameter");
        if(it != _parameters.end())
          _thirdParameter = it->second;
        else
          return false;


      }catch(...){
        return false;
      }
      return true;
    }

	
  	
	//! function to find a solution path
		bool MyPRMPlanner::trySolve()
		{
			cout << "MyPRMPlanner::trySolve - now a call to PRMPlanner::trySolve()..."<<endl<<flush;
			cout << "A bettter alternative is expected to be implemented here..."<<endl<<flush;

			return PRMPlanner::trySolve();

			/*
			The available resorces to implement your planner are:
			1) A sampler to obtain samples:
					Sample *smp;
					smp = _sampler->nextSample();
			   or a way to determine new sample at a given configuration, e.g.:
					Sample *smp = new Sample();
					KthReal* coords = new KthReal[_wkSpace->getDimension()];
					for(int k = 0; k < _wkSpace->getDimension(); k++) coords[k] = 0.0;
					smp->setCoords(coords);

			2) A collision-checker to check for collision at a given sample smp:
					_wkSpace->collisionCheck(smp)

			3) A local planner to connect two samples
					_locPlanner->canConect();

			The solution must be specified as a sequence of samples (vector _path)
		  	*/

		}
      }
}


