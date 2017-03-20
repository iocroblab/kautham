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


#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/mygridplanner.h>


namespace Kautham {

  namespace IOC{
	//! Constructor
    MyGridPlanner::MyGridPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
              gridPlanner(stype, init, goal, samples, ws)
	{
		//set intial values
		_firstParameter = 10;
		_secondParameter = 0.5;
		_thirdParameter = 0.1;
	  
		_guiName = "My Grid Planner";
		addParameter("First parameter", _firstParameter);
		addParameter("Second parameter", _secondParameter);
		addParameter("Third parameter", _thirdParameter);
    }

	//! void destructor
	MyGridPlanner::~MyGridPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool MyGridPlanner::setParameters(){
      try{
        HASH_S_K::iterator it  = _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

		char *str = new char[20];
        for(unsigned i=0; i<_wkSpace->getNumRobControls();i++)
		{
			sprintf(str,"Discr. Steps %d",i);
			it = _parameters.find(str);
			if(it != _parameters.end())
			{
				setStepsDiscretization(it->second,i);
			}
			else
				return false;
		}


      }catch(...){
        return false;
      }
      return true;
    }

	
	  
	//! function to find a solution path
		bool MyGridPlanner::trySolve()
		{
			cout << "MyGridPlanner::trySolve - now a simple rectilinear connection without collision checking..."<<endl<<flush;


			
			_path.clear();
			clearSimulationPath();
            _path.push_back(_init.at(0));
            _path.push_back(_goal.at(0));
			_solved = true;
			return _solved;
			
		}
      }
}


