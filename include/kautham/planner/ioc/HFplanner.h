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


#if !defined(_HFPLANNER_H)
#define _HFPLANNER_H


#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/planner.h>
#include <kautham/planner/ioc/gridplanner.h>

using namespace std;

namespace Kautham {
/** \addtogroup GridPlanners
 *  @{
 */
  namespace IOC{

    class HFPlanner:public gridPlanner {
	    public:
        HFPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
        ~HFPlanner();
        
		bool trySolve();
		bool setParameters();
		//Add public data and functions
		

		protected:
		//Add protected data and functions
			int _mainiter;
			int _hfiter;
			int _dirichlet;

		
	    private:
		//Add private data and functions	
		void computeHF(gridVertex  vgoal);

		
	  };
   }
  /** @}   end of Doxygen module "Planner */
}

#endif  //_HFPLANNER_H

