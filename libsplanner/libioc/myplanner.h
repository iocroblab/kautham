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


#if !defined(_MYPLANNER_H)
#define _MYPLANNER_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include "localplanner.h"
#include "iocplanner.h"

using namespace std;

namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */
  namespace IOC{
    class MyPlanner:public iocPlanner {
	    public:
        MyPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
          WorkSpace *ws);
        ~MyPlanner();
        
		bool trySolve();
		bool setParameters();
		//Add public data and functions

		protected:
		//Add protected data and functions
		int _firstParameter;
		double _secondParameter;
		double _thirdParameter;

	    private:
		//Add private data and functions
	  };
  }
  /** @}   end of Doxygen module "libPlanner */
}

#endif  //_MYPLANNER_H

