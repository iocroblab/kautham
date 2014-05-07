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

 

#if !defined(_LOCALPLANNER_H)
#define _LOCALPLANNER_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>


namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */
 namespace IOC{

	class LocalPlanner {
	  public:
      LocalPlanner(SPACETYPE type, Sample *init, Sample *goal, WorkSpace *ws, KthReal st );
      virtual ~LocalPlanner();
      inline Sample*    initSamp(){return _init;}
      inline Sample*    goalSamp(){return _goal;}
      inline string     getIDName(){return _idName;}
      inline void       setInitSamp(Sample* init){_init = init;}
      inline void       setGoalSamp(Sample* goal){_goal = goal;}
      inline WorkSpace* wkSpace(){return _wkSpace;}
      inline void       setWorkSpace(WorkSpace* ws){_wkSpace = ws;}
      inline KthReal    stepSize(){return _step;}
      inline void       setStepSize(KthReal ss){ _step = ss;}
      virtual bool      canConect()=0;
      virtual KthReal   distance(Sample* from, Sample* to)=0;
	    //inline string getName(){return _name;};
	  private:
      LocalPlanner();
    protected:
      std::string       _idName;
      KthReal           _step;
      WorkSpace*        _wkSpace;
      Sample*           _init;
      Sample*           _goal;
      SPACETYPE         _spType;
	    //string _name;
    };
 }
 /** @}   end of Doxygen module "libPlanner */
}

#endif  //_PLANNER_H

