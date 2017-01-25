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

 

#if !defined(_LINEARLOCALPLANNER_H)
#define _LINEARLOCALPLANNER_H

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/localplanner.h>


namespace Kautham {
/** \addtogroup Planner
 *  @{
 */
namespace IOC{
class LinearLocalPlanner:public LocalPlanner {
    public:
      LinearLocalPlanner(SPACETYPE stype, Sample *init, Sample *goal, WorkSpace *ws, KthReal st );
      void setMethod(bool vandercorput = true);
      bool canConect();
      KthReal distance(Sample* from, Sample* to);
    private:
      LinearLocalPlanner();
      bool vanderMethod;
	};
}
/** @}   end of Doxygen module "Planner */
}

#endif  //_LINEARLOCALPLANNER_H
