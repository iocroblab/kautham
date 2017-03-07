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

/* Author: Nestor Garcia Hidalgo */


#ifndef OMPLFOSVFRRTPLANNER_H
#define OMPLFOSVFRRTPLANNER_H

#if defined(KAUTHAM_USE_OMPL)

#include "omplplanner.h"
#include <kautham/planner/omplg/synergy_tree.h>

namespace Kautham {
    /** \addtogroup GeometricPlanners
 *  @{
 */
    namespace omplplanner {
        class omplFOSVFRRTPlanner : public omplPlanner {
        public:
            omplFOSVFRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                             SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr,
                                const std::string &synergyTreeFilename);

            ~omplFOSVFRRTPlanner();

            virtual bool trySolve();//!< Overloaded trySolve function to include evaluation of final path cost

            bool setParameters();

        protected:
            SynergyTree *st_;
        };
    }
    /** @}   end of Doxygen module */
}

#endif // KAUTHAM_USE_OMPL
#endif // OMPLFOSVFRRTPLANNER_H
