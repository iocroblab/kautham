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

#if !defined(_omplVALIDITYCHECKER_H)
#define _omplVALIDITYCHECKER_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <kautham/planner/planner.h>

namespace ob = ompl::base;

using namespace std;


namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */
namespace omplplanner{

class ValidityChecker : public ob::StateValidityChecker {
private:
    Planner *theplanner;
    ob::SpaceInformationPtr thesi;

public:
    //! Creator
    ValidityChecker(const ob::SpaceInformationPtr& si, Planner *p) :
        ob::StateValidityChecker(si),theplanner(p),thesi(si) {}

    //! isValid returns whether the given state's position overlaps the obstacles
    bool isValid(const ob::State* state) const;

    //! clearance returns the distance from the given state's position to the obstacles
    double clearance(const ob::State* state) const;

    unsigned int getCollisionChecks() {return theplanner->wkSpace()->getCollCheckCounter();}

    void resetCollisionChecks() {theplanner->wkSpace()->resetCollCheckCounter();}
};
}
/** @}   end of Doxygen module */
}


#endif // KAUTHAM_USE_OMPL
#endif  //_omplVALIDITYCHECKER_H

