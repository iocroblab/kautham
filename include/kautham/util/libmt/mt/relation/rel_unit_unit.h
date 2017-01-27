/***************************************************************************
 *   Copyright (C) 2006 by Adolfo Rodriguez                                *
 *   adolfo.rodriguez@upc.edu                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/////////////////////////////// PREPROCESSOR DIRECTIVES //////////////////////

// HEADER GUARD
#ifndef MT_REL_UNIT_UNIT_H
#define MT_REL_UNIT_UNIT_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/unit3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between two unit vectors.

class RelUnitUnit : public RelBase,
                    public RelBaseAng
{
public:

// LIFECYCLE

  RelUnitUnit(const Unit3& u1,
              const Unit3& u2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&      os,
                         const RelUnitUnit& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelUnitUnit::RelUnitUnit(const Unit3& u1,
                                const Unit3& u2) :

                                RelBaseAng(angle(u1, u2))
{
  m_type = angleType(m_angle);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&      os,
                                const RelUnitUnit& rel)
{
  return os << "angle: "    << radToDeg(rel.getAngle());
}


} // mt

#endif // MT_REL_UNIT_UNIT_H
