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
#ifndef MT_REL_BASE_ANG_H
#define MT_REL_BASE_ANG_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/direction_type.h>
#include <mt/scalar.h>
#include <mt/relation/rel_type.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Base class for relations between geometric elements with angular
/// component.
///
/// The RelBaseAng class provides the methods and members for relations
/// between geometric elements that feature an angular component.

class RelBaseAng
{
public:

// LIFECYCLE

  RelBaseAng(const Scalar& s = 0.0);


// ACCESS

  Scalar getAngle() const;


protected:

// MEMBERS

  Scalar m_angle;


// OPERATIONS
  RelType angleType(const Scalar& ang,
                    const DirectionType& type = DIRECTED);
};

/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelBaseAng::RelBaseAng(const Scalar& s) : m_angle(s) {}


// ACCESS

inline Scalar RelBaseAng::getAngle() const
{
  return m_angle;
}


// PROTECTED OPERATIONS

inline RelType RelBaseAng::angleType(const Scalar& ang,
                                     const DirectionType& type)
{
  if (ang == 0.0)
  {
    if (type == DIRECTED)
    {
      return PARALLEL;
    }
    else
    {
      return SAME_DIRECTION;
    }
  }
  else if (ang == PI)
  {
    if (type == DIRECTED)
    {
      return ANTIPARALLEL;
    }
    else
    {
      return SAME_DIRECTION;
    }
  }
  else if (ang == HALF_PI)
  {
    return RelType(ANGLE, PERPENDICULAR);
  }
  else
  {
    return ANGLE;
  }
}


} // mt


#endif // MT_REL_BASE_ANG_H
