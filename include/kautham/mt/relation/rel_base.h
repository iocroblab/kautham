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
#ifndef MT_REL_BASE_H
#define MT_REL_BASE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <kautham/mt/relation/rel_type.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Base class for relations between geometric elements.
///
/// The RelBase class provides the members and methods for expressing the
/// relation between geometric elements in qualitative terms. See the
/// of the rel_type.h header for more details.
///
/// Given the distance and/or angle between the elements
/// it computes a #RelType value that represents the relation. Example
/// values are "PARALLEL, COINCIDENT", "PERPENDICULAR", and
/// "SAME_DIRECTION, DISTANCE"

class RelBase
{
public:

// ACCESS

  RelType getType() const;


protected:

// MEMBERS

  RelType m_type;

};


/////////////////////////////// INLINE METHODS ///////////////////////////////

// ACCESS

inline RelType RelBase::getType() const
{
  return m_type;
}

} // mt

#endif  // MT_REL_BASE_H
