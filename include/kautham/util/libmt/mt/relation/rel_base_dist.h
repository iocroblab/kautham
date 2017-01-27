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
#ifndef MT_REL_BASE_DIST_H
#define MT_REL_BASE_DIST_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/scalar.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Base class for relations between geometric elements with distance
/// component.
///
/// The RelBaseDist class provides the methods and members for relations
/// between geometric elements that feature a distance component.

class RelBaseDist
{
public:

// LIFECYCLE

  RelBaseDist(const Scalar& s = 0.0);

// ACCESS

  Scalar getDistance() const;


protected:

// MEMBERS

  Scalar m_dist;
};

/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelBaseDist::RelBaseDist(const Scalar& s) : m_dist(s) {}


// ACCESS

inline Scalar RelBaseDist::getDistance() const
{
  return m_dist;
}


} // mt

#endif // MT_REL_BASE_DIST_H
