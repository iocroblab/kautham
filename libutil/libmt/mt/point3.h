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
#ifndef MT_POINT3_H
#define MT_POINT3_H

// MT LIBRARY HEADERS
#include <mt/exception.h>
#include <mt/scalar.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup elements

/// \ingroup elements
/// \brief Three dimensional point class.
///
/// This class differs from the Vector3 class in that the comparison criterion
/// used for equality and difference is based only on the distance between the
/// points instead of both distance and angle for Vector3's.

class Point3 : public Vector3
{
public:

// LIFECYCLE

  /// Default constructor. Creates [0.0, 0.0, 0.0] point.
  Point3();

  /// Constructor for three input values.
  Point3(const Scalar& x,
         const Scalar& y,
         const Scalar& z);

  /// Constructor for pointer input.
  explicit Point3(const Scalar* p);

  /// Constructor for Vector3 input.
  Point3(const Vector3& v);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  /// Equality operator. The comparison criterion is that the distance between
  /// the two points must be equal to zero.
  bool operator==(const Point3& p) const;

  bool operator!=(const Point3& p) const;
};


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Point3::Point3() : Vector3() {}


inline Point3::Point3(const Scalar& x,
                      const Scalar& y,
                      const Scalar& z) : Vector3(x, y, z) {}


inline Point3::Point3(const Scalar* p) : Vector3(p) {}


inline Point3::Point3(const Vector3& v) : Vector3(v) {}

// OPERATORS

inline bool Point3::operator==(const Point3& p) const
{
  return (distance(p) == 0.0);
}


inline bool Point3::operator!=(const Point3& p) const
{
  return !(*this == p);
}

} // mt

#endif // MT_POINT3_H
