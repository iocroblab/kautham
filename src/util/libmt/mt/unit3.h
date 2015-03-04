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
#ifndef MT_UNIT3_H
#define MT_UNIT3_H


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
/// \brief Three dimensional unit vector class.
///
/// The Unit3 class provides the usual operators and functions used in vector
/// algebra and manipulation.
///
/// The main difference from Vector3, its base class, is that the vector length
/// is \e always guarranteed to be unity, so for example, a unit vector cannot
/// be initialized with a null vector, and if a vector with length different
/// than unity is assigned to a unit vector, it is scaled to unit length.
/// Also, the comparison criterion used for equality and difference is based 
/// only on the angle between the Unit3's instead of both distance and angle for
/// Vector3's.
/// This is an example of how to use the Unit3 class and its differences with
/// the Vector3 class:
///
/// \code
/// // The definition of the used Scalar type is from the scalar.h header of
/// // this library
/// using namespace mt;
///
/// // Constructors
/// Unit3   u1;                   // u1 = [0.0, 0.0, 1.0]
/// Unit3   u2(2.0, 0.0, 0.0);    // u2 = [1.0, 0.0, 0.0]
///
/// Vector3 v1;                   // v1 = [0.0, 0.0, 0.0]
/// Vector3 v2(2.0, 0.0, 0.0);    // v2 = [2.0, 0.0, 0.0]
///
/// // Operations
/// u1 = -2.0 * u2;               // u1 = [-1.0, 0.0, 0.0]
/// u1 = -2.0 * v2;               // u1 = [-1.0, 0.0, 0.0]
/// v1 = -2.0 * v2;               // v1 = [-4.0, 0.0, 0.0]
///
/// // Orthonormal basis defined by two unit vectors
/// u1.setValue(1.0, 0.0, 0.0);   // u1 = [1.0, 0.0, 0.0]
/// u2.setValue(1.0, 1.0, 0.0);   // u2 = [0.70717, 0.70717, 0.0]
///
/// Unit3 onbase2;
/// Unit3 onbase3;
///
/// orthonormalBasis(u1, u2, onbase2, onbase3);
/// // Orthonormal basis is given by unit vectors {u1, onbase2, onbase3}
/// // u1      = [1.0, 0.0, 0.0]
/// // onbase2 = [0.0, 1.0, 0.0]
/// // onbase3 = [0.0, 0.0, 1.0]
/// \endcode

class Unit3 : public Vector3
{
public:

// LIFECYCLE

  /// Default constructor. Creates [0.0, 0.0, 1.0] vector.
  Unit3();

  /// Constructor for three input values.
  ///
  /// The input values need not define a vector of unit length, since a
  /// vector is constructed and then normalized.
  Unit3(const Scalar& x,
        const Scalar& y,
        const Scalar& z);

  /// Constructor for pointer input.
  explicit Unit3(const Scalar* u);

  /// Constructor for Vector3 input.
  Unit3(const Vector3& v);

  // Compiler generated copy constructor for Unit3 input is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator for Unit3 input is being used

  /// Assignment operator.
  /// @param v The Vector3 to assign to this object.
  /// @return A reference to this object.
  Unit3& operator=(const Vector3& v);

  /// Equality operator. The comparison criterion is that the angle between
  /// the two vectors must be equal to zero.
  bool operator==(const Unit3& u) const;

  bool operator!=(const Unit3& u) const;

// OPERATIONS

  /// Cosine of the angle between vectors.
  Scalar angleCos(const Unit3& u) const;

  /// Angle between vectors in the range [0, pi] expressed in radians.
  Scalar angle(const Unit3& u) const;

// ACCESS

  /// Sets vector values from three input values.
  void setValue(const Scalar& x,
                const Scalar& y,
                const Scalar& z);

  /// Sets vector values from pointer input.
  void setValue(const Scalar* u);

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// FUNCTIONS

/// Cosine of the angle between vectors.
Scalar angleCos(const Unit3& u1,
                const Unit3& u2);


/// Angle between vectors in the range [0, pi] expressed in radians.
Scalar angle(const Unit3& u1,
             const Unit3& u2);

/// Orthonormal basis defined by two unit vectors.
/// Given two unit vectors \a u1 and \a u2, the orthonormal basis associated to
/// them is given by {\a u1, \a v2, \a v3}.
///
/// The initial values of vectors \a v2 and \a v3 are never used. The only
/// operation done on them is the assignation of the orthonormal basis vectors.
///
/// \param u1 Input unit vector (orthonormal basis reference).
/// \param u2 Input unit vector.
/// \param v2 Output unit vector (part of orthonormal basis).
/// \param v3 Output unit vector (part of orthonormal basis).
void orthonormalBasis(const Unit3& u1,
                      const Unit3& u2,
                            Unit3& v2,
                            Unit3& v3);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Unit3::Unit3() : Vector3(0.0, 0.0, 1.0) {}


inline Unit3::Unit3(const Scalar& x,
                    const Scalar& y,
                    const Scalar& z)
{
  setValue(x, y, z);
}


inline Unit3::Unit3(const Scalar* u)
{
  setValue(u);
}


inline Unit3::Unit3(const Vector3& v) : Vector3(v)
{
  normalize();
}


// OPERATORS

inline Unit3& Unit3::operator=(const Vector3& v)
{
  if (this != &v)
  {
    // Assignation
    Vector3::operator=(v);
    normalize();
  }
  return *this;
}


inline bool Unit3::operator==(const Unit3& u) const
{
  return (angle(u) == 0.0);
}


inline bool Unit3::operator!=(const Unit3& u) const
{
  return !(*this == u);
}


// OPERATIONS

inline Scalar Unit3::angleCos(const Unit3& u) const
{
  Scalar ang_cos(saturate(dot(u), Scalar(-1.0), Scalar(1.0)));
  return ang_cos;
}


inline Scalar Unit3::angle(const Unit3& u) const
{
  const Scalar ang_cos(angleCos(u));
  return acos(ang_cos);
}


// ACCESS

inline void Unit3::setValue(const Scalar& x,
                            const Scalar& y,
                            const Scalar& z)
{
  Vector3::setValue(x, y, z);
  normalize();
}


inline void Unit3::setValue(const Scalar* u)
{
  Vector3::setValue(u);
  normalize();
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

inline Scalar angleCos(const Unit3& u1,
                       const Unit3& u2)
{
  return u1.angleCos(u2);
}


inline Scalar angle(const Unit3& u1,
                    const Unit3& u2)
{
  return u1.angle(u2);
}


inline void orthonormalBasis(const Unit3& u1,
                             const Unit3& u2,
                                   Unit3& v2,
                                   Unit3& v3)
{
  // If vectors are parallel or antiparallel an exception is thrown
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(abs(angleCos(u1, u2)) != 1.0,
                Exception("Cannot define orthonormal basis from (anti)parallel \
input vectors"));
  #endif

  // Values of unit vectors perpendicular to "u1"
  v2 = u2 - dot(u1, u2) * u1;
  v3 = cross(u1, v2);
}

} // mt

#endif // MT_UNIT3_H
