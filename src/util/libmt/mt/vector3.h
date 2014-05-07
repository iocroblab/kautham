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
#ifndef MT_VECTOR3_H
#define MT_VECTOR3_H

// UNDEFINES min AND max MACROS IF THEY EXIST
#ifdef min
  #undef min
#endif // min

#ifdef max
  #undef max
#endif // max

// C++ STANDARD HEADERS
#include <iostream>
#include <stdexcept>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>

// MT LIBRARY HEADERS
#include <mt/scalar.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief Three-dimensional vector class.
///
/// The Vector3 class provides the usual operators and functions used in vector
/// algebra and manipulation.
/// The type of a vector element is Scalar, so in order to use the class, the
/// type name must exist either as a class or a typedef.
///
/// This is an example of how to use the Vector3 class:
///
/// \code
/// // The definition of the used Scalar type is from the scalar.h header of
/// // this library
/// using namespace mt;
///
/// // Constructors and some basic operators
/// Vector3 v1(1.0, 0.0, -2.5);   // three-value constructor
/// Vector3 v2;                   // default constructor, creates null vector
/// v2 = 2.0 * v1;                // v2 = [2.0, 0.0, -5.0]
/// Vector3 v3 = v1 + v2;         // v3 = [3.0, 0.0, -7.5]
///
/// // Some vector operations
/// Scalar s1  = dot(v2, v2);     // s1 = 29.0
/// v1.setValue(2.0, 0.0, 0.0);   // v1 = [2.0, 0.0, 0.0]
/// v2.setValue(0.0, 0.0, 1.0);   // v2 = [0.0, 0.0, 1.0]
/// v3 = cross(v1, v2);           // v3 = [0.0, -2.0, 0.0]
/// s1 = distance(v1, v2);        // s1 = 2.23607
/// s1 = radToDeg(angle(v1, v2)); // s1 = 90 degrees
/// v1 = normalize(v3);           // v1 = [0.0, -1.0, 0.0], v3 = [0.0, -2.0, 0.0] v3 remains the same
/// v3.normalize();               // v3 = [0.0, -1.0, 0.0] v3 has changed
/// \endcode

class Vector3
{
public:

// LIFECYCLE

  /// Default constructor. Creates a null vector [0.0, 0.0, 0.0].
  Vector3();

  /// Constructor for three input values.
  Vector3(const Scalar& x,
          const Scalar& y,
          const Scalar& z);

  /// Constructor for pointer input.
  explicit Vector3(const Scalar* v);

  // Compiler generated copy constructor is being used

  virtual ~Vector3() {}


// OPERATORS

  // Compiler generated assignment operator is being used

  /// Unchecked element access.
  Scalar& operator[](size_t n);

  /// Unchecked element access.
  const Scalar& operator[](size_t n) const;

  Vector3& operator+=(const Vector3& v);
  Vector3& operator-=(const Vector3& v);

  /// Scalar-vector product
  Vector3& operator*=(const Scalar& s);

  /// Memberwise product.
  Vector3& operator*=(const Vector3& v);

   /// Scalar-vector division.
  Vector3& operator/=(const Scalar& s);

  /// Memberwise division.
  Vector3& operator/=(const Vector3& v);


  /// Equality operator. The comparison criterion is that both the distance
  /// and angle between the two vectors must be equal to zero.
  bool operator==(const Vector3& v) const;

  bool operator!=(const Vector3& v) const;


// OPERATIONS

  /// Vector element sum.
  Scalar sum() const;

  /// Dot product.
  Scalar dot(const Vector3& v) const;

  /// Cross product.
  Vector3 cross(const Vector3& v) const;

  /// Squared vector length.
  Scalar length2() const;

  /// Vector length.
  Scalar length() const;

  /// Normalize vector to unit length.
  Vector3& normalize();

  /// Squared distance between vectors.
  Scalar distance2(const Vector3& v) const;

  /// Distance between vectors.
  Scalar distance(const Vector3& v) const;

  /// Cosine of the angle between vectors.
  Scalar angleCos(const Vector3& v) const;

  /// Sine of the angle between vectors.
  Scalar angleSin(const Vector3& v) const;

  /// Angle between vectors in the range [0, pi] expressed in radians.
  Scalar angle(const Vector3& v) const;

  /// Index of element with minimum value.
  size_t minAxis() const;

  /// Index of element with maximum value.
  size_t maxAxis() const;

  /// Index of furthest axis.
  size_t furthestAxis() const;

  /// Index of closest axis.
  size_t closestAxis() const;

  /// Minimum value.
  Scalar min() const;

  /// Maximum value.
  Scalar max() const;


// ACCESS

  /// Checked element access.
  Scalar& at(size_t n);

  /// Checked element access.
  const Scalar& at(size_t n) const;

  /// Sets vector values from three input values.
  void setValue(const Scalar& x,
                const Scalar& y,
                const Scalar& z);

  /// Sets vector values from pointer input.
  void setValue(const Scalar* v);


private:

// MEMBERS

Scalar m_co[3];


// FRIENDS

friend class Quaternion;

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

Vector3 operator+(const Vector3& v);

Vector3 operator-(const Vector3& v);

Vector3 operator+(const Vector3& v1,
                  const Vector3& v2);

Vector3 operator-(const Vector3& v1,
                  const Vector3& v2);

Vector3 operator*(const Vector3& v1,
                  const Vector3& v2);

Vector3 operator/(const Vector3& v1,
                  const Vector3& v2);

Vector3 operator*(const Scalar&  s,
                  const Vector3& v);

Vector3 operator*(const Vector3& v,
                  const Scalar&  s);

Vector3 operator/(const Vector3& v,
                  const Scalar&  s);

std::ostream& operator<<(std::ostream&  os,
                         const Vector3& v);


// FUNCTIONS

/// Squared vector length.
Scalar length2(const Vector3& v);

/// Vector length.
Scalar length(const Vector3& v);

/// Normalized vector.
Vector3 normalize(const Vector3& v);

/// Suqared distance between two vectors.
Scalar distance2(const Vector3& v1,
                 const Vector3& v2);

/// Distance between two vectors.
Scalar distance(const Vector3& v1,
                const Vector3& v2);

/// Cosine of the angle between vectors.
Scalar angleCos(const Vector3& v1,
                const Vector3& v2);

/// Sine of the angle between vectors.
Scalar angleSin(const Vector3& v1,
                const Vector3& v2);

/// Angle between vectors in the range [0, pi] expressed in radians.
Scalar angle(const Vector3& v1,
             const Vector3& v2);

/// Memberwise absolute value.
Vector3 abs(const Vector3& v);

/// Dot product.
Scalar dot(const Vector3& v1,
           const Vector3& v2);

/// Cross product.
Vector3 cross(const Vector3& v1,
              const Vector3& v2);

/// Triple product between vectors: dot(\a v1, cross(\a v2, \a v3)).
Scalar triple(const Vector3& v1,
              const Vector3& v2,
              const Vector3& v3);

/// Linear inerpolation/extrapolation between input vectors.
Vector3 lerp(const Vector3& v1,
             const Vector3& v2,
             const Scalar&   s);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Vector3::Vector3()
{
  setValue(Scalar(0.0),
           Scalar(0.0),
           Scalar(0.0));
}


inline Vector3::Vector3(const Scalar& x,
                        const Scalar& y,
                        const Scalar& z)
{
  setValue(x, y, z);
}


inline Vector3::Vector3(const Scalar* v)
{
  setValue(v);
}


// OPERATORS

inline Scalar& Vector3::operator[](size_t n)
{
  return m_co[n];
}


inline const Scalar& Vector3::operator[](size_t n) const
{
  return m_co[n];
}


inline Vector3& Vector3::operator+=(const Vector3& v)
{
  m_co[0] += v[0];
  m_co[1] += v[1];
  m_co[2] += v[2];
  return *this;
}


inline Vector3& Vector3::operator-=(const Vector3& v)
{
  m_co[0] -= v[0];
  m_co[1] -= v[1];
  m_co[2] -= v[2];
  return *this;
}


inline Vector3& Vector3::operator*=(const Scalar& s)
{
  m_co[0] *= s;
  m_co[1] *= s;
  m_co[2] *= s;
  return *this;
}


inline Vector3& Vector3::operator*=(const Vector3& v)
{
  m_co[0] *= v[0];
  m_co[1] *= v[1];
  m_co[2] *= v[2];
  return *this;
}


inline Vector3& Vector3::operator/=(const Scalar& s)
{
  m_co[0] /= s;
  m_co[1] /= s;
  m_co[2] /= s;
  return *this;
}


inline Vector3& Vector3::operator/=(const Vector3& v)
{
  m_co[0] /= v[0];
  m_co[1] /= v[1];
  m_co[2] /= v[2];
  return *this;
}


inline bool Vector3::operator==(const Vector3& v) const
{
  if (this->length2() == Scalar(0.0) ||
      v.length2()     == Scalar(0.0))
  {
    return (distance(v) == Scalar(0.0));
  }
  else
  {
    return (distance(v) == Scalar(0.0)) && (angle(v) == Scalar(0.0));
  }
}


inline bool Vector3::operator!=(const Vector3& v) const
{
  return !(*this == v);
}


// OPERATIONS

inline Scalar Vector3::sum() const
{
  return (m_co[0] + m_co[1] + m_co[2]);
}


inline Scalar Vector3::dot(const Vector3& v) const
{
  return (m_co[0] * v[0]) + (m_co[1] * v[1]) + (m_co[2] * v[2]);
}


inline Vector3 Vector3::cross(const Vector3& v) const
{
  return Vector3(m_co[1] * v[2] - m_co[2] * v[1],
                 m_co[2] * v[0] - m_co[0] * v[2],
                 m_co[0] * v[1] - m_co[1] * v[0]);
}


inline Scalar Vector3::length2() const
{
  return dot(*this);
}


inline Scalar Vector3::length() const
{
  return sqrt(length2());
}


inline Vector3& Vector3::normalize()
{
  *this /= length();
  return *this;
}


inline Scalar Vector3::distance2(const Vector3& v) const
{
  Vector3 diff(*this);
  diff -= v;
  const Scalar d2 = diff.length2();
  return d2;
}


inline Scalar Vector3::distance(const Vector3& v) const
{
  Vector3 diff(*this);
  diff -= v;
  const Scalar d = diff.length();
  return d;
}


inline Scalar Vector3::angleCos(const Vector3& v) const
{
  const Scalar num(dot(v));
  const Scalar den(sqrt(length2() * v.length2()));

  #ifdef MT_USE_BASIC_SCALAR
    util::Assert((den != Scalar(0.0)),
            Exception("Angles with respect to the null vector are undefined."));
  #endif

  Scalar ang_cos(num / den);
  return ang_cos;
}


inline Scalar Vector3::angleSin(const Vector3& v) const
{
  const Vector3 cross(this->cross(v));
  const Scalar  num(cross.length());
  const Scalar  den(sqrt(length2() * v.length2()));

  Scalar ang_sin(num / den);
  return ang_sin;
}


inline Scalar Vector3::angle(const Vector3& v) const
{
  const Scalar ang_cos(angleCos(v));
  return acos(ang_cos);
}


inline size_t Vector3::minAxis() const
{
  return m_co[0] < m_co[1] ? (m_co[0] < m_co[2] ? 0 : 2)
                           : (m_co[1] < m_co[2] ? 1 : 2);
}


inline size_t Vector3::maxAxis() const
{
  return m_co[0] < m_co[1] ? (m_co[1] < m_co[2] ? 2 : 1)
                           : (m_co[0] < m_co[2] ? 2 : 0);
}


inline size_t Vector3::furthestAxis() const
{
  return abs(*this).minAxis();
}


inline size_t Vector3::closestAxis() const
{
  return abs(*this).maxAxis();
}


inline Scalar Vector3::min() const
{
  const size_t t = minAxis();
  return m_co[t];
}


inline Scalar Vector3::max() const
{
  const size_t t = maxAxis();
  return m_co[t];
}


// ACCESS

inline Scalar& Vector3::at(size_t n)
{
  util::Assert(n < 3, std::range_error("Index out of range"));
  return m_co[n];
}


inline const Scalar& Vector3::at(size_t n) const
{
  util::Assert(n < 3, std::range_error("Index out of range"));
  return m_co[n];
}


inline void Vector3::setValue(const Scalar& x,
                              const Scalar& y,
                              const Scalar& z)
{
  m_co[0] = x;
  m_co[1] = y;
  m_co[2] = z;
}


inline void Vector3::setValue(const Scalar* v)
{
  m_co[0] = v[0];
  m_co[1] = v[1];
  m_co[2] = v[2];
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Vector3 operator+(const Vector3& v)
{
  return v;
}


inline Vector3 operator-(const Vector3& v)
{
  const Vector3 v1(-v[0],
                   -v[1],
                   -v[2]);
  return v1;
}


inline Vector3 operator+(const Vector3& v1,
                         const Vector3& v2)
{
  Vector3 v(v1);
  return v+= v2;
}


inline Vector3 operator-(const Vector3& v1,
                         const Vector3& v2)
{
  Vector3 v(v1);
  return v-= v2;
}


inline Vector3 operator*(const Vector3& v1,
                         const Vector3& v2)
{
  Vector3 v(v1);
  return v*= v2;
}


inline Vector3 operator/(const Vector3& v1,
                         const Vector3& v2)
{
  Vector3 v(v1);
  return v/= v2;
}


inline Vector3 operator*(const Scalar&  s,
                         const Vector3& v)
{
  Vector3 v1(v);
  return v1*= s;
}


inline Vector3 operator*(const Vector3& v,
                         const Scalar&  s)
{
  Vector3 v1(v);
  return v1*= s;
}


inline Vector3 operator/(const Vector3& v,
                         const Scalar&  s)
{
  Vector3 v1(v);
  return v1/= s;
}


inline std::ostream& operator<<(std::ostream& os,
                                const Vector3& v)
{
  return os << '[' << v[0] << ' '
                   << v[1] << ' '
                   << v[2] << ']';
}


// FUNCTIONS

inline Scalar length2(const Vector3& v)
{
  return v.length2();
}


inline Scalar length(const Vector3& v)
{
  return v.length();
}


inline Vector3 normalize(const Vector3& v)
{
  Vector3 v_norm(v);
  v_norm.normalize();
  return v_norm;
}


inline Scalar distance2(const Vector3& v1,
                        const Vector3& v2)
{
  return v1.distance2(v2);
}


inline Scalar distance(const Vector3& v1,
                       const Vector3& v2)
{
  return v1.distance(v2);
}


inline Scalar angleCos(const Vector3& v1,
                       const Vector3& v2)
{
  return v1.angleCos(v2);
}


inline Scalar angleSin(const Vector3& v1,
                       const Vector3& v2)
{
  return v1.angleSin(v2);
}


inline Scalar angle(const Vector3& v1,
                    const Vector3& v2)
{
  return v1.angle(v2);
}


inline Vector3 abs(const Vector3& v)
{
  Vector3 v_abs(v);
  v_abs[0] = abs(v_abs[0]);
  v_abs[1] = abs(v_abs[1]);
  v_abs[2] = abs(v_abs[2]);
  return v_abs;
}


inline Scalar dot(const Vector3& v1,
                  const Vector3& v2)
{
  return v1.dot(v2);
}


inline Vector3 cross(const Vector3& v1,
                     const Vector3& v2)
{
  const Vector3 v_cross(v1.cross(v2));
  return v_cross;
}


inline Scalar triple(const Vector3& v1,
                     const Vector3& v2,
                     const Vector3& v3)
{
  return v1[0] * (v2[1] * v3[2] - v2[2] * v3[1]) +
         v1[1] * (v2[2] * v3[0] - v2[0] * v3[2]) +
         v1[2] * (v2[0] * v3[1] - v2[1] * v3[0]);
}


inline Vector3 lerp(const Vector3& v1,
                    const Vector3& v2,
                    const Scalar&   s)
{
  const Vector3 v_in(v1[0] + (v2[0] - v1[0]) * s,
                     v1[1] + (v2[1] - v1[1]) * s,
                     v1[2] + (v2[2] - v1[2]) * s);
  return v_in;
}


} // mt

#endif // MT_VECTOR3_H
