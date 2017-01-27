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
#ifndef MT_QUATERNION_H
#define MT_QUATERNION_H

// C++ STANDARD HEADERS
#include <iostream>
#include <stdexcept>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>

// MT LIBRARY HEADERS
#include <mt/scalar.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief %Quaternion class.
///
/// The Quaternion class provides the usual operators and functions used for
/// expressing and manipulating quaternions.
///
/// Let \f$ q \f$ be a quaternion, then the following notations are analogous:
/// \f$ \mathbf{q} = [ x, y, z, w ] = xi + yj + zk + w \f$.

class Quaternion
{
public:

// LIFECYCLE

  /// Default constructor. Creates the [0.0, 0.0, 0.0, 1.0] quaternion.
  Quaternion();

  /// Constructor for four input values, where \f$ xi + yj + zk + w \f$.
  Quaternion(const Scalar& x,
             const Scalar& y,
             const Scalar& z,
             const Scalar& w);

  /// Constructor for pointer input.
  explicit Quaternion(const Scalar* q);

  // Compiler generated copy constructor is being used

  virtual ~Quaternion(){}


// OPERATORS

  // Compiler generated assignment operator is being used

  /// Unchecked element access.
  Scalar& operator[](size_t n);

  /// Unchecked element access.
  const Scalar& operator[](size_t n) const;

  Quaternion& operator+=(const Quaternion& q);
  Quaternion& operator-=(const Quaternion& q);

  /// Quaternion product (\e not memberwise product).
  Quaternion& operator*=(const Quaternion& q);

  /// Scalar-quaternion product.
  Quaternion& operator*=(const Scalar& s);

   /// Scalar-quaternion division.
  Quaternion& operator/=(const Scalar& s);

  /// Applies rotation to input vector.
  Vector3 operator()(const Vector3& v) const;

  /// Equality operator. The comparison criterion is that both the distance
  /// and angle between the two quaternions must be equal to zero.
  bool operator==(const Quaternion& q) const;

  bool operator!=(const Quaternion& q) const;

// OPERATIONS

  /// Dot product.
  Scalar dot(const Quaternion& q) const;

  /// Squared quaternion length.
  Scalar length2() const;

  /// Quaternion length.
  Scalar length() const;

  /// Normalize quaternion to unit length.
  Quaternion& normalize();

  /// Squared distance between quaternions.
  Scalar distance2(const Quaternion& q) const;

  /// Distance between quaternions.
  Scalar distance(const Quaternion& q) const;

  /// Cosine of the angle between quaternions.
  Scalar angleCos(const Quaternion& q) const;

  /// Angle between quaternions in the range [0, pi] expressed in radians.
  Scalar angle(const Quaternion& q) const;

  /// Quaternion conjugate.
  Quaternion conjugate() const;

  /// Quaternion inverse.
  Quaternion inverse() const;

  /// Spherical linear interpolation/extrapolation with input quaternion.
  Quaternion slerp(const Quaternion& q,
                   const Scalar&     t) const;

// ACCESS

  /// Checked element access.
  Scalar& at(size_t n);

  /// Checked element access.
  const Scalar& at(size_t n) const;

  /// Sets quaternion values.
  void setValue(const Scalar& x,
                const Scalar& y,
                const Scalar& z,
                const Scalar& w);

  /// Sets quaternion values from pointer input.
  void setValue(const Scalar* q);


protected:

// MEMBERS

Scalar m_co[4];

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

Quaternion operator+(const Quaternion& q);

Quaternion operator-(const Quaternion& q);

Quaternion operator+(const Quaternion& q1,
                     const Quaternion& q2);

Quaternion operator-(const Quaternion& q1,
                     const Quaternion& q2);

Quaternion operator*(const Quaternion& q1,
                     const Quaternion& q2);

Quaternion operator*(const Quaternion&  q,
                     const Vector3&     v);

Quaternion operator*(const Vector3&     v,
                     const Quaternion&  q);

Quaternion operator*(const Scalar&      s,
                     const Quaternion&  q);

Quaternion operator*(const Quaternion&  q,
                     const Scalar&      s);

Quaternion operator/(const Quaternion&  q,
                     const Scalar&      s);

std::ostream& operator<<(std::ostream&      os,
                         const Quaternion&  q);


// FUNCTIONS

/// Squared quaternion length.
Scalar length2(const Quaternion& q);

/// Quaternion length.
Scalar length(const Quaternion& q);

/// Normalized quaternion.
Quaternion normalize(const Quaternion& q);

/// Cosine of the angle between quaternions.
Scalar angleCos(const Quaternion& q1,
                const Quaternion& q2);

/// Angle between quaternions in the range [0, pi] expressed in radians.
Scalar angle(const Quaternion& q1,
             const Quaternion& q2);

/// Dot product.
Scalar dot(const Quaternion& q1,
           const Quaternion& q2);

/// Quaternion conjugate.
Quaternion conjugate(const Quaternion& q);

/// Quaternion inverse.
Quaternion inverse(const Quaternion& q);

/// Spherical linear interpolation/extrapolation between input quaternions.
Quaternion slerp(const Quaternion& q1,
                 const Quaternion& q2,
                 const Scalar&      t);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Quaternion::Quaternion()
{
  setValue(0.0, 0.0, 0.0, 1.0);
}


inline Quaternion::Quaternion(const Scalar& x,
                              const Scalar& y,
                              const Scalar& z,
                              const Scalar& w)
{
  setValue(x, y, z, w);
}


inline Quaternion::Quaternion(const Scalar* q)
{
  setValue(q);
}


// OPERATORS

inline Scalar& Quaternion::operator[](size_t n)
{
  return m_co[n];
}


inline const Scalar& Quaternion::operator[](size_t n) const
{
  return m_co[n];
}


inline Quaternion& Quaternion::operator+=(const Quaternion& q)
{
  m_co[0] += q[0];
  m_co[1] += q[1];
  m_co[2] += q[2];
  m_co[3] += q[3];
  return *this;
}


inline Quaternion& Quaternion::operator-=(const Quaternion& q)
{
  m_co[0] -= q[0];
  m_co[1] -= q[1];
  m_co[2] -= q[2];
  m_co[3] -= q[3];
  return *this;
}


inline Quaternion& Quaternion::operator*=(const Scalar& s)
{
  m_co[0] *= s;
  m_co[1] *= s;
  m_co[2] *= s;
  m_co[3] *= s;
  return *this;
}


inline Quaternion& Quaternion::operator*=(const Quaternion& q)
{
  setValue(m_co[3] * q[0] + m_co[0] * q[3] + m_co[1] * q[2] - m_co[2] * q[1],
           m_co[3] * q[1] + m_co[1] * q[3] + m_co[2] * q[0] - m_co[0] * q[2],
           m_co[3] * q[2] + m_co[2] * q[3] + m_co[0] * q[1] - m_co[1] * q[0],
           m_co[3] * q[3] - m_co[0] * q[0] - m_co[1] * q[1] - m_co[2] * q[2]);
  return *this;
}


inline Quaternion& Quaternion::operator/=(const Scalar& s)
{
  m_co[0] /= s;
  m_co[1] /= s;
  m_co[2] /= s;
  m_co[3] /= s;
  return *this;
}


inline Vector3 Quaternion::operator()(const Vector3& v) const
{
  const Quaternion q(*this * (v * inverse()));
  return Vector3(q[0], q[1], q[2]);
}


inline bool Quaternion::operator==(const Quaternion& q) const
{
  return (distance(q) == Scalar(0.0)) && (angle(q) == Scalar(0.0));
}


inline bool Quaternion::operator!=(const Quaternion& q) const
{
  return !(*this == q);
}


// OPERATIONS

inline Scalar Quaternion::dot(const Quaternion& q) const
{
  return m_co[0] * q[0] +
         m_co[1] * q[1] +
         m_co[2] * q[2] +
         m_co[3] * q[3];
}


inline Scalar Quaternion::length2() const
{
  return dot(*this);
}


inline Scalar Quaternion::length() const
{
  return sqrt(length2());
}


inline Quaternion& Quaternion::normalize()
{
  return *this /= length();
}


inline Scalar Quaternion::distance2(const Quaternion& q) const
{
  Quaternion diff(*this);
  diff -= q;
  const Scalar d2 = diff.length2();
  return d2;
}


inline Scalar Quaternion::distance(const Quaternion& q) const
{
  Quaternion diff(*this);
  diff -= q;
  const Scalar d = diff.length();
  return d;
}


inline Scalar Quaternion::angleCos(const Quaternion& q) const
{
  const Scalar num(dot(q));
  const Scalar den(sqrt(length2() * q.length2()));

  Scalar ang_cos(saturate(num / den, Scalar(-1.0), Scalar(1.0)));
  return ang_cos;
}


inline Scalar Quaternion::angle(const Quaternion& q) const
{
  const Scalar ang_cos(angleCos(q));
  return acos(ang_cos);
}


inline Quaternion Quaternion::conjugate() const
{
  return Quaternion(-m_co[0], -m_co[1], -m_co[2], m_co[3]);
}


inline Quaternion Quaternion::inverse() const
{
  return conjugate() / length2();
}


inline Quaternion Quaternion::slerp(const Quaternion& q,
                                    const Scalar&     t) const
{
  // Angle between quaternions
  Scalar theta = angle(q);

  // Parallel test between quaternions
  if (theta != 0.0)
  {
    // Quaternion located at position determined by "t"
    Scalar d  = Scalar(1.0) / sin(theta);
    Scalar s0 = sin((Scalar(1.0) - t) * theta);
    Scalar s1 = sin(t * theta);
    return Quaternion((m_co[0] * s0 + q[0] * s1) * d,
                      (m_co[1] * s0 + q[1] * s1) * d,
                      (m_co[2] * s0 + q[2] * s1) * d,
                      (m_co[3] * s0 + q[3] * s1) * d);
  }
  else
  {
    return *this;
  }
}


// ACCESS

inline Scalar& Quaternion::at(size_t n)
{
  util::Assert(n < 4, std::range_error("Index out of range"));
  return m_co[n];
}


inline const Scalar& Quaternion::at(size_t n) const
{
  util::Assert(n < 4, std::range_error("Index out of range"));
  return m_co[n];
}


inline void Quaternion::setValue(const Scalar& x,
                                 const Scalar& y,
                                 const Scalar& z,
                                 const Scalar& w)
{
  m_co[0] = x;
  m_co[1] = y;
  m_co[2] = z;
  m_co[3] = w;
}


inline void Quaternion::setValue(const Scalar* q)
{
  m_co[0] = q[0];
  m_co[1] = q[1];
  m_co[2] = q[2];
  m_co[3] = q[3];
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Quaternion operator+(const Quaternion& q)
{
  return q;
}


inline Quaternion operator-(const Quaternion& q)
{
  return Quaternion(-q[0],
                    -q[1],
                    -q[2],
                    -q[3]);
}


inline Quaternion operator+(const Quaternion& q1,
                            const Quaternion& q2)
{
  Quaternion q(q1);
  return q += q2;
}


inline Quaternion operator-(const Quaternion& q1,
                            const Quaternion& q2)
{
  Quaternion q(q1);
  return q -= q2;
}


inline Quaternion operator*(const Quaternion& q1,
                            const Quaternion& q2)
{
  Quaternion q(q1);
  return q *= q2;
}

inline Quaternion operator*(const Quaternion& q,
                            const Vector3&    v)
{
  return Quaternion(q[3] * v[0] + q[1] * v[2] - q[2] * v[1],
                    q[3] * v[1] + q[2] * v[0] - q[0] * v[2],
                    q[3] * v[2] + q[0] * v[1] - q[1] * v[0],
                   -q[0] * v[0] - q[1] * v[1] - q[2] * v[2]);
}


inline Quaternion operator*(const Vector3&    v,
                            const Quaternion& q)
{
  return Quaternion(v[0] * q[3] + v[1] * q[2] - v[2] * q[1],
                    v[1] * q[3] + v[2] * q[0] - v[0] * q[2],
                    v[2] * q[3] + v[0] * q[1] - v[1] * q[0],
                   -v[0] * q[0] - v[1] * q[1] - v[2] * q[2]);
}


inline Quaternion operator*(const Scalar&     s,
                            const Quaternion& q)
{
  Quaternion q1(q);
  return q1 *= s;
}


inline Quaternion operator*(const Quaternion& q,
                            const Scalar&     s)
{
  Quaternion q1(q);
  return q1 *= s;
}

inline Quaternion operator/(const Quaternion& q,
                            const Scalar&     s)
{

  Quaternion q1(q);
  return q1 /= s;
}

inline std::ostream& operator<<(std::ostream&     os,
                                const Quaternion& q)
{
  return os << '[' << q[0] << ' '
                   << q[1] << ' '
                   << q[2] << ' '
                   << q[3] << ']';
}


// FUNCTIONS

inline Scalar length2(const Quaternion& q)
{
  return q.length2();
}


inline Scalar length(const Quaternion& q)
{
  return q.length();
}


inline Quaternion normalize(const Quaternion& q)
{
  Quaternion q1(q);
  return q1.normalize();
}


inline Scalar angleCos(const Quaternion& q1,
                       const Quaternion& q2)
{
  return q1.angleCos(q2);
}


inline Scalar angle(const Quaternion& q1,
                    const Quaternion& q2)
{
  return q1.angle(q2);
}


inline Scalar dot(const Quaternion& q1,
                  const Quaternion& q2)
{
  return q1.dot(q2);
}


inline Quaternion conjugate(const Quaternion& q)
{
  return q.conjugate();
}


inline Quaternion inverse(const Quaternion& q)
{
  return q.inverse();
}


inline Quaternion slerp(const Quaternion& q1,
                        const Quaternion& q2,
                        const Scalar&      t)
{
  return q1.slerp(q2, t);
}

} // mt

#endif // MT_QUATERNION_H
