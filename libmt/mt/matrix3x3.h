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
#ifndef MT_MATRIX3X3_H
#define MT_MATRIX3X3_H

// C++ STANDARD HEADERS
#include <iostream>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>

// MT LIBRARY HEADERS
#include <mt/scalar.h>
#include <mt/unit3.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief Row-major 3x3 matrix class.
///
/// The Matrix3x3 class provides the usual operators and functions used for
/// expressing and manipulating 3x3 matrices.

class Matrix3x3
{
public:

// LIFECYCLE

  /// Default constructor. Creates null matrix.
  Matrix3x3();

  /// Constructor for 9 (row-major) scalar input values.
  Matrix3x3(const Scalar& xx, const Scalar& xy, const Scalar& xz,
            const Scalar& yx, const Scalar& yy, const Scalar& yz,
            const Scalar& zx, const Scalar& zy, const Scalar& zz);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  /// Unchecked element access.
  /// \return \a n th matrix row.
  Vector3& operator[](size_t n);

  /// Unchecked element access.
  /// \return \a n th matrix row.
  const Vector3& operator[](size_t n) const;

  /// Matrix-matrix product.
  Matrix3x3& operator*=(const Matrix3x3& m);

// OPERATIONS

  /// Dot product between matrix column given by \a c and vector \a v.
  Scalar tdot(      size_t   c,
              const Vector3& v) const;

  /// Matrix determinant.
  Scalar determinant() const;

  /// Adjoint matrix.
  Matrix3x3 adjoint() const;

  /// Transpose matrix.
  Matrix3x3 transpose() const;

  /// Inverse matrix.
  Matrix3x3 inverse() const;

  /// Calculates \f$ \mathrm{M_1}^T \mathrm{M_2} \f$.
  Matrix3x3 transposeTimes(const Matrix3x3& m) const;

  /// Calculates  \f$ \mathrm{M_1} \mathrm{M_2}^T \f$.
  Matrix3x3 timesTranspose(const Matrix3x3& m) const;

  /// Scales matrix according to input vector \a v.
  Matrix3x3& scale(const Vector3& v);


// ACCESS

  /// Checked element access.
  /// \return \a n th matrix row.
  Vector3& at(size_t n);

  /// Checked element access.
  /// \return \a n th matrix row.
  const Vector3& at(size_t n) const;

  /// Gets matrix scaling vector.
  Vector3 getScaling() const;

  /// Sets matrix values from 9 scalar input values (row-major ordered).
  void setValue(const Scalar& xx, const Scalar& xy, const Scalar& xz,
                const Scalar& yx, const Scalar& yy, const Scalar& yz,
                const Scalar& zx, const Scalar& zy, const Scalar& zz);

  /// Sets identity matrix values.
  void setIdentity();


private:

// MEMBERS

Vector3 m_el[3];


// METHODS

/** Calculates the determinant defined by the 2x2 matrix
\f[
  \left[
  \begin{array}{l l}
    a_{r_1, c_1} & a_{r_1, c_2} \\
    a_{r_2, c_1} & a_{r_2, c_2} \\
  \end{array}
  \right]
\f]
*/
Scalar cofac(size_t r1, size_t c1,
             size_t r2, size_t c2) const;

};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Matrix-vector product.
Vector3 operator*(const Matrix3x3& m,
                  const Vector3&   v);

/// Vector-matrix product.
Vector3 operator*(const Vector3&   v,
                  const Matrix3x3& m);

/// Matrix-matrix product.
Matrix3x3 operator*(const Matrix3x3& m1,
                    const Matrix3x3& m2);

std::ostream& operator<<(std::ostream&    os,
                         const Matrix3x3& m);


// FUNCTIONS

/// Memberwise absolute value.
Matrix3x3 abs(const Matrix3x3& m);

/// Scales matrix \a m according to input vector \a v.
Matrix3x3 scale(const Matrix3x3 m,
                const Vector3& v);

/// Matric determinant.
Scalar determinant(const Matrix3x3& m);

/// Matrix adjoint.
Matrix3x3 adjoint(const Matrix3x3& m);

/// Matrix transpose.
Matrix3x3 transpose(const Matrix3x3& m);

/// Matrix inverse.
Matrix3x3 inverse(const Matrix3x3& m);

/// Calculates \f$ \mathrm{M_1}^T \mathrm{M_2} \f$.
Matrix3x3 transposeTimes(const Matrix3x3& m1,
                         const Matrix3x3& m2);

/// Calculates \f$ \mathrm{M_1} \mathrm{M_2}^T \f$.
Matrix3x3 timesTranspose(const Matrix3x3& m1,
                         const Matrix3x3& m2);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Matrix3x3::Matrix3x3() {}


inline
Matrix3x3::Matrix3x3(const Scalar& xx, const Scalar& xy, const Scalar& xz,
                     const Scalar& yx, const Scalar& yy, const Scalar& yz,
                     const Scalar& zx, const Scalar& zy, const Scalar& zz)
{
  setValue(xx, xy, xz,
           yx, yy, yz,
           zx, zy, zz);
}


// OPERATORS

inline Vector3& Matrix3x3::operator[](size_t n)
{
  return m_el[n];
}


inline const Vector3& Matrix3x3::operator[](size_t n) const
{
  return m_el[n];
}


inline Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& m)
{
  setValue(m.tdot(0, m_el[0]), m.tdot(1, m_el[0]), m.tdot(2, m_el[0]),
           m.tdot(0, m_el[1]), m.tdot(1, m_el[1]), m.tdot(2, m_el[1]),
           m.tdot(0, m_el[2]), m.tdot(1, m_el[2]), m.tdot(2, m_el[2]));
  return *this;
}


// OPERATIONS

inline Scalar Matrix3x3::tdot(      size_t   c,
                              const Vector3& v) const
{
  return (m_el[0][c] * v[0] + m_el[1][c] * v[1] + m_el[2][c] * v[2]);
}


inline Scalar Matrix3x3::determinant() const
{
  return triple((*this)[0], (*this)[1], (*this)[2]);
}


inline Matrix3x3 Matrix3x3::adjoint() const
{
  return Matrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
                   cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
                   cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
}


inline Matrix3x3 Matrix3x3::transpose() const
{
  return Matrix3x3(m_el[0][0], m_el[1][0], m_el[2][0],
                   m_el[0][1], m_el[1][1], m_el[2][1],
                   m_el[0][2], m_el[1][2], m_el[2][2]);
}


inline Matrix3x3 Matrix3x3::inverse() const
{
  // Matrix cofactors
  const Vector3 co(cofac(1, 1, 2, 2),
                   cofac(1, 2, 2, 0),
                   cofac(1, 0, 2, 1));

  // Matrix determiant
  const Scalar det((*this)[0].dot(co));

  // Inverse computation
  const Scalar s = Scalar(1.0) / det;
  return Matrix3x3(co[0] * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
                   co[1] * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
                   co[2] * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
}


inline Matrix3x3 Matrix3x3::transposeTimes(const Matrix3x3& m) const
{
  return Matrix3x3(
  m_el[0][0] * m[0][0] + m_el[1][0] * m[1][0] + m_el[2][0] * m[2][0],
  m_el[0][0] * m[0][1] + m_el[1][0] * m[1][1] + m_el[2][0] * m[2][1],
  m_el[0][0] * m[0][2] + m_el[1][0] * m[1][2] + m_el[2][0] * m[2][2],
  m_el[0][1] * m[0][0] + m_el[1][1] * m[1][0] + m_el[2][1] * m[2][0],
  m_el[0][1] * m[0][1] + m_el[1][1] * m[1][1] + m_el[2][1] * m[2][1],
  m_el[0][1] * m[0][2] + m_el[1][1] * m[1][2] + m_el[2][1] * m[2][2],
  m_el[0][2] * m[0][0] + m_el[1][2] * m[1][0] + m_el[2][2] * m[2][0],
  m_el[0][2] * m[0][1] + m_el[1][2] * m[1][1] + m_el[2][2] * m[2][1],
  m_el[0][2] * m[0][2] + m_el[1][2] * m[1][2] + m_el[2][2] * m[2][2]);
}


inline Matrix3x3 Matrix3x3::timesTranspose(const Matrix3x3& m) const
{
  return Matrix3x3(m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
                   m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
                   m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));
}


inline Matrix3x3& Matrix3x3::scale(const Vector3& v)
{
  setValue(m_el[0][0] * v[0], m_el[0][1] * v[1], m_el[0][2] * v[2],
           m_el[1][0] * v[0], m_el[1][1] * v[1], m_el[1][2] * v[2],
           m_el[2][0] * v[0], m_el[2][1] * v[1], m_el[2][2] * v[2]);
  return *this;
}


// ACCESS

inline Vector3& Matrix3x3::at(size_t n)
{
  util::Assert(n < 3, std::range_error("Index out of range"));
  return m_el[n];
}


inline const Vector3& Matrix3x3::at(size_t n) const
{
  util::Assert(n < 3, std::range_error("Index out of range"));
  return m_el[n];
}

inline Vector3 Matrix3x3::getScaling() const
{
  return Vector3
  (m_el[0][0] * m_el[0][0] + m_el[1][0] * m_el[1][0] + m_el[2][0] * m_el[2][0],
   m_el[0][1] * m_el[0][1] + m_el[1][1] * m_el[1][1] + m_el[2][1] * m_el[2][1],
   m_el[0][2] * m_el[0][2] + m_el[1][2] * m_el[1][2] + m_el[2][2] * m_el[2][2]);
}


inline
void Matrix3x3::setValue(const Scalar& xx, const Scalar& xy, const Scalar& xz,
                         const Scalar& yx, const Scalar& yy, const Scalar& yz,
                         const Scalar& zx, const Scalar& zy, const Scalar& zz)
{
  m_el[0][0] = xx; 
  m_el[0][1] = xy; 
  m_el[0][2] = xz;
  m_el[1][0] = yx; 
  m_el[1][1] = yy; 
  m_el[1][2] = yz;
  m_el[2][0] = zx; 
  m_el[2][1] = zy; 
  m_el[2][2] = zz;
}


inline void Matrix3x3::setIdentity()
{
  setValue(Scalar(1.0), Scalar(0.0), Scalar(0.0),
           Scalar(0.0), Scalar(1.0), Scalar(0.0),
           Scalar(0.0), Scalar(0.0), Scalar(1.0));
}


// PRIVATE METHODS

inline Scalar Matrix3x3::cofac(size_t r1, size_t c1,
                               size_t r2, size_t c2) const
{
  return (m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1]);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Vector3 operator*(const Matrix3x3& m,
                         const Vector3&   v)
{
  return Vector3(m[0].dot(v),
                 m[1].dot(v),
                 m[2].dot(v));
}


inline Vector3 operator*(const Vector3&   v,
                         const Matrix3x3& m)
{
  return Vector3(m.tdot(0, v),
                 m.tdot(1, v),
                 m.tdot(2, v));
}


inline Matrix3x3 operator*(const Matrix3x3& m1,
                           const Matrix3x3& m2)
{
  Matrix3x3 m(m1);
  return m *= m2;
}


inline std::ostream& operator<<(std::ostream&    os,
                                const Matrix3x3& m)
{
  os << m[0] << '\n'
     << m[1] << '\n'
     << m[2];
  return os;
}


// FUNCTIONS

inline Matrix3x3 abs(const Matrix3x3& m)
{
  return Matrix3x3(abs(m[0][0]), abs(m[0][1]), abs(m[0][2]),
                   abs(m[1][0]), abs(m[1][1]), abs(m[1][2]),
                   abs(m[2][0]), abs(m[2][1]), abs(m[2][2]));
}


inline Matrix3x3 scale(const Matrix3x3 m,
                       const Vector3&  v)
{
  Matrix3x3 m1(m);
  return m1.scale(v);
}


inline Scalar determinant(const Matrix3x3& m)
{
  return m.determinant();
}


inline Matrix3x3 adjoint(const Matrix3x3& m)
{
  return m.adjoint();
}


inline Matrix3x3 transpose(const Matrix3x3& m)
{
  return m.transpose();
}


inline Matrix3x3 inverse(const Matrix3x3& m)
{
  return m.inverse();
}


inline Matrix3x3 transposeTimes(const Matrix3x3& m1,
                                const Matrix3x3& m2)
{
  return m1.transposeTimes(m2);
}


inline Matrix3x3 timesTranspose(const Matrix3x3& m1,
                                const Matrix3x3& m2)
{
  return m1.timesTranspose(m2);
}


} // mt

#endif // MT_MATRIX3X3_H
