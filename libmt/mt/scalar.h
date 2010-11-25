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
#ifndef MT_SCALAR_H
#define MT_SCALAR_H

// C++ STANDARD HEADERS
#include <algorithm>
#include <cmath>

// MT LIBRARY HEADERS
#include <mt/exception.h>

#ifdef MT_USE_BASIC_SCALAR

  #include <mt/basic_scalar.h>
  #include <mt/basic_scalar_utility.h>

#endif


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

// USING DECLARATIONS/DIRECTIVES
using std::min;
using std::max;

using std::floor;
using std::ceil;

using std::abs;
using std::pow;
using std::sqrt;

using std::exp;
using std::log;
using std::log10;

using std::sin;
using std::cos;
using std::tan;

using std::sinh;
using std::cosh;
using std::tanh;

using std::acos;
using std::asin;
using std::atan;
using std::atan2;

/// \file scalar.h
/// \ingroup basic
/// This file contains a typedef for setting the scalar quantity representation
/// to be used in the library. It is set by default to \a float, but other
/// representations such as \a double and the BasicScalar<T> type provided
/// in this library can be used as well.
/// For example, for using \a doubles, the macro \a MT_USE_DOUBLES should be
/// defined; for using BasicScalar<float>, the macro \a MT_USE_BASIC_SCALAR
/// should be defined; and for using BasicScalar<float>, both
/// \a MT_USE_DOUBLES and \a MT_USE_BASIC_SCALAR should be defined.
///
/// Additionally, a few convenience methods for operating on scalar quantities
/// are provided.
///
/// By setting \a MT_USE_BASIC_SCALAR, numeric checks are enabled
/// throughout the library that increase its robustness.
/// As examples, when constructing a plane with three points, it is checked
/// that they are not collinear, and when setting a unit quaternion from a
/// rotation matrix, its determinant is checked to be unity.

/////////////////////////////// TYPE DEFINITION //////////////////////////////

// Value type definition

#ifdef MT_USE_DOUBLES

  typedef double value_t;

#else

  typedef float value_t;

#endif


// Scalar type definition

#ifdef MT_USE_BASIC_SCALAR

  typedef BasicScalarTraits<value_t> ScalarTraits;
  typedef BasicScalar<value_t> Scalar;

#else

  typedef value_t Scalar;

#endif


/////////////////////////////// CONSTANTS ////////////////////////////////////

const Scalar PI(std::atan2(Scalar(0.0), Scalar(-1.0)));
const Scalar HALF_PI(PI / Scalar(2.0));
const Scalar TWO_PI (PI * Scalar(2.0));


/////////////////////////////// FUNCTIONS ////////////////////////////////////

/// Applies the degrees to radians conversion factor.
Scalar degToRad(const Scalar& s);

/// Applies the radians to degrees conversion factor.
Scalar radToDeg(const Scalar& s);

/// Computes the square of the input value.
Scalar sq(const Scalar& s);

/// Computes the sign of the input value.
/// The function returns +1 for positive or zero input values, and -1 for
/// negative input values.
Scalar sgn(const Scalar& s);

/// Normalizes value to a reprersentation belonging to the interval
/// \a [lower, \a upper), so that the following holds:
///
/// old_value = \a cycles * abs(\a upper - \a lower ) + norm_value
///
/// where old_value and norm_value represent the values before and after
/// normalization, respectively.
///
/// The interval can span any length and can include negative numbers.
///
/// As an example, an angle of 810 degrees can be normalized using the
/// interval [0, 360), yielding 90 degrees and 2 cycles, so that
/// 810 = 2 * 360 + 90.
///
/// \param s The value to be normalized.
/// \param lower The lower bound of the normalization interval.
/// \param upper The upper bound of the normalization interval.
/// \param cycles The number of (\a upper - \a lower) size cycles implied by
/// the normalization.
/// \return Normalized value.
template<class T>
T normalize(const T& s,
            const T& lower,
            const T& upper,
                  T& cycles);

/// Same as above, but without the cycles parameter.
template<class T>
T normalize(const T& s,
            const T& lower,
            const T& upper);

/// Saturates value to a reprersentation belonging to the interval
/// \a [lower, \a upper].
template<class T>
T saturate(const T& s,
           const T& lower,
           const T& upper);

/// Rounds the current value to the precision defined by the absolute value of
/// \a tol.
/// \param s Value to be rounded.
/// \param tol Rounding tolerance. If omitted, unity tolerance is used.
/// \return Rounded value.
template<class T>
T round(const T& s,
        const T& tol = T(1.0));

/// Computes the arcsine of \a sin_ang. The returned value is expressed in
/// radians and belongs to the interval [0, 2pi).
///
/// Since there are two possible solutions in the above interval, the returned
/// value is the one closest to \a ref_ang (expressed in radians).
template<class T>
T asinMt(const T& sin_ang,
         const T& ref_ang);

/// Computes the arccosine of \a cos_ang. The returned value is expressed in
/// radians and belongs to the interval [0, 2pi).
///
/// Since there are two possible solutions in the above interval, the returned
/// value is the one closest to \a ref_ang (expressed in radians).
template<class T>
T acosMt(const T& cos_ang,
         const T& ref_ang);

/// Gets scalar value.
template<class T>
T getValue(const T& s);

/////////////////////////////// INLINE FUNCTION DEFINITIONS //////////////////

inline Scalar degToRad(const Scalar& s)
{
  static const Scalar rads_per_deg(PI / Scalar(180.0));
  return Scalar(s * rads_per_deg);
}


inline Scalar radToDeg(const Scalar& s)
{
  static const Scalar degs_per_rad(Scalar(180.0) / PI);
  return Scalar(s * degs_per_rad);
}


inline Scalar sq(const Scalar& s)
{
  return s * s;
}

inline Scalar sgn(const Scalar& s)
{
  if (s >= Scalar(0.0))
  {
    return Scalar(1.0);
  }
  else
  {
    return Scalar(-1.0);
  }
}


template<class T> inline
T normalize(const T& s,
            const T& lower,
            const T& upper,
                  T& cycles)
{
  util::Assert(lower < upper,
               Exception("Normalizing interval is ill defined"));

  T s_norm(s);
  if (s >= lower && s < upper)
  {
    cycles = T(0.0);
  }
  else
  {
    const T diff(upper - lower);
    cycles = (s < lower) ?
    ceil((s - upper) / diff) : floor((s - lower) / diff);
    s_norm -= (diff * cycles);
  }
  return s_norm;
}


template<class T> inline
T normalize(const T& s,
            const T& lower,
            const T& upper)
{
  T cycles;
  return normalize(s, lower, upper, cycles);
}


template<class T> inline
T saturate(const T& s,
           const T& lower,
           const T& upper)
{
  util::Assert(lower < upper,
               Exception("Saturation interval is ill defined"));

  T s_sat(s);
  if (s < lower)
  {
    s_sat = lower;
  }
  else if (s > upper)
  {
    s_sat = upper;
  }
  return s_sat;
}


template<class T> inline
T round(const T& s,
        const T& tol)
{
  const T abs_tol = abs(tol); // Tolerance must be > 0.0
  const T x = s / abs_tol;
  return T(floor(x + 0.5) * abs_tol);
}


template<class T> inline
T asinMt(const T& sin_ang,
         const T& ref_ang)
{
  // Two possible angle values
  T val1(asin(sin_ang));
  T val2(PI - val1);

  // Normalizes val1 and ref_ang value to the [0, 2pi) interval
  val1    = normalize(val1,    T(0.0), TWO_PI);
  Scalar ref_norm(normalize(ref_ang, T(0.0), TWO_PI));

  // Selects the appropiate angle using the the reference ang_ref
  T diff1(abs(ref_norm - val1));
  T diff2(abs(ref_norm - val2));
  return (diff1 < diff2) ? val1 : val2;
}


template<class T> inline
T acosMt(const T& cos_ang,
         const T& ref_ang)
{
  // Two possible angle values
  const T val1(acos(cos_ang));
  const T val2(TWO_PI - val1);

  // Normalizes ref_ang value to the [0, 2pi) interval
  Scalar ref_norm(normalize(ref_ang, T(0.0), TWO_PI));

  // Selects the appropiate angle using the the reference ang_ref
  T diff1(abs(ref_norm - val1));
  T diff2(abs(ref_norm - val2));
  return (diff1 < diff2) ? val1 : val2;
}


template<class T> inline
T getValue(const T& s)
{
  return s;
}

} // mt

#endif // MT_SCALAR_H
