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
#ifndef MT_BASIC_SCALAR_H
#define MT_BASIC_SCALAR_H

// C++ STANDARD HEADERS
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/basic_scalar_traits.h>
#include <mt/exception.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
///
/// \brief Floating point number with tolerance-based comparison operators.
///
/// The BasicScalar class is used when floating point numbers comparisons depend
/// on a tolerance.
/// The class wraps a floating point datatype -the template parameter- and
/// provides an interface with the usual operations for the original datatype
/// such as operators, mathematical functions, etc.
///
/// The equality and less-than operators -hence the other operators that derive
/// from these two- take into account a tolerance while performing the
/// respective tests. The user can specify whether an absolute or relative
/// tolerance is to be used, as well as its value by means of the
/// BasicScalarTraits traits class.
/// It is important to notice that such information is stored in static
/// variables, so a change in the tolerance type or value will affect \e all
/// instances of the class (but not the results of previously performed tests).
///
/// The default user-defined tolerance is absolute and equal to 0.0001.
///
/// Even if the user-defined tolerance is set to zero, the \e epsilon associated
/// to the current floating-point representation is taken into account in the
/// tests.
/// This is done to compensate for numbers that cannot be expressed exactly
/// using the current representation, case in which the difference between the
/// real value and its representation is upper bounded by \e epsilon.
///
/// The criteria used for comparing two scalars \f$ s_1 \f$ and \f$ s_2 \f$
/// is the following:
///
/// - \e Equality: \f$ d \leq tol + 2 \epsilon \f$
///
/// - \e Less-than: \f$ d > tol + 2 \epsilon \f$
///
/// where
///
/// - \f$ tol \f$ is the user-defined tolerance (defined in the traits class)
///
/// - \f$ \epsilon \f$ is the epsilon associated to the current floating-point
/// representation
///
/// - \f$ d \f$ equals \f$ \mid s_2 - s_1 \mid \f$ for absolute tolerance based
/// tests, and
/// \f$ \mid s_2 - s_1 \mid / \mathrm{min}(\mid s_1 \mid, \mid s_2 \mid) \f$
/// for relative tolerance based tests when
/// \f$ \mathrm{min}(\mid s_1 \mid, \mid s_2 \mid) \geq \epsilon \f$.
/// If \f$ \mathrm{min}(\mid s_1 \mid, \mid s_2 \mid) < \epsilon \f$ an absolute
/// tolerance test is performed even if the tolerance mode is set to be
/// relative. This prevents the problem of comparing a scalar against a
/// near-zero value using a relative tolerance.
///
/// A very important remark is that tolerance-based tests such as the ones
/// presented above are \e not transitive, that is, the following does not
/// necessarily hold:
/// if \f$ (a = b)\f$ and \f$ (b = c) \f$, then \f$ (a = c) \f$.
///
/// As a particular example, if \f$ a = 0.9 \f$, \f$ b = 1.0 \f$, \f$ c = 1.1 \f$,
/// and an absolute tolerance \f$ tol = 0.1 \f$ is used, it turns out that
/// \f$ (a = b) \f$ and \f$ (b = c) \f$, but \f$ (a \neq c) \f$!!!.
///
///
/// This is an example of how to use the BasicScalar class:
///
/// \code
/// using namespace mt;
///
/// // Typedefs are a good idea to make code more readable
/// typedef BasicScalarTraits<float> ScalarTraitsF;
/// typedef BasicScalarTraits<double> ScalarTraitsD;
///
/// typedef BasicScalar<float> ScalarF;
/// typedef BasicScalar<double> ScalarD;
///
/// // Tests using floats and a relative tolerance of 1e-2
/// ScalarTraitsF::setTolType(ScalarTraitsF::RELATIVE);
/// ScalarTraitsF::setTol(1e-2);
/// ScalarF f1(100.0f);
/// ScalarF f2(99.0f);
///
/// bool test;
/// test = (f1 == f2); // test = true
/// test = (f1 <  f2); // test = false
///
/// // Tests using doubles and an absolute tolerance of 1e-3
/// ScalarTraitsD::setTolType(ScalarTraitsD::ABSOLUTE);
/// ScalarTraitsD::setTol(1e-3);
/// ScalarD d1(1.000);
/// ScalarD d2(1.0011);
///
/// test = (d1 == d2); // test = false
/// test = (d1 <  d2); // test = true
///
/// \endcode

template<class T>
class BasicScalar
{
public:

// LIFECYCLE

  // Compiler generated default constructor for a BasicScalar<T> is being used

  /// Constructor for input value.
  BasicScalar(const T& x = 0.0);

  // Compiler generated copy constructor for a BasicScalar<T> is being used

  /// Copy constructor.
  template<class T2>
  BasicScalar(const BasicScalar<T2>& s);


  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator for a BasicScalar<T> is being used

  /// Assignment operator.
  /// \param s The value to assign to this object.
  /// \return A reference to this object.
  template<class T2>
  BasicScalar<T>& operator=(const BasicScalar<T2>& s);


  /// Assignment operator.
  /// \param x The value to assign to this object.
  /// \return A reference to this object.
  BasicScalar<T>& operator=(const T& x);

  BasicScalar<T>& operator+=(const BasicScalar<T>& s);
  BasicScalar<T>& operator-=(const BasicScalar<T>& s);
  BasicScalar<T>& operator*=(const BasicScalar<T>& s);
  BasicScalar<T>& operator/=(const BasicScalar<T>& s);

  bool operator==(const BasicScalar<T>& s) const;
  bool operator< (const BasicScalar<T>& s) const;
  bool operator!=(const BasicScalar<T>& s) const;
  bool operator> (const BasicScalar<T>& s) const;
  bool operator<=(const BasicScalar<T>& s) const;
  bool operator>=(const BasicScalar<T>& s) const;


// ACCESS

  /// Gets the scalar value.
  T getValue() const;

  /// Gets a reference to the scalar value.
  T& getRef();

  /// Gets a constant reference to the scalar value.
  const T& getRef() const;

  /// Sets the scalar's value.
  void setValue(const T& x);


private:

// MEMBERS

  /// Floating-point value.
  T m_val;


// METHODS

  /// Absolute tolerance based equality test.
  bool absEquality(const BasicScalar<T>& s) const;

  /// Absolute tolerance based less-than test.
  bool absLessThan(const BasicScalar<T>& s) const;

  /// Relative tolerance based equality test.
  bool relEquality(const BasicScalar<T>& s) const;

  /// Relative tolerance based less-than test.
  bool relLessThan(const BasicScalar<T>& s) const;


// FRIENDS

  template<class T1>
  friend std::ostream& operator<<(std::ostream&          os,
                                  const BasicScalar<T1>& s);

  template<class T1>
  friend std::istream& operator>>(std::istream&          is,
                                  const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> min(const BasicScalar<T1>& s1,
                             const BasicScalar<T1>& s2);

  template<class T1>
  friend BasicScalar<T1> max(const BasicScalar<T1>& s1,
                             const BasicScalar<T1>& s2);

  template<class T1>
  friend BasicScalar<T1> abs(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> ceil(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> floor(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> sqrt(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> exp(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> log(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> log10(const BasicScalar<T1>& s);

  template<class T1, class T2>
  friend BasicScalar<T1> pow(const BasicScalar<T1>& s1,
                             const BasicScalar<T2>& s2);

  template<class T1>
  friend BasicScalar<T1> sin(const BasicScalar<T1>& s);

  template<class T1>
  friend BasicScalar<T1> cos(const BasicScalar<T1>& s);

  template<class T2>
  friend BasicScalar<T2> tan(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> sinh(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> cosh(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> tanh(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> asin(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> acos(const BasicScalar<T2>& s);

  template<class T2>
  friend BasicScalar<T2> atan(const BasicScalar<T2>& s);

  template<class T1, class T2>
  friend BasicScalar<T1> atan2(const BasicScalar<T1>& s1,
                               const BasicScalar<T2>& s2);

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

template<class T>
BasicScalar<T> operator+(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> operator-(const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> operator+(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2);


// template<class T>
// BasicScalar<T> operator+(const BasicScalar<T>& s,
//                          const T&              x);
// 
// 
// template<class T>
// BasicScalar<T> operator+(const T&              x,
//                          const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> operator-(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2);


// template<class T>
// BasicScalar<T> operator-(const BasicScalar<T>& s,
//                          const T&              x);
// 
// 
// template<class T>
// BasicScalar<T> operator-(const T&              x,
//                          const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> operator*(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2);


// template<class T>
// BasicScalar<T> operator*(const BasicScalar<T>& s,
//                          const T&              x);
// 
// 
// template<class T>
// BasicScalar<T> operator*(const T&              x,
//                          const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> operator/(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2);


// template<class T>
// BasicScalar<T> operator/(const BasicScalar<T>& s,
//                          const T&              x);
// 
// 
// template<class T>
// BasicScalar<T> operator/(const T&              x,
//                          const BasicScalar<T>& s);

/// Output operator.
///
/// Output is performed using a fixed width and values are rouded to the
/// current tolerance value (tolerance is treated as absolute).
template<class T>
std::ostream& operator<<(std::ostream&         os,
                         const BasicScalar<T>& s);


template<class T>
std::istream& operator>>(std::istream&         is,
                         const BasicScalar<T>& s);


// FUNCTIONS

template<class T>
BasicScalar<T> min(const BasicScalar<T>& s1,
                   const BasicScalar<T>& s2);


template<class T>
BasicScalar<T> max(const BasicScalar<T>& s1,
                   const BasicScalar<T>& s2);


template<class T>
BasicScalar<T> abs(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> ceil(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> floor(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> sqrt(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> exp(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> log(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> log10(const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> pow(const BasicScalar<T1>& s1,
                    const BasicScalar<T2>& s2);


template<class T>
BasicScalar<T> sin(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> cos(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> tan(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> sinh(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> cosh(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> tanh(const BasicScalar<T>& s);

template<class T>
BasicScalar<T> asin(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> acos(const BasicScalar<T>& s);


template<class T>
BasicScalar<T> atan(const BasicScalar<T>& s);


template<class T1, class T2>
BasicScalar<T1> atan2(const BasicScalar<T1>& s1,
                      const BasicScalar<T2>& s2);


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
BasicScalar<T> normalize(const BasicScalar<T>& s,
                         const BasicScalar<T>& lower,
                         const BasicScalar<T>& upper,
                               BasicScalar<T>& cycles);

/// Same as above, but without the cycles parameter.
template<class T>
BasicScalar<T> normalize(const BasicScalar<T>& s,
                         const BasicScalar<T>& lower,
                         const BasicScalar<T>& upper);


/// Saturates input value \a s to a reprersentation belonging to the interval
/// \a [lower, \a upper].
template<class T>
BasicScalar<T> saturate(const BasicScalar<T>& s,
                        const BasicScalar<T>& lower,
                        const BasicScalar<T>& upper);


/// Rounds the current value to the precision defined by the absolute value of
/// \a tol.
/// \param s Value to be rounded.
/// \param tol Rounding tolerance. If omitted, unity tolerance is used.
/// \return Rounded value.
template<class T>
BasicScalar<T> round(const BasicScalar<T>& s,
                     const BasicScalar<T>& tol = BasicScalar<T>(1.0));

/// Gets scalar value.
template<class T>
T getValue(const BasicScalar<T> s);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

template<class T> inline
BasicScalar<T>::BasicScalar(const T& x) : m_val(x) {}


template<class T1> template<class T2> inline
BasicScalar<T1>::BasicScalar(const BasicScalar<T2>& s) : m_val(s.getValue()){}


// OPERATORS

template<class T1> template<class T2> inline
BasicScalar<T1>& BasicScalar<T1>::operator=(const BasicScalar<T2>& s)
{
  m_val = s.getValue();
  return *this;
}


template<class T> inline
BasicScalar<T>& BasicScalar<T>::operator=(const T& x)
{
  m_val = x;
  return *this;
}


template<class T> inline
BasicScalar<T>& BasicScalar<T>::operator+=(const BasicScalar<T>& s)
{
  m_val += s.m_val;
  return *this;
}


template<class T> inline
BasicScalar<T>& BasicScalar<T>::operator-=(const BasicScalar<T>& s)
{
  m_val -= s.m_val;
  return *this;
}


template<class T> inline
BasicScalar<T>& BasicScalar<T>::operator*=(const BasicScalar<T>& s)
{
  m_val *= s.m_val;
  return *this;
}


template<class T> inline
BasicScalar<T>& BasicScalar<T>::operator/=(const BasicScalar<T>& s)
{
  util::Assert(std::abs(s.m_val) >=  std::numeric_limits<T>::epsilon(),
               ZeroDivide());
  m_val /= s.m_val;
  return *this;
}


template<class T> inline
bool BasicScalar<T>::operator==(const BasicScalar<T>& s) const
{
  const bool test =
  BasicScalarTraits<T>::getTolType() == BasicScalarTraits<T>::ABSOLUTE;
  return test ? absEquality(s) : relEquality(s);
}


template<class T> inline
bool BasicScalar<T>::operator<(const BasicScalar<T>& s) const
{
  const bool test =
  BasicScalarTraits<T>::getTolType() == BasicScalarTraits<T>::ABSOLUTE;
  return test ? absLessThan(s) : relLessThan(s);
}


template<class T> inline
bool BasicScalar<T>::operator!=(const BasicScalar<T>& s) const
{
  return  !(*this == s);
}


template<class T> inline
bool BasicScalar<T>::operator>(const BasicScalar<T>& s) const
{
  return s < *this;
}


template<class T> inline
bool BasicScalar<T>::operator<=(const BasicScalar<T>& s) const
{
  return (*this < s) || (*this == s);
}


template<class T> inline
bool BasicScalar<T>::operator>=(const BasicScalar<T>& s) const
{
  return (*this > s) || (*this == s);
}


// ACCESS

template<class T> inline
T BasicScalar<T>::getValue() const
{
  return m_val;
}


template<class T> inline
T& BasicScalar<T>::getRef()
{
  return m_val;
}


template<class T> inline
const T& BasicScalar<T>::getRef() const
{
  return m_val;
}


template<class T> inline
void BasicScalar<T>::setValue(const T& x)
{
  m_val = x;
}


// PRIVATE METHODS

template<class T> inline
bool BasicScalar<T>::absEquality(const BasicScalar<T>& s) const
{
  const T diff = std::abs(m_val - s.m_val);
  return  diff <= BasicScalarTraits<T>::getTestTol();
}


template<class T> inline
bool BasicScalar<T>::absLessThan(const BasicScalar<T>& s) const
{
  const T diff = s.m_val - m_val;
  return diff > BasicScalarTraits<T>::getTestTol();
}


template<class T> inline
bool BasicScalar<T>::relEquality(const BasicScalar<T>& s) const
{
  T ref  = std::min(std::abs(s.m_val), std::abs(m_val));

  if (!(ref > std::numeric_limits<T>::epsilon()))
  {
    // Absolute tolerance is used for comparing values smaller than unity
    ref = 1.0;
  }
  const T diff = std::abs(s.m_val - m_val);
  return (diff / ref) <= BasicScalarTraits<T>::getTestTol();
}


template<class T> inline
bool BasicScalar<T>::relLessThan(const BasicScalar<T>& s) const
{
  T ref  = std::min(std::abs(s.m_val), std::abs(m_val));
  if (!(ref > std::numeric_limits<T>::epsilon()))
  {
    // Absolute tolerance is used for comparing values smaller than unity
    ref = 1.0;
  }
  const T diff = s.m_val - m_val;
  return (diff / ref) > BasicScalarTraits<T>::getTestTol();
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

template<class T> inline
BasicScalar<T> operator+(const BasicScalar<T>& s)
{
  return s;
}


template<class T> inline
BasicScalar<T> operator-(const BasicScalar<T>& s)
{
  const BasicScalar<T> s1(-s.getValue());
  return s1;
}


template<class T1, class T2> inline
BasicScalar<T1> operator+(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2)
{
  BasicScalar<T1> s(s2);
  return s += s1;
}


// template<class T> inline
// BasicScalar<T> operator+(const BasicScalar<T>& s,
//                          const T&              x)
// {
//   BasicScalar<T> s1(x);
//   return s1 += s;
// }
// 
// 
// template<class T> inline
// BasicScalar<T> operator+(const T&              x,
//                          const BasicScalar<T>& s)
// {
//   BasicScalar<T> s1(x);
//   return s1 += s;
// }


template<class T1, class T2> inline
BasicScalar<T1> operator-(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2)
{
  BasicScalar<T1> s3(s1);
  const BasicScalar<T1> s4(s2);
  return s3 -= s4;
}


// template<class T> inline
// BasicScalar<T> operator-(const BasicScalar<T>& s,
//                          const T&              x)
// {
//   BasicScalar<T> s1(s);
//   return s1 -= x;
// }
// 
// 
// template<class T> inline
// BasicScalar<T> operator-(const T&              x,
//                          const BasicScalar<T>& s)
// {
//   BasicScalar<T> s1(x);
//   return s1 -= s;
// }


template<class T1, class T2> inline
BasicScalar<T1> operator*(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2)
{
  BasicScalar<T1> s(s2);
  return s *= s1;
}


// template<class T> inline
// BasicScalar<T> operator*(const BasicScalar<T>& s,
//                          const T&              x)
// {
//   BasicScalar<T> s1(x);
//   return s1 *= s;
// }
// 
// 
// template<class T> inline
// BasicScalar<T> operator*(const T&              x,
//                          const BasicScalar<T>& s)
// {
//   BasicScalar<T> s1(x);
//   return s1 *= s;
// }


template<class T1, class T2> inline
BasicScalar<T1> operator/(const BasicScalar<T1>& s1,
                          const BasicScalar<T2>& s2)
{
  BasicScalar<T1> s3(s1);
  const BasicScalar<T1> s4(s2);
  return s3 /= s4;
}


// template<class T> inline
// BasicScalar<T> operator/(const BasicScalar<T>& s,
//                          const T&              x)
// {
//   BasicScalar<T> s1(s);
//   return s1 /= x;
// }
// 
// 
// template<class T> inline
// BasicScalar<T> operator/(const T&              x,
//                          const BasicScalar<T>& s)
// {
//   BasicScalar<T> s1(x);
//   return s1 /= s;
// }


template<class T> inline
std::ostream& operator<<(std::ostream&         os,
                         const BasicScalar<T>& s)
{
  const size_t width = BasicScalarTraits<T>::getWidth();
  const BasicScalar<T> tol = BasicScalarTraits<T>::getTol();
  return os << std::setw(width) << round(s, tol).m_val;
}


template<class T> inline
std::istream& operator>>(std::istream&         is,
                         const BasicScalar<T>& s)
{
  return is >> s.m_val;
}


// FUNCTIONS

template<class T> inline
BasicScalar<T> min(const BasicScalar<T>& s1,
                   const BasicScalar<T>& s2)
{
  const T x = std::min(s1.m_val, s2.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> max(const BasicScalar<T>& s1,
                   const BasicScalar<T>& s2)
{
  const T x = std::max(s1.m_val, s2.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> abs(const BasicScalar<T>& s)
{
  const T x = std::abs(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> ceil(const BasicScalar<T>& s)
{
  const T x = std::ceil(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> floor(const BasicScalar<T>& s)
{
  const T x = std::floor(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> sqrt(const BasicScalar<T>& s)
{
  util::Assert((s >= 0.0), Exception("Negative sqrt parameter"));
  const T x = std::sqrt(std::abs(s.m_val));
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> exp(const BasicScalar<T>& s)
{
  const T x = std::exp(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> log(const BasicScalar<T>& s)
{
  util::Assert((s >= 0.0), Exception("Negative log parameter"));
  const T x = std::log(std::abs(s.m_val));
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> log10(const BasicScalar<T>& s)
{
  util::Assert((s >= 0.0), Exception("Negative log10 parameter"));
  const T x = std::log10(std::abs(s.m_val));
  return BasicScalar<T>(x);
}


template<class T1, class T2> inline
BasicScalar<T1> pow(const BasicScalar<T1>& s1,
                    const BasicScalar<T2>& s2)
{
  const T1 x = std::pow(s1.m_val, static_cast<T1>(s2.m_val));
  return BasicScalar<T1>(x);
}


template<class T> inline
BasicScalar<T> sin(const BasicScalar<T>& s)
{
  const T x = std::sin(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> cos(const BasicScalar<T>& s)
{
  const T x = std::cos(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> tan(const BasicScalar<T>& s)
{
  const T x = std::tan(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> sinh(const BasicScalar<T>& s)
{
  const T x = std::sinh(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> cosh(const BasicScalar<T>& s)
{
  const T x = std::cosh(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> tanh(const BasicScalar<T>& s)
{
  const T x = std::tanh(s.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> asin(const BasicScalar<T>& s)
{
  const BasicScalar<T> s1(saturate(s,
                                   BasicScalar<T>(-1.0),
                                   BasicScalar<T>( 1.0)));
  const T x = std::asin(s1.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> acos(const BasicScalar<T>& s)
{
  const BasicScalar<T> s1(saturate(s,
                                   BasicScalar<T>(-1.0),
                                   BasicScalar<T>( 1.0)));
  const T x = std::acos(s1.m_val);
  return BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> atan(const BasicScalar<T>& s)
{
  const T x = std::atan(s.m_val);
  return BasicScalar<T>(x);
}


template<class T1, class T2> inline
BasicScalar<T1> atan2(const BasicScalar<T1>& s1,
                      const BasicScalar<T2>& s2)
{
  T1 t1 = s1.m_val;
  T1 t2 = static_cast<T1>(s2.m_val);

  // Prevents sign problems for near-zero values
  if (s1 == BasicScalar<T1>(0.0))
  {
    t1 = static_cast<T1>(0.0);
  }

  if (s2 == BasicScalar<T2>(0.0))
  {
    t2 = static_cast<T1>(0.0);
  }

  const T1 x = std::atan2(t1, t2);
  return BasicScalar<T1>(x);
}


template<class T> inline
BasicScalar<T> normalize(const BasicScalar<T>& s,
                         const BasicScalar<T>& lower,
                         const BasicScalar<T>& upper,
                               BasicScalar<T>& cycles)
{
  util::Assert(lower < upper,
               Exception("Normalizing interval is ill defined"));

  BasicScalar<T> s_norm(s);
  if (s >= lower && s < upper)
  {
    cycles = BasicScalar<T>(0.0);
  }
  else
  {
    const BasicScalar<T> diff(upper - lower);
    cycles = (s.getValue() < lower.getValue()) ?
    ceil((s - upper) / diff) : floor((s - lower) / diff);
    s_norm -= (diff * cycles);
  }
  return s_norm;
}


template<class T> inline
BasicScalar<T> normalize(const BasicScalar<T>& s,
                         const BasicScalar<T>& lower,
                         const BasicScalar<T>& upper)
{
  BasicScalar<T> cycles;
  return normalize(s, lower, upper, cycles);
}


template<class T> inline
BasicScalar<T> saturate(const BasicScalar<T>& s,
                        const BasicScalar<T>& lower,
                        const BasicScalar<T>& upper)
{
  util::Assert(lower < upper,
               Exception("Saturation interval is ill defined"));

  BasicScalar<T> s_sat(s);
  if (s.getValue() < lower.getValue())
  {
    s_sat = lower;
  }
  else if (s.getValue() > upper.getValue())
  {
    s_sat = upper;
  }
  return s_sat;
}


template<class T> inline
BasicScalar<T> round(const BasicScalar<T>& s,
                     const BasicScalar<T>& tol)
{
  if (tol > std::numeric_limits<T>::epsilon())
  {
    // Tolerance is greater than epsilon
    const T abs_tol = std::abs(tol.getValue());
    const T x = (s.getValue() + std::numeric_limits<T>::epsilon()) / abs_tol;
    return BasicScalar<T>(std::floor(x + 0.5) * abs_tol);
  }
  else
  {
    // Tolerance is less or equal than epsilon
    return s;
  }
}

template<class T> inline
T getValue(const BasicScalar<T> s)
{
  return s.getValue();
}

} // mt

#endif // MT_BASIC_SCALAR_H
