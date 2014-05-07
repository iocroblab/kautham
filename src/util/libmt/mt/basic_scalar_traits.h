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
#ifndef MT_BASIC_SCALAR_TRAITS_H
#define MT_BASIC_SCALAR_TRAITS_H

// C++ STANDARD HEADERS
#include <limits>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

// USING DECLARATIONS/DIRECTIVES

/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \brief Traits class for BasicScalar tolerance-based floating-point class.


template<class T>
class BasicScalarTraits
{
public:

// TYPES

  /// Tolerance type values.
  enum ToleranceType
  {
    ABSOLUTE,
    RELATIVE
  };


// ACCESS

  /// Gets the user-defined tolerance.
  static T getTol();

  /// Gets the tolerance used for comparing scalars.
  /// This tolerance equals \f$ m\_tol + 2 \epsilon \f$
  static T getTestTol();

  /// Gets the tolerance type used for comparing scalars.
  static typename BasicScalarTraits<T>::ToleranceType getTolType();

  /// Gets width (in characters) the scalar will have when output streamed.
  static size_t getWidth();

  /// Sets the tolerance used for comparing scalars.
  ///
  /// Since only positive tolerance values make sense, the absolute value of
  /// the input value is taken as the tolerance value.
  static void setTol(const T& x);

  /// Sets the tolerance used for comparing scalars.
  static void setTolType(const ToleranceType& type);


private:

// MEMBERS

  /// User-defined tolerance.
  static T m_tol;

  /// Tolerance used for for equality and less-than tests.
  /// This tolerance equals \f$ m\_tol + 2 \epsilon \f$
  static T m_test_tol;

  /// Tolerance type.
  static ToleranceType m_tol_type;

  /// Width (in characters) the scalar will have when output streamed.
  static size_t m_width;
};


/////////////////////////////// INLINE METHODS ///////////////////////////////

template<class T> inline
T BasicScalarTraits<T>::getTol()
{
  return m_tol;
}

template<class T> inline
T BasicScalarTraits<T>::getTestTol()
{
  return m_test_tol;
}


template<class T> inline
typename BasicScalarTraits<T>::ToleranceType BasicScalarTraits<T>::getTolType()
{
  return m_tol_type;
}


template<class T> inline
size_t BasicScalarTraits<T>::getWidth()
{
  return m_width;
}


template<class T> inline
void BasicScalarTraits<T>::setTol(const T& x)
{
  m_tol = std::abs(x);
  m_test_tol = m_tol + 2.0 * std::numeric_limits<T>::epsilon();
}


template<class T> inline
void BasicScalarTraits<T>::setTolType(const ToleranceType& type)
{
  m_tol_type = type;
}


// STATIC MEMBER INITIALIZATION

template<class T>
T BasicScalarTraits<T>::m_tol = static_cast<T>(0.0001);

template<class T>
T BasicScalarTraits<T>::m_test_tol = 2.0 * std::numeric_limits<T>::epsilon() +
                                     BasicScalarTraits<T>::m_tol;

template<class T>
typename BasicScalarTraits<T>::ToleranceType
BasicScalarTraits<T>::m_tol_type = BasicScalarTraits<T>::ABSOLUTE;

template<class T>
size_t BasicScalarTraits<T>::m_width = 6;

} // mt

#endif // MT_BASIC_SCALAR_TRAITS_H
