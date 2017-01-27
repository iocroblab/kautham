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
#ifndef MT_BASIC_SCALAR_UTILITY_H
#define MT_BASIC_SCALAR_UTILITY_H

// MT LIBRARY HEADERS
#include <mt/basic_scalar.h>

/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

/// \file basic_scalar_utility.h
/// This file contains overloads for the four basic arithmetic operators
/// (+ - * /) between a BasicScalar instance and the following POD types:
/// float, double, and int.

// ADDITION

template<class T>
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const float&          x);

template<class T>
BasicScalar<T> operator+(const float&          x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const double&         x);

template<class T>
BasicScalar<T> operator+(const double&         x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const int&            x);

template<class T>
BasicScalar<T> operator+(const int&            x,
                         const BasicScalar<T>& s);


// SUBSTRACTION

template<class T>
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const float&          x);

template<class T>
BasicScalar<T> operator-(const float&          x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const double&         x);

template<class T>
BasicScalar<T> operator-(const double&         x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const int&            x);

template<class T>
BasicScalar<T> operator-(const int&            x,
                         const BasicScalar<T>& s);


// MULTIPLICATION

template<class T>
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const float&          x);

template<class T>
BasicScalar<T> operator*(const float&          x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const double&         x);

template<class T>
BasicScalar<T> operator*(const double&         x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const int&            x);

template<class T>
BasicScalar<T> operator*(const int&            x,
                         const BasicScalar<T>& s);


// DIVISION

template<class T>
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const float&          x);

template<class T>
BasicScalar<T> operator/(const float&          x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const double&         x);

template<class T>
BasicScalar<T> operator/(const double&         x,
                         const BasicScalar<T>& s);

template<class T>
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const int&            x);

template<class T>
BasicScalar<T> operator/(const int&            x,
                         const BasicScalar<T>& s);


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// ADDITION

template<class T> inline
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const float&          x)
{
  return s + BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator+(const float&          x,
                         const BasicScalar<T>& s)
{
  return s + BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const double&         x)
{
  return s + BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator+(const double&         x,
                         const BasicScalar<T>& s)
{
  return s + BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator+(const BasicScalar<T>& s,
                         const int&            x)
{
  return s + BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator+(const int&            x,
                         const BasicScalar<T>& s)
{
  return s + BasicScalar<T>(x);
}


// SUBSTRACTION

template<class T> inline
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const float&          x)
{
  return s - BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator-(const float&          x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) - s;
}


template<class T> inline
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const double&         x)
{
  return s - BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator-(const double&         x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) - s;
}


template<class T> inline
BasicScalar<T> operator-(const BasicScalar<T>& s,
                         const int&            x)
{
  return s - BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator-(const int&            x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) - s;
}


// MULTIPLICATION

template<class T> inline
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const float&          x)
{
  return s * BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator*(const float&          x,
                         const BasicScalar<T>& s)
{
  return s * BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const double&         x)
{
  return s * BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator*(const double&         x,
                         const BasicScalar<T>& s)
{
  return s * BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator*(const BasicScalar<T>& s,
                         const int&            x)
{
  return s * BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator*(const int&            x,
                         const BasicScalar<T>& s)
{
  return s * BasicScalar<T>(x);
}


// DIVISION

template<class T> inline
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const float&          x)
{
  return s / BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator/(const float&          x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) / s;
}


template<class T> inline
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const double&         x)
{
  return s / BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator/(const double&         x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) / s;
}


template<class T> inline
BasicScalar<T> operator/(const BasicScalar<T>& s,
                         const int&            x)
{
  return s / BasicScalar<T>(x);
}


template<class T> inline
BasicScalar<T> operator/(const int&            x,
                         const BasicScalar<T>& s)
{
  return BasicScalar<T>(x) / s;
}


} // mt

#endif // MT_BASIC_SCALAR_UTILITY_H
