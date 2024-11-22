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
#ifndef MT_EMPTY_H
#define MT_EMPTY_H

// C++ STANDARD HEADERS
#include <iostream>

// BOOST LIBRARY HEADERS
#ifdef MT_USE_BOOST
  #include <boost/variant/static_visitor.hpp>
#endif


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

/////////////////////////////// CLASS DEFINITION /////////////////////////////

///\brief Empty class for representing an unititialized variant data structure.
///
/// The class meets the requirements of the following concepts:
/// - Default constructible
/// - Copy constructible
/// - Has a no-throw destructor
/// - Assignable
/// - Equality comparable
/// - Less-than comparable
/// - Output streamable

struct Empty
{
  Empty(){}
};

#ifdef MT_USE_BOOST
class EmptyVisitor : public boost::static_visitor<bool>
{
public:
  bool operator()(const Empty&) const {return true;}
  template <class T>
  bool operator()(const T&)     const {return false;}
};
#endif

/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

inline bool operator==(const Empty&, const Empty&) {return true;}

inline bool operator<(const Empty&, const Empty&) {return false;}

inline std::ostream& operator<<(std::ostream& os, const Empty&) {return os;}

} // mt

#endif // MT_EMPTY_H
