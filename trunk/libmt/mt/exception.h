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
#ifndef MT_EXCEPTION_H
#define MT_EXCEPTION_H

// C++ STANDARD HEADERS
#include <stdexcept>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// Run-time exception class for mt library instances.
class Exception : public std::runtime_error
{
public:
  Exception() : std::runtime_error("mt library error") {}
  Exception(const std::string& what_arg) : std::runtime_error(what_arg) {}
};


/// Division by zero exception.
class ZeroDivide : public Exception
{
public:
  ZeroDivide() : Exception("Division by near-zero value") {}
};

} // mt

#endif // MT_EXCEPTION_H
