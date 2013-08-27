/***************************************************************************
 *   Copyright (C) 2006 by Adolfo Rodriguez                                *
 *   adolfo.rodriguez@upc.edu                                              *
 *                                                                         *
 *   The contents of this file were taken from Bjarne Stroustroup's        *
 *    "The C++ Programming Language", third edition, p. 753.               *
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
#ifndef ASSERT_TEMPLATE_H
#define ASSERT_TEMPLATE_H


// DEBUGGING OPTIONS
#define NDEBUG // Not debugging macro

#ifdef NDEBUG
const bool ARG_CHECK = false; // We are not debugging, disable checks
#else
const bool ARG_CHECK = true;  // We are debugging
#endif


/////////////////////////////// FUNCTION DEFINITION //////////////////////////

template<class A, class E> inline void Assert(A assertion, E excep)
{
  if (!assertion) throw excep;
}

#endif // ASSERT_TEMPLATE_H
