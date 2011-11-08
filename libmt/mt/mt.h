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
#ifndef MT_MT_H
#define MT_MT_H

// MT LIBRARY HEADERS
#include <mt/scalar.h>
#include <mt/interval.h>
#include <mt/vector3.h>
#include <mt/unit3.h>
#include <mt/point3.h>
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/sphere3.h>
#include <mt/cylinder3.h>
#include <mt/circle3.h>
#include <mt/ellipse3.h>
#include <mt/matrix3x3.h>
#include <mt/object.h>
#include <mt/rotation.h>
#include <mt/transform.h>
#include <mt/transform_utility.h>

#include <mt/relation/relation.h>

// BOOST LIBRARY DEPENDENT HEADERS
#ifdef MT_USE_BOOST

  #include <mt/mt_variant.h>
  #include <mt/element.h>

  #ifdef MT_USE_SERIALIZE
    #include <mt/serialize.h>
  #endif

#endif // MT_USE_BOOST

/// \defgroup basic Basic math classes
/// \brief This module contains classes that model the basic mathematical
/// concepts used in the library.

/// \defgroup elements Geometric elements
/// \brief This module contains classes that model spatial geometric elements.

/// \file mt.h
/// This file includes all the necessary headers to use \e all of the \e mt
/// library's functionalities.

/// \mainpage mt library documentation
///
/// The \e mt library is a set of classes and functions that implement
/// concepts from the domains of geometry and rigid body mechanics.
/// Here are some examples of the functionalities provided by the library:
///
/// - Parameterized floating-point type specifically designed for handling
///   tolerance-based (absolute and relative) equality and inequality tests,
///   as well as robust implementations of common math functions such as
///   inverse trigonometric functions, sqrt, log, etc. (e.g., sqrt of a
///   negative value that is smaller than the specified tolerance will yield
///   zero, not an error).
///   Refer to the documentation of the scalar.h and basic_scalar.h for
///   further details.
///
/// - Classes that model closed intervals, 3D vectors, 3x3 matrices,
///   quaternions, and rigid transformations, along with a comprehensive set
///   of meaningful operations that can be applied
///   (e.g., usual operator overloads, matrix inverse computation,
///   transformation propagation and interpolation).
///   The special cases of 3D unit vectors and unit quaternions have also
///   been implemented.
///
/// - Classes that model 3D geometric elements such as points, lines, planes,
///   spheres, cylinders, circles, and ellipses.
///   There also exist relation classes for querying the geometric
///   relation that exists between two elements
///   (distance, angle, intersections, etc.)
///
/// If the Boost library is installed (http://www.boost.org)
/// a variant datatype is avaliable that can contain any of the geometric
/// elements (uses boost::variant).
/// Also, all the basic math classes as well as the geometric element classes
/// (the only exceptions are the relation classes) have serialization
/// functions implemented (uses boost::serialization), which can be useful for
/// marshalling and persistence purposes.
/// Define the project-wide macros MT_USE_BOOST and MT_USE_SERIALIZATION to
/// make these headers available when including this file.
/// If MT_USE_BOOST is defined, but MT_USE_SERIALIZATION is not, then the
/// variant datatype functionality is included, but the serialization header
/// is not.

#endif // MT_MT_H
