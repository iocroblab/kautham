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
#ifndef MT_REL_TYPE_H
#define MT_REL_TYPE_H

// C++ STANDARD HEADERS
#include <iostream>
#include <string>

// TTL library headers
#include <mt/ttl/flags.hpp>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

// TYPES

/// \file rel_type.h Qualitative representation of the geometric relation
/// between two elements.
/// \ingroup rel
///
/// This qualitative representation can be very useful for performing boolean
/// tests such as the following:
///
/// \code
/// using namespace mt;
///
/// // RelType instances
/// RelType type1(PARALLEL);
/// RelType type2(DISTANCE);
///
/// type2 = type1 | type2;
/// type2 |= ANTIPARALLEL;
///
/// // Boolean tests
/// bool test;
///
/// test = type1.test(PARALLEL);        // test = true
/// test = type1.test(SAME_DIRECTION);  // test = true
/// test = (type1 == PERPENDICULAR);    // test = false
///
/// \endcode
///
/// This datatype is based on the typesafe and generic bit flags class
/// implemented as part of the <em> Tiny Template Library </em>.
/// For more information on the subject, please refer to the following link:
/// http://www.codeproject.com/cpp/TTLFlags.asp


enum RelTypeBase
{
  DISTANCE       = 1,
  COINCIDENT     = 2,
  CONTAINED      = 4,
  INTERSECTING   = 8,
  TANGENT        = 16,
  CONCENTRIC     = 32,

  ANGLE          = 64,
  PARALLEL       = 128,
  ANTIPARALLEL   = 256,
  SAME_DIRECTION = PARALLEL + ANTIPARALLEL,
  PERPENDICULAR  = 512,

  FIXED_R        = 1024,
  FIXED_T        = 2048,
  FIXED_RT       = 4096
};

/// Relation type flag.
typedef ttl::flg::flags<RelTypeBase> RelType;


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&  os,
                         const RelType& type);


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

inline std::ostream& operator<<(std::ostream&  os,
                                const RelType& type)
{
  // Relation type string representation
  std::string name;

  // Distance type output
  const bool is_distance      = type.test(DISTANCE);
  const bool is_coincident    = type.test(COINCIDENT);
  const bool is_contained     = type.test(CONTAINED);
  const bool is_intersecting  = type.test(INTERSECTING);
  const bool is_tangent       = type.test(TANGENT);
  const bool is_concentric    = type.test(CONCENTRIC);

  if (is_distance)
  {
    name +="distance ";
  }
  if (is_coincident)
  {
    name +="coincident ";
  }
  if (is_contained)
  {
    name +="contained ";
  }
  if (is_intersecting)
  {
    name +="intersecting ";
  }
  if (is_tangent)
  {
    name +="tangent ";
  }
  if (is_concentric)
  {
    name +="concentric ";
  }

  // Angle type output
  const bool is_parallel      = type.test(PARALLEL);
  const bool is_antiparallel  = type.test(ANTIPARALLEL);
  const bool is_perpendicular = type.test(PERPENDICULAR);
  const bool is_angle         = type.test(ANGLE);

  if (is_parallel && is_antiparallel)
  {
    name +="same direction ";
  }
  if (is_parallel)
  {
    name +="parallel ";
  }
  if (is_antiparallel)
  {
    name +="antiparallel ";
  }
  if (is_perpendicular)
  {
    name +="perpendicular ";
  }
  else if (is_angle)
  {
    name +="angle ";
  }

  // Fixed types output
  const bool is_fixed_r = type.test(FIXED_R);
  const bool is_fixed_t = type.test(FIXED_T);
  const bool is_fixed   = type.test(FIXED_RT);

  if (is_fixed_r)
  {
    name +="fixed R ";
  }
  else if (is_fixed_t)
  {
    name +="fixed T ";
  }
  else if (is_fixed)
  {
    name +="fixed ";
  }

  // Removes trailing space
  name.erase(name.end() - 1);

  return os << name;
}

} // mt

#endif // MT_RELATION_TYPE_H
