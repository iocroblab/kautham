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
#ifndef MT_REL_LINE_PLANE_H
#define MT_REL_LINE_PLANE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_base_dist.h>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a line and a plane.
///
/// The computed distance between elements is \e unsigned.

class RelLinePlane : public RelBase,
                     public RelBaseAng,
                     public RelBaseDist
{
public:

// LIFECYCLE

  RelLinePlane(const Line3&  L,
               const Plane3& P);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Intersection between elements.
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Point3 getIntersectionPoint() const;


private:

  // MEMBERS
  Point3 m_intersection;

  // METHODS
  void intersection(const Line3&  L,
                    const Plane3& P);

  /// This function requires the angular part of the relation type to be
  /// already set, since it uses it.
  RelType distanceType(const Scalar& dist);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&       os,
                         const RelLinePlane& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelLinePlane::RelLinePlane(const Line3&  L,
                                  const Plane3& P)
{
  // Angle between line and plane
  m_angle = abs(HALF_PI - angle(L.getDirection(), P.getNormal()));

  // Angle type
  m_type = angleType(m_angle);

  // Distance between elements
  if (m_type.test(SAME_DIRECTION))
  {
    // Line is parallel to/contained in plane
    m_dist = distance(L.getSupport(), P);
  }
  else
  {
    // Line intersects plane
    m_dist = 0.0;
    intersection(L, P);
  }

  // Distance type
  m_type |= distanceType(m_dist);
}


// ACCESS

inline Point3 RelLinePlane::getIntersectionPoint() const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Line does not intersect plane."));

  return m_intersection;
}


// PRIVATE METHODS

inline void RelLinePlane::intersection(const Line3&  L,
                                       const Plane3& P)
{
  // Line parameters
  const Unit3  Ldir(L.getDirection());
  const Point3 Lsup(L.getSupport());

  // Plane parameters
  const Unit3  Pnorm(P.getNormal());
  const Scalar Pdist(P.getDistOrig());

  // Independent parameter "t" of line at intersection point
  const Scalar t = -(dot(Pnorm, Lsup) + Pdist) / dot(Pnorm, Ldir);

  // Intersection point
  m_intersection = L.getPoint(t);
}


inline RelType RelLinePlane::distanceType(const Scalar& dist)
{
  if (dist == Scalar(0.0))
  {
    if (m_type.test(SAME_DIRECTION))
    {
      return CONTAINED;
    }
    else
    {
      return INTERSECTING;
    }
  }
  else
  {
    return DISTANCE;
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&       os,
                                const RelLinePlane& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(INTERSECTING))
  {
    // Line intersects plane
    os << ", intersection: " << rel.getIntersectionPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_LINE_PLANE_H
