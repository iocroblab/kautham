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
#ifndef MT_REL_PLANE_CIRCLE_H
#define MT_REL_PLANE_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_line_circle.h>
#include <mt/relation/rel_plane_plane.h>

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
/// The angle between elements is measured between the plane and the cylinder
/// support plane; and the computed distance is \e unsigned.

class RelPlaneCircle : public RelBase,
                       public RelBaseAng,
                       public RelBaseDist
{
public:

// LIFECYCLE

  RelPlaneCircle(const Plane3&  P,
                 const Circle3& c);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets tangency point between elements.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;

  /// Gets the closest points between elements.
  /// If the input parameter \a n = 1, the return value corresponds to the
  /// point on the plane, otherwise it belongs to the circle.
  ///
  /// If closest points do not apply for the current element configuration, a
  /// mt::Exception exception is thrown.
  Point3 getClosestPoint(size_t n = 1) const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  /// If the elements intersect in two points, each one can be accessed using
  /// the input parameter (1 = one point, 2 = the other point).
  Point3 getIntersectionPoint(size_t n = 1) const;


private:

  // MEMBERS
  Point3 m_p1;
  Point3 m_p2;
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&         os,
                         const RelPlaneCircle& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPlaneCircle::RelPlaneCircle(const Plane3&  P,
                                      const Circle3& c)
{
  // Circle parameters
  const Point3 center = c.getCenter();
  const Scalar radius = c.getRadius();
  const Plane3 sup    = c.getSupportPlane();

  // Relation between plane and circle support plane
  const RelPlanePlane rel(P, sup);
  const RelType rel_type = rel.getType();

  // Angle between elements and angle type
  m_angle = rel.getAngle();
  m_type = angleType(m_angle);

  // Distance between elements and distance type
  if (rel_type.test(COINCIDENT))
  {
    m_dist  = Scalar(0.0);
    m_type |= CONTAINED;
  }
  else if (rel_type.test(DISTANCE))
  {
    m_dist  = rel.getDistance();
    m_type |= DISTANCE;
  }
  else if (rel_type.test(INTERSECTING))
  {
    const Line3 L = rel.getIntersectionLine();
    const RelLineCircle rel_lc(L, c);
    const RelType rel_lc_type = rel_lc.getType();

    if (rel_lc_type.test(DISTANCE))
    {
      const Point3 proj = project(center, L);
      const Unit3  dir  = proj - center;

      m_p2    = center + dir * radius; // Closest point on circle
      m_p1    = project(m_p2, P);      // Closest point on plane
      m_type |= DISTANCE;
    }
    else if (rel_lc_type.test(TANGENT))
    {
      m_p1    = rel_lc.getTangentPoint();
      m_type |= TANGENT;
    }
    else if (rel_lc_type.test(INTERSECTING))
    {
      m_p1    = rel_lc.getIntersectionPoint(1);
      m_p2    = rel_lc.getIntersectionPoint(2);
      m_type |= INTERSECTING;
    }
  }
}


// ACCESS

inline Point3 RelPlaneCircle::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


inline Point3 RelPlaneCircle::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(DISTANCE) && !m_type.test(SAME_DIRECTION),
               Exception("Closest point computation does not apply"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelPlaneCircle::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&         os,
                                const RelPlaneCircle& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(DISTANCE) &&
      !type.test(SAME_DIRECTION))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(TANGENT))
  {
    os << "tangency: " << rel.getTangentPoint();
  }
  else if (type.test(INTERSECTING))
  {
    os << ", intersections: " << rel.getIntersectionPoint(1) << " "
                              << rel.getIntersectionPoint(2);
  }

  return os;
}


} // mt

#endif // MT_REL_PLANE_CIRCLE_H
