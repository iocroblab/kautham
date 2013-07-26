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
#ifndef MT_REL_LINE_CIRCLE_H
#define MT_REL_LINE_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/line3.h>
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
/// \brief Computes the geometric relation between a line and a circle.
///
/// Angle and distance measurements have been omitted in this class.
/// Only the relation type is calculated, and when applicable tangency and
/// intersection points.

class RelLineCircle : public RelBase
{
public:

// LIFECYCLE

  RelLineCircle(const Line3&   L,
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
std::ostream& operator<<(std::ostream&          os,
                         const RelLineCircle&   rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelLineCircle::RelLineCircle(const Line3&   L,
                                    const Circle3& c)
{
  // Relation between line and circle support plane
  const Plane3       sup = c.getSupportPlane();
  const RelLinePlane rel(L, sup);
  const RelType      rel_type = rel.getType();

  // Circle parameters
  const Point3 center = c.getCenter();
  const Scalar radius = c.getRadius();
  const Unit3  normal = c.getNormal();

  // Line direction
  const Unit3 direction = L.getDirection();

  // Computes closest line point to circle center and projects it on the
  // circle support plane
  const Point3 proj_l = project(center, L);
  const Point3 proj_p = project(proj_l, sup);

  // Distance between elements
  if (rel_type.test(INTERSECTING) &&
      proj_p == center)
  {
    // Line passes through circle center
    m_type |= (rel_type.test(PERPENDICULAR)) ? CONCENTRIC : DISTANCE;
  }
  else if (rel_type.test(CONTAINED) &&
           proj_p == center)
  {
    // Line is contained in circle plane and passes through circle center
    const Vector3 offset = radius * direction;
    m_p1  = center + offset;
    m_p2  = center - offset;
    m_type |= INTERSECTING;
  }
  else
  {
    // Closest circle and line points.
    const Unit3  dir_c = proj_p - center;
    const Point3 closest_c = center + radius * dir_c;
    const Point3 closest_l = project(closest_c, L);
    const Scalar dist_closest = distance(closest_l, closest_c);
    const Scalar dist_center  = distance(closest_l, center);

    if (dist_closest == Scalar(0.0))
    {
      // Tangent elements
      m_p1  = closest_l;
      m_type |= TANGENT;
    }
    else if(dist_center < radius &&
            rel_type.test(CONTAINED))
    {
      // Intersecting elements
      const Scalar  d      = sqrt(sq(radius) - distance(closest_l, center));
      const Vector3 offset = d * direction;

      m_p1  = closest_l + offset;
      m_p2  = closest_l - offset;
      m_type |= INTERSECTING;
    }
    else if (dist_center < radius &&
             rel_type.test(DISTANCE))
    {
      // Line is over circle (i.e., casts a shadow on it)
      m_type |= DISTANCE;
    }
    else
    {
      m_type |= DISTANCE;
    }
  }
}


// ACCESS

inline Point3 RelLineCircle::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Line and circle are not tangent."));

  return m_p1;
}


inline Point3 RelLineCircle::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Line and cylinder do not intersect."));

  return (n == 1) ? m_p1 : m_p2;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&          os,
                                const RelLineCircle&   rel)
{
  const RelType type(rel.getType());

  if (type.test(TANGENT))
  {
    os << "tangency: " << rel.getTangentPoint();
  }
  else if (type.test(INTERSECTING))
  {
    os << "intersections: " << rel.getIntersectionPoint(1) << " "
                            << rel.getIntersectionPoint(2);
  }
  return os;
}


} // mt

#endif // MT_REL_LINE_CIRCLE_H
