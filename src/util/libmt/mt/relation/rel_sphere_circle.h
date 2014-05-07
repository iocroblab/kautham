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
#ifndef MT_REL_SPHERE_CIRCLE_H
#define MT_REL_SPHERE_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/sphere3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_plane_circle.h>
#include <mt/relation/rel_plane_sphere.h>
#include <mt/relation/rel_sphere_sphere.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a sphere and a circle.
///
/// The computed distance between elements is \e signed, with
/// negative values indicating that one element is \e contained in the
/// other (circle inside sphere, or sphere surrounded by circle, like Saturn).

class RelSphereCircle :  public RelBase,
                         public RelBaseDist
{
public:

// LIFECYCLE

  RelSphereCircle(const Sphere3&  s,
                  const Circle3& c);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Point3 getIntersectionPoint(size_t n = 1) const;

  /// Gets tangent point.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;


private:

// MEMBERS

  Point3  m_p1;
  Point3  m_p2;
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&          os,
                         const RelSphereCircle& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelSphereCircle::RelSphereCircle(const Sphere3&   s,
                                        const Circle3&   c)
{
  // Sphere parameters
  const Point3 center_s(s.getCenter());
  const Scalar radius_s(s.getRadius());

  // Circle parameters
  const Point3 center_c(c.getCenter());
  const Scalar radius_c(c.getRadius());
  const Plane3 plane_c (c.getSupportPlane());

  // Relation between sphere and circle support plane
  const RelPlaneSphere rel(plane_c, s);
  const RelType rel_type = rel.getType();

  if (rel_type.test(DISTANCE))
  {
    // Distance elements
    m_dist = distance(center_s, c) - radius_s;
    m_type = DISTANCE;
  }
  else if (rel_type.test(TANGENT))
  {
    const Point3 p = rel.getTangentPoint();
    const Scalar dist = distance(p, c);

    if (dist == Scalar(0.0))
    {
      // Tangent elements
      m_p1  = p;
      m_dist = Scalar(0.0);
      m_type = TANGENT;
    }
    else
    {
      // Distance elements
      m_dist = dist;
      m_type = DISTANCE;
    }
  }
  else
  {
    // Intersection of circle support plane and sphere
    const Circle3 c2 = rel.getIntersectionCircle();

    const Sphere3 s1(center_c, radius_c);
    const Sphere3 s2(c2.getCenter(), c2.getRadius());

    const RelSphereSphere rel_s(s1, s2);
    const RelType rel_s_type = rel_s.getType();

    m_dist = rel_s.getDistance();

    if (rel_s_type.test(COINCIDENT))
    {
      m_type = RelType(CONTAINED, CONCENTRIC);
    }
    else
    {
      m_type = rel_s_type;
    }

    if (rel_s_type.test(TANGENT))
    {
      m_p1 = rel_s.getTangentPoint();
    }
    else if (rel_s_type.test(INTERSECTING))
    {
      const Circle3 c3 = rel_s.getIntersectionCircle();
      const RelPlaneCircle rel_pc(plane_c, c3);
      m_p1   = rel_pc.getIntersectionPoint(1);
      m_p2   = rel_pc.getIntersectionPoint(2);
    }
  }
}


// ACCESS

inline Point3 RelSphereCircle::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelSphereCircle::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&            os,
                                const RelSphereCircle&   rel)
{

  os << "distance: " << rel.getDistance();

  const RelType type(rel.getType());

  if (type.test(INTERSECTING))
  {
    os << ", intersections: " << rel.getIntersectionPoint(1) << " "
                              << rel.getIntersectionPoint(2);
  }
  else if (type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_SPHERE_CIRCLE_H
