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
#ifndef MT_REL_PLANE_SPHERE_H
#define MT_REL_PLANE_SPHERE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/plane3.h>
#include <mt/sphere3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_dist.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a plane and a sphere.
///
/// The computed distance between elements is \e unsigned.

class RelPlaneSphere :  public RelBase,
                        public RelBaseDist
{
public:

// LIFECYCLE

  RelPlaneSphere(const Plane3&  P,
                 const Sphere3& s);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets closest points between non-touching elements.
  ///
  /// If the above conditions are not met, then a mt::Exception exception is
  /// thrown.
  ///
  /// If the input parameter \a n = 1, the returned point belongs to the
  /// plane, otherwise it belongs to the sphere.
  Point3 getClosestPoint(size_t n = 1) const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Circle3 getIntersectionCircle() const;

  /// Gets tangent point.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;


private:

// MEMBERS

  Point3  m_p1;
  Point3  m_p2;
  Circle3 m_c;


// OPERATIONS

  /// \param dist Signed distance between the sphere and  the projection of
  /// its center on the line.
  RelType distanceType(const Scalar& dist);
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&         os,
                         const RelPlaneSphere& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPlaneSphere::RelPlaneSphere(const Plane3&  P,
                                      const Sphere3& s)
{
  // Sphere parameters
  const Point3 center(s.getCenter());
  const Scalar radius(s.getRadius());

  // Projection of sphere center on plane
  const Point3 proj(P.project(center));

  // Distance between sphere center and its projection on the plane
  const Scalar dist_proj(length(proj - center));

  // Signed distance between dist_proj and sphere
  const Scalar dist_sphere = dist_proj - radius;

  // Relation type
  m_type = distanceType(dist_sphere);

  // Distance between elements and tangencies/intersections
  if (m_type.test(DISTANCE))
  {
    m_p1 = project(center, P); // Closest point on plane
    m_p2 = project(m_p1, s);   // Closest point on sphere
    m_dist = dist_sphere;
  }
  else if (m_type.test(TANGENT))
  {
    m_dist = Scalar(0.0);
    m_p1 = proj;
  }
  else if (m_type.test(INTERSECTING))
  {
    m_dist = Scalar(0.0);

    // Intersection circle
    const Scalar c_radius(sqrt(sq(radius) - sq(dist_proj)));
    const Unit3  c_normal = P.getNormal();
    m_c.setValue(proj, c_radius, c_normal);
  }

}


// ACCESS

inline Point3 RelPlaneSphere::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(DISTANCE), Exception("Elements are in contact"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Circle3 RelPlaneSphere::getIntersectionCircle() const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return m_c;
}


inline Point3 RelPlaneSphere::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


// OPERATIONS

inline RelType RelPlaneSphere::distanceType(const Scalar& dist)
{
  if (dist > Scalar(0.0))
  {
    return DISTANCE;
  }
  else if (dist < Scalar(0.0))
  {
    return INTERSECTING;
  }
  else
  {
    return TANGENT;
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&        os,
                                const RelPlaneSphere& rel)
{

  os << "distance: " << rel.getDistance();

  const RelType type(rel.getType());

  if (type.test(DISTANCE))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(INTERSECTING))
  {
    // Line intersects sphere
    os << ", intersection: " << rel.getIntersectionCircle();
  }
  else if (type.test(TANGENT))
  {
    // Line is tangent to sphere
    os << ", tangency: " << rel.getTangentPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_PLANE_SPHERE_H
