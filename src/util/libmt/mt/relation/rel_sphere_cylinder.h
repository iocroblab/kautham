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
#ifndef MT_REL_SPHERE_CYLINDER_H
#define MT_REL_SPHERE_CYLINDER_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/cylinder3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/sphere3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_plane_sphere.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a sphere and a cylinder.
///
/// The computed distance between elements is \e signed, with
/// negative values indicating that the sphere is \e contained in the
/// cylinder.

class RelSphereCylinder :  public RelBase,
                           public RelBaseDist
{
public:

// LIFECYCLE

  RelSphereCylinder(const Sphere3& s,
                    const Cylinder3& c);

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
  /// sphere, otherwise it belongs to the cylinder.
  Point3 getClosestPoint(size_t n = 1) const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Circle3 getIntersectionCircle(size_t n = 1) const;

  /// Gets tangent point.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;

  /// Gets tangent point.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Circle3 getTangentCircle() const;


private:

// MEMBERS

  Point3  m_p1;
  Point3  m_p2;
  Circle3 m_c1;
  Circle3 m_c2;
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&            os,
                         const RelSphereCylinder& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelSphereCylinder::RelSphereCylinder(const Sphere3&   s,
                                            const Cylinder3& c)
{
  // Sphere parameters
  const Point3 cs(s.getCenter());
  const Scalar rs(s.getRadius());

  // Cylinder parameters
  const Line3  ac(c.getAxis());
  const Scalar rc(c.getRadius());

  // Distance between sphere center and cylinder axis
  const Scalar d = distance(cs, ac);

  const Scalar d1 = rs + rc;
  const Scalar d2 = rc - rs;

  // Distance between elements and distance type
  if (d == Scalar(0.0) && d2 == Scalar(0.0))
  {
    // Tangent circle
    m_c1.setValue(cs, rs, ac.getDirection());
    m_dist = Scalar(0.0);
    m_type = RelType(TANGENT, CONCENTRIC);
  }
  else if (d == Scalar(0.0) && d2 > Scalar(0.0))
  {
    m_dist = -d2;
    m_type = CONCENTRIC;
  }
  else if (d == Scalar(0.0) && d2 < Scalar(0.0))
  {
    // Two intersection circles
    const Unit3 dir = ac.getDirection();
    const Scalar offset = sqrt(sq(rs) - sq(rc));
    const Point3 center1 = cs + dir * offset;
    const Point3 center2 = cs - dir * offset;
    m_c1.setValue(center1, rc, dir);
    m_c2.setValue(center2, rc, dir);
    m_dist = Scalar(0.0);
    m_type = RelType(CONCENTRIC, INTERSECTING);
  }
  else if (d > d1 || d < d2)
  {
    // Closest points computation
    m_p2 = project(cs,   c); // Closest point on cylinder
    m_p1 = project(m_p2, s); // Closest point on sphere

    m_dist = sgn(d - d2) * distance(m_p1, m_p2);
    m_type = DISTANCE;
  }
  else if (d == d1 || d == d2)
  {
    m_p1   = project(cs, c);
    m_dist = Scalar(0.0);
    m_type = TANGENT;
  }
  else
  {
    m_dist = Scalar(0.0);
    m_type = INTERSECTING;
  }
}


// ACCESS

inline Point3 RelSphereCylinder::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(DISTANCE),
               Exception("Elements are in contact or concentric"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Circle3 RelSphereCylinder::getIntersectionCircle(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING) && m_type.test(CONCENTRIC),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_c1 : m_c2;
}


inline Point3 RelSphereCylinder::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT) && !m_type.test(CONCENTRIC),
               Exception("Elements are not tangent"));

  return m_p1;
}


inline Circle3 RelSphereCylinder::getTangentCircle() const
{
  util::Assert(m_type.test(TANGENT) && m_type.test(CONCENTRIC),
               Exception("Elements are not tangent"));

  return m_c1;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&          os,
                                const RelSphereCylinder& rel)
{

  os << "distance: " << rel.getDistance();

  const RelType type(rel.getType());

  if (type.test(DISTANCE))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(INTERSECTING) &&
           type.test(CONCENTRIC))
  {
    os << ", intersections: " << rel.getIntersectionCircle(1) << " "
                              << rel.getIntersectionCircle(2);
  }
  else if (type.test(TANGENT) &&
           type.test(CONCENTRIC))
  {
    os << ", tangency: " << rel.getTangentCircle();
  }
  else if (type.test(TANGENT) &&
           !type.test(CONCENTRIC))
  {
    os << ", tangency: " << rel.getTangentPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_SPHERE_CYLINDER_H
