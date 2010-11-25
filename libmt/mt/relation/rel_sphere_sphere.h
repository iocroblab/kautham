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
#ifndef MT_REL_SPHERE_SPHERE_H
#define MT_REL_SPHERE_SPHERE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
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
/// \brief Computes the geometric relation between two spheres.
///
/// The computed distance between elements is \e signed, with
/// negative values indicating that one sphere is \e contained in the other.

class RelSphereSphere :  public RelBase,
                         public RelBaseDist
{
public:

// LIFECYCLE

  RelSphereSphere(const Sphere3& s1,
                  const Sphere3& s2);

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
  /// If the input parameter \a n = 1, the returned point belongs to the first
  /// sphere, otherwise it belongs to the second one.
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
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&          os,
                         const RelSphereSphere& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelSphereSphere::RelSphereSphere(const Sphere3& s1,
                                        const Sphere3& s2)
{
  // Sphere parameters
  const Point3 c1(s1.getCenter());
  const Scalar r1(s1.getRadius());

  const Point3 c2(s2.getCenter());
  const Scalar r2(s2.getRadius());

  // Distance between sphere centers
  const Scalar d = distance(c1, c2);

  const Scalar d1 = abs(r1 - r2);
  const Scalar d2 = r1 + r2;

  // Distance between elements and distance type
  if (d == Scalar(0.0) && d1 == Scalar(0.0))
  {
    m_dist = Scalar(0.0);
    m_type = RelType(COINCIDENT, CONCENTRIC);
  }
  else if (d == Scalar(0.0) && d1 != Scalar(0.0))
  {
    m_dist = -d1;
    m_type = CONCENTRIC;
  }
  else if (d < d1 || d > d2)
  {
    // Closest points computation
    if (r1 < r2)
    {
      m_p2 = project(c1,   s2); // Closest point on first sphere
      m_p1 = project(m_p2, s1); // Closest point on second sphere
    }
    else
    {
      m_p1 = project(c2,   s1); // Closest point on first sphere
      m_p2 = project(m_p1, s2); // Closest point on second sphere
    }
    m_dist = sgn(d - d2) * distance(m_p1, m_p2);
    m_type = DISTANCE;
  }
  else if (d == d1 || d == d2)
  {
    if (r1 < r2)
    {
      m_p1 = project(c1, s2);
    }
    else
    {
      m_p1 = project(c2, s1);
    }
    m_dist = Scalar(0.0);
    m_type = TANGENT;
  }
  else
  {
    // Plane passing through the intersection of the two spheres
    Unit3  dir       = c2 - c1;
    Scalar offset    = (sq(d) + sq(r1) - sq(r2)) / (Scalar(2.0) * d);
    const Point3 sup = c1 + offset * dir;
    const Plane3 P(dir, sup);

    // Intersection circle
    RelPlaneSphere rel(P, s1);
    m_c    = rel.getIntersectionCircle();
    m_dist = Scalar(0.0);
    m_type = INTERSECTING;
  }
}


// ACCESS

inline Point3 RelSphereSphere::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(DISTANCE),
               Exception("Elements are in contact or concentric"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Circle3 RelSphereSphere::getIntersectionCircle() const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return m_c;
}


inline Point3 RelSphereSphere::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&          os,
                                const RelSphereSphere& rel)
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

#endif // MT_REL_SPHERE_SPHERE_H
