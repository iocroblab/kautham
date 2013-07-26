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
#ifndef MT_REL_LINE_SPHERE_H
#define MT_REL_LINE_SPHERE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/line3.h>
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
/// \brief Computes the geometric relation between a line and a sphere.
///
/// The computed distance between elements is \e unsigned.

class RelLineSphere :  public RelBase,
                       public RelBaseDist
{
public:

// LIFECYCLE

  RelLineSphere(const Line3&   L,
                const Sphere3& s);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  /// If the elements intersect in two points, each one can be accessed using
  /// the input parameter (1 = one point, 2 = the other point).
  /// If the elements intersect in only one point, it will be returned no
  /// matter what input argument was provided.
  Point3 getIntersectionPoint(size_t n = 1) const;

  /// Gets tangent point.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;


private:

// MEMBERS

  Point3 m_p1;
  Point3 m_p2;


// OPERATIONS

  /// \param dist Signed distance between the sphere and  the projection of
  /// its center on the line.
  RelType distanceType(const Scalar& dist);
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&        os,
                         const RelLineSphere& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelLineSphere::RelLineSphere(const Line3&   L,
                                    const Sphere3& s)
{
  // Sphere parameters
  const Point3 center(s.getCenter());
  const Scalar radius(s.getRadius());

  // Projection of sphere center on line
  const Point3 proj(L.project(center));

  // Distance between sphere center and its projection on the line
  const Scalar dist_proj(length(proj - center));

  // Signed distance between dist_proj and sphere
  const Scalar dist_sphere = dist_proj - radius;

  // Relation type
  m_type = distanceType(dist_sphere);

  // Distance between elements and tangencies/intersections
  if (m_type.test(DISTANCE))
  {
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

    // Distance between projection point and intersections
    const Scalar dist_int(sqrt(sq(radius) - sq(dist_proj)));

    // Intersection points
    const Unit3 dir(L.getDirection());
    m_p1 = proj + dist_int * dir;
    m_p2 = proj - dist_int * dir;
  }

}


// ACCESS

inline Point3 RelLineSphere::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelLineSphere::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


// OPERATIONS

inline RelType RelLineSphere::distanceType(const Scalar& dist)
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
                                const RelLineSphere& rel)
{

  os << "distance: " << rel.getDistance();

  const RelType type(rel.getType());

  if (type.test(INTERSECTING))
  {
    // Line intersects sphere
    os << ", intersections: " << rel.getIntersectionPoint(1) << " "
                               << rel.getIntersectionPoint(2);
  }
  else if (type.test(TANGENT))
  {
    // Line is tangent to sphere
    os << ", tangency: " << rel.getTangentPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_LINE_SPHERE_H
