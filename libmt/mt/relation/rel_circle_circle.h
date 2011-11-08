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
#ifndef MT_REL_CIRCLE_CIRCLE_H
#define MT_REL_CIRCLE_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/cylinder3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_cylinder_circle.h>
#include <mt/relation/rel_plane_plane.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between two circles.
///
/// The computed distance between elements is \e signed, with
/// negative values indicating that one circle is \e contained in the
/// other.
///
/// <em> Very important note: </em> For simplicity reasons, this clas only
/// handles cases in which the circles are coplanar.
/// If this condition is not met, an mt::Exception is thrown.

class RelCircleCircle :  public RelBase,
                         public RelBaseAng,
                         public RelBaseDist
{
public:

// LIFECYCLE

  RelCircleCircle(const Circle3& c1,
                  const Circle3& c2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets closest points between coplanar and non-touching elements.
  ///
  /// If the above conditions are not met, then a mt::Exception exception is
  /// thrown.
  ///
  /// If the input parameter \a n = 1, the returned point belongs to the line,
  /// otherwise the returned point belongs to the cylinder.
  Point3 getClosestPoint(size_t n = 1) const;

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
                         const RelCircleCircle& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelCircleCircle::RelCircleCircle(const Circle3& c1,
                                        const Circle3& c2) :

                             RelBaseAng(angle(c1.getNormal(),
                                              c2.getNormal()))
{
  // Angle type
  m_type = angleType(m_angle);

  // Circle support planes
  const Plane3 sup1 = c1.getSupportPlane();
  const Plane3 sup2 = c2.getSupportPlane();

  const RelPlanePlane rel_p(sup1, sup2);
  util::Assert(rel_p.getType().test(COINCIDENT),
               Exception("Circles are not coplanar"));

  // Cylinder that passes through first circle and is parallel to it
  const Line3 axis(c1.getNormal(), c1.getCenter());
  const Cylinder3 cyl(axis, c1.getRadius());

  // Relation between cylinder and second circle
  const RelCylinderCircle rel(cyl, c2);
  const RelType rel_type = rel.getType();

  // Distance between elements and distance type
  m_dist = rel.getDistance();

  if (rel_type.test(CONTAINED))
  {
    m_type |= RelType(COINCIDENT, CONCENTRIC);
  }
  else if (rel_type.test(CONCENTRIC))
  {
    m_type |= CONCENTRIC;
  }
  else if (rel_type.test(DISTANCE))
  {
    m_p1    = rel.getClosestPoint(1);
    m_p2    = rel.getClosestPoint(2);
    m_type |= DISTANCE;
  }
  else if (rel_type.test(TANGENT))
  {
    m_p1    = rel.getTangentPoint();
    m_type |= TANGENT;
  }
  else if (rel_type.test(INTERSECTING))
  {
    m_p1    = rel.getIntersectionPoint(1);
    m_p2    = rel.getIntersectionPoint(2);
    m_type |= INTERSECTING;
  }
}


// ACCESS

inline Point3 RelCircleCircle::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(DISTANCE),
               Exception("Elements are in contact or non-(anti)parallel"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelCircleCircle::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelCircleCircle::getTangentPoint() const
{
  util::Assert(m_type.test(SAME_DIRECTION) &&  m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&            os,
                                const RelCircleCircle& rel)
{

  os << "distance: " << rel.getDistance();

  const RelType type(rel.getType());

  if (type.test(SAME_DIRECTION) && type.test(DISTANCE))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(SAME_DIRECTION) && type.test(INTERSECTING))
  {
    os << ", intersections: " << rel.getIntersectionPoint(1) << " "
                              << rel.getIntersectionPoint(2);
  }
  else if (type.test(SAME_DIRECTION) && type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentPoint();
  }

  return os;
}


} // mt

#endif // MT_REL_CIRCLE_CIRCLE_H
