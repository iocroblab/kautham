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
#ifndef MT_REL_CYLINDER_CIRCLE_H
#define MT_REL_CYLINDER_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <kautham/mt/circle3.h>
#include <kautham/mt/cylinder3.h>
#include <kautham/mt/plane3.h>
#include <kautham/mt/point3.h>
#include <kautham/mt/relation/rel_base.h>
#include <kautham/mt/relation/rel_base_dist.h>
#include <kautham/mt/relation/rel_base_ang.h>
#include <kautham/mt/relation/rel_cylinder_cylinder.h>
#include <kautham/mt/relation/rel_line_plane.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a cylinder and a circle.
///
/// The computed distance between elements is \e signed, with
/// negative values indicating that one element is \e contained in the
/// other (circle inside cylinder, or cylinder surrounded by circle).
///
/// <em> Very important note: </em> For simplicity reasons, this clas only
/// handles cases in which the cylinder and circle are (anti)parallel.
/// If the angle between the cylinder axis and the circle normal differs from
/// \f$ n \pi \f$ for an integer \f$ n \f$, an mt::Exception is thrown.

class RelCylinderCircle :  public RelBase,
                           public RelBaseAng,
                           public RelBaseDist
{
public:

// LIFECYCLE

  RelCylinderCircle(const Cylinder3&  cyl,
                    const Circle3&    cir);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets closest points between parallel and non-touching elements.
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
std::ostream& operator<<(std::ostream&            os,
                         const RelCylinderCircle& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelCylinderCircle::RelCylinderCircle(const Cylinder3& cyl,
                                            const Circle3&   cir) :

                           RelBaseAng(angle(cyl.getAxis().getDirection(),
                                            cir.getNormal()))
{
  // Angle type
  m_type = angleType(m_angle);

  util::Assert(m_type.test(SAME_DIRECTION),
               Exception("Cylinder and circle are not (anti)parallel"));

  // Circle support plane
  const Plane3 sup = cir.getSupportPlane();

  // Cylinder that passes through circle and is parallel to it
  const Line3 axis2(cir.getNormal(), cir.getCenter());
  const Cylinder3 cyl2(axis2, cir.getRadius());

  // Relation between cylinders
  const RelCylinderCylinder rel(cyl, cyl2);
  const RelType rel_type = rel.getType();

  // Distance between elements and distance type
  m_dist = rel.getDistance();

  if (rel_type.test(COINCIDENT))
  {
    m_type |= RelType(CONTAINED, CONCENTRIC);
  }
  else if (rel_type.test(CONCENTRIC))
  {
    m_type |= CONCENTRIC;
  }
  else if (rel_type.test(DISTANCE))
  {
    const Line3 L1 = rel.getClosestLine(1);
    const Line3 L2 = rel.getClosestLine(2);

    const RelLinePlane rel_lp1(L1, sup);
    const RelLinePlane rel_lp2(L2, sup);

    m_p1 = rel_lp1.getIntersectionPoint();
    m_p2 = rel_lp2.getIntersectionPoint();

    m_type |= DISTANCE;
  }
  else if (rel_type.test(TANGENT))
  {
    const Line3 L = rel.getTangentLine();
    const RelLinePlane rel_lp(L, sup);

    m_p1 = rel_lp.getIntersectionPoint();

    m_type |= TANGENT;
  }
  else if (rel_type.test(INTERSECTING))
  {
    const Line3 L1 = rel.getIntersectionLine(1);
    const Line3 L2 = rel.getIntersectionLine(2);

    const RelLinePlane rel_lp1(L1, sup);
    const RelLinePlane rel_lp2(L2, sup);

    m_p1 = rel_lp1.getIntersectionPoint();
    m_p2 = rel_lp2.getIntersectionPoint();

    m_type |= INTERSECTING;
  }
}


// ACCESS

inline Point3 RelCylinderCircle::getClosestPoint(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(DISTANCE),
               Exception("Elements are in contact or non-(anti)parallel"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelCylinderCircle::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelCylinderCircle::getTangentPoint() const
{
  util::Assert(m_type.test(SAME_DIRECTION) &&  m_type.test(TANGENT),
               Exception("Elements are not tangent"));

  return m_p1;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&            os,
                                const RelCylinderCircle& rel)
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

#endif // MT_REL_CYLINDER_CIRCLE_H
