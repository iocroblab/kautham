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
#ifndef MT_REL_CYLINDER_CYLINDER_H
#define MT_REL_CYLINDER_CYLINDER_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/cylinder3.h>
#include <mt/line3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_line_line.h>
#include <mt/relation/rel_plane_cylinder.h>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between two cylinders.
///
/// The computed distance between elements is a \e signed distance, with
/// negative values indicating that one cylinder is \e contained inside the
/// other.

class RelCylinderCylinder : public RelBase,
                            public RelBaseAng,
                            public RelBaseDist
{
public:

// LIFECYCLE

  RelCylinderCylinder(const Cylinder3& c1,
                      const Cylinder3& c2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets closest points between non-parallel and non-touching elements.
  ///
  /// If the above conditions are not met, then a mt::Exception exception is
  /// thrown.
  ///
  /// If the input parameter \a n = 1, the returned point belongs to the first
  /// cylinder, otherwise the returned point belongs to the second one.
  Point3 getClosestPoint(size_t n = 1) const;

  /// Gets closest lines between parallel and non-touching elements.
  ///
  /// If the above conditions are not met, then a mt::Exception exception is
  /// thrown.
  ///
  /// If the input parameter \a n = 1, the returned point belongs to the first
  /// cylinder, otherwise the returned line belongs to the second one.
  Line3 getClosestLine(size_t n = 1) const;

  /// Gets tangency point between elements.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Point3 getTangentPoint() const;

  /// Gets tangency line between elements.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Line3 getTangentLine() const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  /// If the elements intersect in two lines, each one can be accessed using
  /// the input parameter (1 = one line, 2 = the other line).
  Line3 getIntersectionLine(size_t n = 1) const;


private:

  // MEMBERS
  Point3 m_p1;
  Point3 m_p2;
  Line3  m_L1;
  Line3  m_L2;

  // OPERATIONS

  /// Performs instance initialization for (anti)parallel elements
  void initSameDirection(const Cylinder3& c1, const Cylinder3& c2);

  /// Performs instance initialization for non-(anti)parallel elements
  void initDiffDirection(const Cylinder3& c1, const Cylinder3& c2);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&              os,
                         const RelCylinderCylinder& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelCylinderCylinder::RelCylinderCylinder(const Cylinder3& c1,
                                                const Cylinder3& c2) :

                            RelBaseAng(angle(c1.getAxis().getDirection(),
                                             c2.getAxis().getDirection()))
{
  // Angle type
  const DirectionType type =
  (c1.getAxis().getDirectionType() == DIRECTED &&
   c2.getAxis().getDirectionType() == DIRECTED) ? DIRECTED : UNDIRECTED;

  m_type = angleType(m_angle, type);

  // Distance between elements and distance type
  if (m_type.test(SAME_DIRECTION))
  {
    initSameDirection(c1, c2);
  }
  else
  {
    initDiffDirection(c1, c2);
  }
}


// ACCESS

inline Point3 RelCylinderCylinder::getClosestPoint(size_t n) const
{
  util::Assert(!m_type.test(SAME_DIRECTION) && m_type.test(DISTANCE),
               Exception("Elements are not skew"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Line3 RelCylinderCylinder::getClosestLine(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(DISTANCE),
               Exception("Elements are not (anti)parallel or are in contact"));

  return (n == 1) ? m_L1 : m_L2;
}


inline Point3 RelCylinderCylinder::getTangentPoint() const
{
  util::Assert(!m_type.test(SAME_DIRECTION) && m_type.test(TANGENT),
               Exception("Elements are not tangent."));

  return m_p1;
}


inline Line3 RelCylinderCylinder::getTangentLine() const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(TANGENT),
               Exception("Elements are not tangent."));

  return m_L1;
}


inline Line3 RelCylinderCylinder::getIntersectionLine(size_t n) const
{
  util::Assert(m_type.test(SAME_DIRECTION) && m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_L1 : m_L2;
}


// PRIVATE OPERATIONS

inline void RelCylinderCylinder::initSameDirection(const Cylinder3& c1,
                                                   const Cylinder3& c2)
{
  // Cylinder parameters
  const Line3  a1(c1.getAxis());
  const Scalar r1(c1.getRadius());

  const Line3  a2(c2.getAxis());
  const Scalar r2(c2.getRadius());

  // Relation between line axis
  const RelLineLine rel(a1, a2);

  // Distance between axis
  const Scalar d = rel.getDistance();

  const Scalar d1 = abs(r2 - r1);
  const Scalar d2 = r1 + r2;

  // Distance type
  if (d == Scalar(0.0) && d1 == Scalar(0.0))
  {
    m_dist  = Scalar(0.0);
    m_type |= RelType(COINCIDENT, CONCENTRIC);
  }
  else if (d == Scalar(0.0) && d1 != Scalar(0.0))
  {
    m_dist  = -d1;
    m_type |= CONCENTRIC;
  }
  else if (d < d1 || d > d2)
  {
    // Closest lines computation
    Point3 sup;
    Point3 sup1;
    Point3 sup2;

    if (r1 < r2)
    {
      sup  = a1.getSupport();
      sup2 = project(sup,  c2);
      sup1 = project(sup2, c1);
    }
    else
    {
      sup  = a2.getSupport();
      sup1 = project(sup,  c1);
      sup2 = project(sup1, c2);
    }

    m_L1.setValue(a1.getDirection(), sup1, UNDIRECTED);
    m_L2.setValue(a1.getDirection(), sup2, UNDIRECTED);
    m_dist  = sgn(d - d2) * distance(sup1, sup2);
    m_type |= DISTANCE;
  }
  else if (d == d1 || d == d2)
  {
    Point3 sup;
    if (r1 < r2)
    {
      sup = project(a1.getSupport(), c2);
    }
    else
    {
      sup = project(a2.getSupport(), c1);
    }
    m_L1.setValue(a1.getDirection(), sup, UNDIRECTED);
    m_dist  = Scalar(0.0);
    m_type |= TANGENT;
  }
  else
  {
    // Plane passing through the intersection of the two cylinders
    const Point3 sup1 = a1.getSupport();
    const Point3 sup2 = project(sup1, a2);
    Unit3  dir        = sup2 - sup1;
    Scalar offset     = (sq(d) + sq(r1) - sq(r2)) / (Scalar(2.0) * d);
    const Point3 sup  = sup1 + offset * dir;
    const Plane3 P(dir, sup);

    // Intersection lines
    RelPlaneCylinder rel_pc(P, c1);
    m_L1    = rel_pc.getIntersectionLine(1);
    m_L2    = rel_pc.getIntersectionLine(2);
    m_dist  = Scalar(0.0);
    m_type |= INTERSECTING;
  }
}


inline void RelCylinderCylinder::initDiffDirection(const Cylinder3& c1,
                                                   const Cylinder3& c2)
{
  // Cylinder parameters
  const Line3  a1(c1.getAxis());
  const Scalar r1(c1.getRadius());

  const Line3  a2(c2.getAxis());
  const Scalar r2(c2.getRadius());

  // Relation between line axis
  const RelLineLine rel(a1, a2);

  // Distance between axis
  const Scalar d = rel.getDistance();

  const Scalar d2 = r1 + r2;

  // Distance type
  if (d < d2)
  {
    // Intersecting elements (intersection curve not calculated)
    m_dist  = Scalar(0.0);
    m_type |= INTERSECTING;
  }
  else
  {
    // Closest points in cylinder axis
    const Point3 p1 = rel.getClosestPoint(1);
    const Point3 p2 = rel.getClosestPoint(2);

    // Unit vector in the direction p1-p2
    Unit3 dir;
    if (d == Scalar(0.0))
    {
      dir = cross(a1.getDirection(),
                  a2.getDirection()); // Cylinders have both zero radius
    }
    else
    {
      dir = p2 - p1;
    }

    if (d == d2)
    {
      // Tangent point
      m_p1   = p1 + dir * r1;
      m_dist = Scalar(0.0);
      m_type |= TANGENT;
    }
    else
    {
      // Closest points
      m_p1 = p1 + dir * r1;
      m_p2 = p2 - dir * r2;
      m_dist  = distance(m_p1, m_p2);
      m_type |= DISTANCE;
    }
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&              os,
                                const RelCylinderCylinder& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(SAME_DIRECTION) && type.test(DISTANCE))
  {
    os << ", closest lines: "  << rel.getClosestLine(1) << " "
                               << rel.getClosestLine(2);
  }
  else if (!type.test(SAME_DIRECTION) && type.test(DISTANCE))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(SAME_DIRECTION) && type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentLine();
  }
  else if (!type.test(SAME_DIRECTION) && type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentPoint();
  }
  else if (type.test(SAME_DIRECTION) && type.test(INTERSECTING))
  {
    os << ", intersections: " << rel.getIntersectionLine(1) << " "
                              << rel.getIntersectionLine(2);
  }

  return os;
}


} // mt

#endif // MT_REL_CYLINDER_CYLINDER_H
