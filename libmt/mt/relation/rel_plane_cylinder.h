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
#ifndef MT_REL_PLANE_CYLINDER_H
#define MT_REL_PLANE_CYLINDER_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/cylinder3.h>
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_base_dist.h>
#include <mt/relation/rel_line_plane.h>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a plane and a cylinder.
///
/// The angle between elements is measured between the plane and the cylinder
/// axis, and the computed distance is \e unsigned.


class RelPlaneCylinder : public RelBase,
                         public RelBaseAng,
                         public RelBaseDist
{
public:

// LIFECYCLE

  RelPlaneCylinder(const Plane3&    P,
                   const Cylinder3& c);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets tangency line between elements.
  ///
  /// If the elements are not tangent, an mt::Exception exception is thrown.
  Line3 getTangentLine() const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Circle3 getIntersectionCircle() const;

  /// Intersection between elements.
  ///
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  /// If the elements intersect in two lines, each one can be accessed using
  /// the input parameter (1 = one line, 2 = the other line).
  Line3 getIntersectionLine(size_t n = 1) const;


private:

  // MEMBERS
  Circle3 m_c;
  Line3   m_L1;
  Line3   m_L2;


  // METHODS

  /// This function requires the angular part of the relation type to be
  /// already set, since it uses it.
  RelType distanceType(const Scalar& dist);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&          os,
                         const RelPlaneCylinder& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPlaneCylinder::RelPlaneCylinder(const Plane3&    P,
                                          const Cylinder3& c)
{
  // Cylinder parameters
  const Line3  axis(c.getAxis());
  const Scalar radius(c.getRadius());

  // Relation between cylinder axis and plane
  const RelLinePlane rel(axis, P);
  const RelType      rel_type = rel.getType();

  // Angle between elements and angle type
  m_angle = rel.getAngle();
  m_type = angleType(m_angle);

  // Signed distance between cylinder and plane
  const Scalar dist = rel.getDistance() - radius;

  // Distance type
  m_type |= distanceType(dist);

  // Distance between elements and tangencies/intersections
  if (m_type.test(DISTANCE))
  {
    m_dist = dist;
  }
  else if (m_type.test(TANGENT))
  {
    m_dist = Scalar(0.0);
    const Point3 proj = project(axis.getSupport(), P);
    m_L1.setValue(axis.getDirection(), proj, UNDIRECTED);
  }
  else if (m_type.test(INTERSECTING))
  {
    m_dist = Scalar(0.0);
    const Point3 proj = project(axis.getSupport(), P);

    if (m_type.test(SAME_DIRECTION))
    {
      // Elements intersect in two lines
      const Scalar offset(sqrt(sq(radius) - sq(rel.getDistance())));
      const Unit3 dir = cross(axis.getDirection(), P.getNormal());
      const Point3 sup1 = proj + offset * dir;
      const Point3 sup2 = proj - offset * dir;
      m_L1.setValue(axis.getDirection(), sup1, UNDIRECTED);
      m_L2.setValue(axis.getDirection(), sup2, UNDIRECTED);
    }
    else if (m_type.test(PERPENDICULAR))
    {
      // Elements intersect in a circle
      m_c.setValue(rel.getIntersectionPoint(),
                   radius,
                   axis.getDirection());
    }
  }
}


// ACCESS

inline Line3 RelPlaneCylinder::getTangentLine() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Plane and cylinder are not tangent."));

  return m_L1;
}


inline Circle3 RelPlaneCylinder::getIntersectionCircle() const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Plane and cylinder do not intersect in a circle."));

  return m_c;
}


inline Line3 RelPlaneCylinder::getIntersectionLine(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Plane and cylinder do not intersect in two lines."));

  return (n == 1) ? m_L1 : m_L2;
}


// PRIVATE METHODS


inline RelType RelPlaneCylinder::distanceType(const Scalar& dist)
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

inline std::ostream& operator<<(std::ostream&           os,
                                const RelPlaneCylinder& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentLine();
  }
  else if (type.test(INTERSECTING))
  {
    if (type.test(SAME_DIRECTION))
    {
      os << ", intersections: " << rel.getIntersectionLine(1) << " "
                                << rel.getIntersectionLine(2);
    }
    else if (type.test(PERPENDICULAR))
    {
      os << ", intersection: " << rel.getIntersectionCircle();
    }
  }

  return os;
}


} // mt

#endif // MT_REL_PLANE_CYLINDER_H
