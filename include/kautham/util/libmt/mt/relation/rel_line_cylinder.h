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
#ifndef MT_REL_LINE_CYLINDER_H
#define MT_REL_LINE_CYLINDER_H

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

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a line and a cylinder.
///
/// The computed distance between elements is a \e signed distance, with
/// negative values indicating that the line is \e contained in the cylinder.

class RelLineCylinder : public RelBase,
                        public RelBaseAng,
                        public RelBaseDist
{
public:

// LIFECYCLE

  RelLineCylinder(const Line3&     L,
                  const Cylinder3& c);

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
  /// If the input parameter \a n = 1, the returned point belongs to the line,
  /// otherwise the returned point belongs to the cylinder.
  Point3 getClosestPoint(size_t n = 1) const;

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


  // METHODS

  /// This function requires the angular part of the relation type to be
  /// already set, since it uses it.
  RelType distanceType(const Scalar& dist, const Scalar& radius);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&          os,
                         const RelLineCylinder& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelLineCylinder::RelLineCylinder(const Line3&     L,
                                        const Cylinder3& c) :

                                RelBaseAng(angle(L.getDirection(),
                                                 c.getAxis().getDirection()))
{
  // Angle type
  const DirectionType type =
  (L.getDirectionType() == DIRECTED &&
   c.getAxis().getDirectionType() == DIRECTED) ? DIRECTED : UNDIRECTED;

  m_type = angleType(m_angle, type);

  // Cylinder parameters
  const Line3  axis(c.getAxis());
  const Scalar radius(c.getRadius());

  // Relation between cylinder axis and line
  const RelLineLine rel(L, axis);
  //const RelType     rel_type = rel.getType();

  // Signed distance between cylinder and line
  const Scalar dist = rel.getDistance() - radius;

  // Distance type
  m_type |= distanceType(dist, radius);

  // Distance between elements and tangencies/intersections
  if (m_type.test(CONTAINED))
  {
    m_dist = Scalar(0.0);
  }
  else if (m_type.test(CONCENTRIC))
  {
    m_dist = dist;
  }
  else if (m_type.test(DISTANCE))
  {
    m_dist = dist;

    if (!m_type.test(SAME_DIRECTION))
    {
      const Unit3 dir = rel.getClosestPoint(1) - rel.getClosestPoint(2);
      m_p1 = rel.getClosestPoint(1);
      m_p2 = rel.getClosestPoint(2) + radius * dir;
    }
  }
  else if (m_type.test(TANGENT))
  {
    m_dist = Scalar(0.0);
    m_p1 = rel.getClosestPoint(1);
  }
  else if (m_type.test(INTERSECTING))
  {
    m_dist = Scalar(0.0);

    // Distance between closest points of line and cylinder axis
    const Point3 closest1 = rel.getClosestPoint(1);
    const Point3 closest2 = rel.getClosestPoint(2);

    const Scalar dist_closest = length(closest2 - closest1); 

    // Distance between projection point and intersections
    const Scalar dist_int(sqrt(sq(radius) - sq(dist_closest)));

    // Intersection points
    const Unit3 dir = L.getDirection();
    m_p1 = closest1 + dist_int * dir;
    m_p2 = closest1 - dist_int * dir;
  }
}


// ACCESS

inline Point3 RelLineCylinder::getClosestPoint(size_t n) const
{
  util::Assert(!m_type.test(SAME_DIRECTION) && m_type.test(DISTANCE),
               Exception("Elements are not skew"));

  return (n == 1) ? m_p1 : m_p2;
}


inline Point3 RelLineCylinder::getTangentPoint() const
{
  util::Assert(m_type.test(TANGENT),
               Exception("Elements are not tangent."));

  return m_p1;
}


inline Point3 RelLineCylinder::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Elements do not intersect"));

  return (n == 1) ? m_p1 : m_p2;
}


// PRIVATE METHODS


inline RelType RelLineCylinder::distanceType(const Scalar& dist,
                                             const Scalar& radius)
{
  if (dist > Scalar(0.0))
  {
    return DISTANCE;
  }
  else if (dist < Scalar(0.0))
  {
    if (m_type.test(SAME_DIRECTION))
    {
      return (dist == -radius) ? CONCENTRIC : DISTANCE;
    }
    else
    {
      return INTERSECTING;
    }
  }
  else
  {
    if (m_type.test(SAME_DIRECTION))
    {
      return CONTAINED;
    }
    else
    {
      return TANGENT;
    }
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&          os,
                                const RelLineCylinder& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(DISTANCE) && !type.test(SAME_DIRECTION))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(TANGENT))
  {
    os << ", tangency: " << rel.getTangentPoint();
  }
  else if (type.test(INTERSECTING))
  {
    os << ", intersections: " << rel.getIntersectionPoint(1) << " "
                              << rel.getIntersectionPoint(2);
  }

  return os;
}


} // mt

#endif // MT_REL_LINE_CYLINDER_H
