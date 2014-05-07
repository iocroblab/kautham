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
#ifndef MT_REL_LINE_LINE_H
#define MT_REL_LINE_LINE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/line3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_ang.h>
#include <mt/relation/rel_base_dist.h>

// PROJECT HEADERS
#include <mt/util/assert/assert_template.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between two lines.

class RelLineLine : public RelBase,
                    public RelBaseAng,
                    public RelBaseDist
{
public:

// LIFECYCLE

  RelLineLine(const Line3& L1,
              const Line3& L2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Gets the closest points between two skew or intersecting lines.
  /// If the input parameter \a n = 1, the return value corresponds to the
  /// point of the \e first line that is closest to the \e second one.
  /// Otherwise the return value corresponds to the point of the \e second
  /// line that is closest to the \e first one.
  ///
  /// If the involved lines are not skew or intersecting, then a
  /// mt::Exception exception is thrown.
  Point3 getClosestPoint(size_t n = 1) const;

  /// Gets the intersection point between two lines. Since the intersection
  /// is obtained using tolerance-based tests, two lines are considered
  /// to intersect in a point if the distance that separates them tests equal
  /// to zero (see the basic_scalar.h file documentation). The input parameter
  /// serves the same purpose as in the getClosestPoint() method.
  ///
  /// If the involved lines are not skew or intersecting, then a
  /// mt::Exception exception is thrown.
  Point3 getIntersectionPoint(size_t n = 1) const;


private:

  // MEMBERS
  Point3 m_closest1;
  Point3 m_closest2;


  // METHODS
  void closestPoints(const Line3& L1,
                     const Line3& L2);

  /// This function requires the angular part of the relation type to be
  /// already set, since it uses it.
  RelType distanceType(const Scalar& dist);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&      os,
                         const RelLineLine& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelLineLine::RelLineLine(const Line3& L1,
                                const Line3& L2) :

                                RelBaseAng(angle(L1.getDirection(),
                                                 L2.getDirection()))
{
  // Angle type
  const DirectionType type =
  (L1.getDirectionType() == DIRECTED && L2.getDirectionType() == DIRECTED) ?
  DIRECTED : UNDIRECTED;
  m_type = angleType(m_angle, type);

  // Distance between lines
  if (m_type.test(SAME_DIRECTION))
  {
    // Parallel/antiparallel lines
    m_dist = distance(L1.getSupport(),
                      L2.getSupport());
  }
  else
  {
    // Skew/intersecting lines
    closestPoints(L1, L2);
  }

  // Distance type
  m_type |= distanceType(m_dist);
}


// ACCESS

inline Point3 RelLineLine::getClosestPoint(size_t n) const
{
  util::Assert(!m_type.test(SAME_DIRECTION),
               Exception("Lines are not skew/intersecting"));

  return (n == 1) ? m_closest1 : m_closest2;
}


inline Point3 RelLineLine::getIntersectionPoint(size_t n) const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Lines do not intersect."));

  return (n == 1) ? m_closest1 : m_closest2;
}


// PRIVATE METHODS

inline void RelLineLine::closestPoints(const Line3& L1,
                                       const Line3& L2)
{
  // Line direction vectors
  const Unit3 dir1(L1.getDirection());
  const Unit3 dir2(L2.getDirection());

  // Line support points
  const Point3 sup1(L1.getSupport());
  const Point3 sup2(L2.getSupport());

  // Unit vector normal to both lines
  const Unit3 normal(cross(dir1, dir2));

  // Difference between line support points
  Vector3 diff(sup2 - sup1);

  // Distance between lines
  const Scalar dist = dot(diff, normal);
  m_dist = abs(dist);

  // Difference between line support points computed with L2 translated the
  // distance between lines in the direction of the inverted normal.
  // Note that the signed distance "dist" is used instead of the unsigned
  // "m_dist"
  diff = (sup2 - dist * normal) - sup1;

  // Normal vector absolute value
  const Unit3 normal_abs(abs(normal));

  // Values of "t1" and "t2" are the line parameters at the intersection, and
  // are found by solving the linear system that arises from intersecting
  // L1 and the translated L2.
  // From the three possible sets of relations that solve the system,
  // the one with largest denominator is chosen to prevent ill-defined
  // results or divisions by zero.

  Scalar t1;
  Scalar t2;

  if (normal_abs[2] > normal_abs[0] &&
      normal_abs[2] > normal_abs[1])
  {
    t1 = (diff[0] * dir2[1] - diff[1] * dir2[0]) / normal[2];
    t2 = (diff[0] * dir1[1] - diff[1] * dir1[0]) / normal[2];
  }
  else if (normal_abs[0] > normal_abs[1])
  {
    t1 = (diff[1] * dir2[2] - diff[2] * dir2[1]) / normal[0];
    t2 = (diff[1] * dir1[2] - diff[2] * dir1[1]) / normal[0];
  }
  else
  {
    t1 = (diff[2] * dir2[0] - diff[0] * dir2[2]) / normal[1];
    t2 = (diff[2] * dir1[0] - diff[0] * dir1[2]) / normal[1];
  }

  // Cartesian values for closest line points
  m_closest1 = L1.getPoint(t1);
  m_closest2 = L2.getPoint(t2);
}


inline RelType RelLineLine::distanceType(const Scalar& dist)
{
  if (dist == Scalar(0.0))
  {
    if (m_type.test(SAME_DIRECTION))
    {
      return COINCIDENT;
    }
    else
    {
      return INTERSECTING;
    }
  }
  else
  {
    return DISTANCE;
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&      os,
                                const RelLineLine& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(DISTANCE) && !type.test(SAME_DIRECTION))
  {
    os << ", closest points: " << rel.getClosestPoint(1) << " "
                               << rel.getClosestPoint(2);
  }
  else if (type.test(INTERSECTING))
  {
    // Intersecting lines
    os << ", intersection: " << rel.getIntersectionPoint();
  }
  return os;
}


} // mt

#endif // MT_REL_LINE_LINE_H
