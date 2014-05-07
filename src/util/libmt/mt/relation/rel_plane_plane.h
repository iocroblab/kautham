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
#ifndef MT_REL_PLANE_PLANE_H
#define MT_REL_PLANE_PLANE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/plane3.h>
#include <mt/line3.h>
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
/// \brief Computes the geometric relation between two planes.

class RelPlanePlane : public RelBase,
                      public RelBaseAng,
                      public RelBaseDist
{
public:

// LIFECYCLE

  RelPlanePlane(const Plane3& P1,
                const Plane3& P2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used


// ACCESS

  /// Intersection between elements.
  /// If the elements do not intersect, a mt::Exception exception is thrown.
  Line3 getIntersectionLine() const;


private:

  // MEMBERS
  Line3 m_intersection;

  // METHODS
  void intersection(const Plane3& P1,
                    const Plane3& P2);

  /// This function requires the angular part of the relation type to be
  /// already set, since it uses it.
  RelType distanceType(const Scalar& dist);
};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance and angle (in degrees) between elements.
std::ostream& operator<<(std::ostream&        os,
                         const RelPlanePlane& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPlanePlane::RelPlanePlane(const Plane3& P1,
                                    const Plane3& P2) :

                                    RelBaseAng(angle(P1.getNormal(),
                                                     P2.getNormal()))
{
  // Angle type
  m_type = angleType(m_angle);

  // Distance between planes
  if (m_type.test(SAME_DIRECTION))
  {
    // Parallel/antiparallel planes
    m_dist = distance(P1.getSupport(),
                      P2.getSupport());
  }
  else
  {
    // Planes intersect
    m_dist = 0.0;
    intersection(P1, P2);
  }

  // Distance type
  m_type |= distanceType(m_dist);
}


// ACCESS

inline Line3 RelPlanePlane::getIntersectionLine() const
{
  util::Assert(m_type.test(INTERSECTING),
               Exception("Planes do not intersect in a line."));

  return m_intersection;
}


// PRIVATE METHODS

inline void RelPlanePlane::intersection(const Plane3& P1,
                                        const Plane3& P2)
{
  // Plane parameters
  const Unit3  norm1(P1.getNormal());
  const Scalar dist1(P1.getDistOrig());

  const Unit3  norm2(P2.getNormal());
  const Scalar dist2(P2.getDistOrig());

  // Intersection line direction vector
  const Unit3 int_dir = cross(norm1, norm2);

  // Intersection line support point
  const Matrix3x3 A (  norm1[0],   norm1[1],   norm1[2],
                       norm2[0],   norm2[1],   norm2[2],
                     int_dir[0], int_dir[1], int_dir[2]);

  const Vector3 b(-dist1,
                  -dist2,
                   Scalar(0.0));

  const Point3 int_sup(inverse(A) * b);

  // Intersection  line
  m_intersection.setValue(int_dir,
                          int_sup,
                          UNDIRECTED);
}


inline RelType RelPlanePlane::distanceType(const Scalar& dist)
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

inline std::ostream& operator<<(std::ostream&        os,
                                const RelPlanePlane& rel)
{
  os << "distance: " << rel.getDistance() << ", "
     << "angle: "    << radToDeg(rel.getAngle());

  const RelType type(rel.getType());

  if (type.test(INTERSECTING))
  {
    // Line intersects plane
    os << ", intersection: " << rel.getIntersectionLine();
  }

  return os;
}


} // mt

#endif // MT_REL_PLANE_PLANE_H
