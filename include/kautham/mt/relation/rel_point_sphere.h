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
#ifndef MT_REL_POINT_SPHERE_H
#define MT_REL_POINT_SPHERE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <kautham/mt/point3.h>
#include <kautham/mt/sphere3.h>
#include <kautham/mt/relation/rel_base.h>
#include <kautham/mt/relation/rel_base_dist.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a point and a sphere.
///
/// The computed distance between elements is a \e signed distance, with
/// negative values indicating that the point is \e contained in the sphere.

class RelPointSphere :  public RelBase,
                        public RelBaseDist
{
public:

// LIFECYCLE

  RelPointSphere(const Point3&  p,
                 const Sphere3& s);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&         os,
                         const RelPointSphere& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPointSphere::RelPointSphere(const Point3&  p,
                                      const Sphere3& s) :

                                      RelBaseDist(distance(p, s))
{
  if (m_dist == Scalar(0.0))
  {
    m_type |= CONTAINED;
  }
  else
  {
    m_type |= (p == s.getCenter()) ? CONCENTRIC : DISTANCE;
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&         os,
                                const RelPointSphere& rel)
{
  return os << "distance: " << rel.getDistance();
}


} // mt

#endif // MT_REL_POINT_SPHERE_H
