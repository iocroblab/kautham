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
#ifndef MT_REL_POINT_CIRCLE_H
#define MT_REL_POINT_CIRCLE_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/circle3.h>
#include <mt/point3.h>
#include <mt/relation/rel_base.h>
#include <mt/relation/rel_base_dist.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup rel

/// \ingroup rel
/// \brief Computes the geometric relation between a point and a circle.

class RelPointCircle :  public RelBase,
                        public RelBaseDist
{
public:

// LIFECYCLE

  RelPointCircle(const Point3&  p,
                 const Circle3& c);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Outputs the distance between elements.
std::ostream& operator<<(std::ostream&         os,
                         const RelPointCircle& rel);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline RelPointCircle::RelPointCircle(const Point3&  p,
                                      const Circle3& c) :

                                      RelBaseDist(distance(p, c))
{
   if (m_dist == Scalar(0.0))
  {
    m_type |= CONTAINED;
  }
  else
  {
    m_type |= (p == c.getCenter()) ? CONCENTRIC : DISTANCE;
  }
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&         os,
                                const RelPointCircle& rel)
{
  return os << "distance: " << rel.getDistance();
}


} // mt

#endif // MT_REL_POINT_CIRCLE_H
