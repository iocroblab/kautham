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
#ifndef MT_TRANSFORM_UTILITY_H
#define MT_TRANSFORM_UTILITY_H


// MT LIBRARY HEADERS
#include <mt/empty.h>
#include <mt/circle3.h>
#include <mt/cylinder3.h>
#include <mt/ellipse3.h>
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/scalar.h>
#include <mt/sphere3.h>
#include <mt/unit3.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

/// \file transform_utility.h
/// This file contains functions for applying rigid transformations to
/// different geometric elements such as vectors, points, lines, planes,
/// spheres, cylinders, circles, and ellipses.

/// Applies transform to the Empty data structure.
Empty apply(const Transform& t,
            const Empty&     e);

/// Applies transform to input vector.
Vector3 apply(const Transform& t,
              const Vector3&   v);

/// Applies transform to input unit vector.
/// Only the rotational part of the transform is involved in this operation.
Unit3 apply(const Transform& t,
            const Unit3&     u);

/// Applies transform to input point.
Point3 apply(const Transform& t,
             const Point3&    p);

/// Applies transform to input line.
Line3 apply(const Transform& t,
            const Line3&     L);

/// Applies transform to input plane.
Plane3 apply(const Transform& t,
             const Plane3&    P);

/// Applies transform to input sphere.
Sphere3 apply(const Transform& t,
              const Sphere3&   s);

/// Applies transform to input cylinder.
Cylinder3 apply(const Transform& t,
                const Cylinder3& c);

/// Applies transform to input circle.
Circle3 apply(const Transform& t,
              const Circle3&   c);

/// Applies transform to input ellipse.
Ellipse3 apply(const Transform& t,
               const Ellipse3&  e);


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////


inline Empty apply(const Transform& t,
                   const Empty&     e)
{
  return e;
}


inline Vector3 apply(const Transform& t,
                     const Vector3&   v)
{
  return t(v);
}


inline Unit3 apply(const Transform& t,
                   const Unit3&     u)
{
  const Rotation rotation(t.getRotation());

  return Unit3 (rotation(u));
}


inline Point3 apply(const Transform& t,
                    const Point3&    p)
{
  return t(p);
}


inline Line3 apply(const Transform& t,
                   const Line3&     L)
{
  const Unit3  dir(apply(t, L.getDirection()));
  const Point3 sup(apply(t, L.getSupport()));

  return Line3(dir, sup, L.getDirectionType());
}


inline Plane3 apply(const Transform& t,
                    const Plane3&    P)
{
  const Unit3  nor(apply(t, P.getNormal()));
  const Point3 sup(apply(t, P.getSupport()));

  return Plane3(nor, sup, P.getDirectionType());
}


inline Sphere3 apply(const Transform& t,
                     const Sphere3&   s)
{
  const Point3 center(apply(t, s.getCenter()));

  return Sphere3(center, s.getRadius());
}


inline Cylinder3 apply(const Transform& t,
                       const Cylinder3& c)
{
  const Line3 axis(apply(t, c.getAxis()));

  return Cylinder3(axis, c.getRadius());
}


inline Circle3 apply(const Transform& t,
                     const Circle3&   c)
{
  const Point3 center(apply(t, c.getCenter()));
  const Unit3  normal(apply(t, c.getNormal()));

  return Circle3(center, c.getRadius(), normal);
}


inline Ellipse3 apply(const Transform& t,
                      const Ellipse3&  e)
{
  const Rotation rotation(t.getRotation());
  const Point3   center(apply(t, e.getCenter()));
  const Vector3  axis1(rotation(e.getAxis(1)));
  const Vector3  axis2(rotation(e.getAxis(2)));

  return Ellipse3(center, axis1, axis2);
}


} // mt


#endif // MT_TRANSFORM_UTILITY_H
