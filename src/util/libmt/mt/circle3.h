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
#ifndef MT_CIRCLE3_H
#define MT_CIRCLE3_H

// C++ STANDARD HEADERS
#include <iostream>
#include <limits>

// MT LIBRARY HEADERS
#include <mt/exception.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/scalar.h>
#include <mt/sphere3.h>
#include <mt/unit3.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup elements

/// \ingroup elements
/// \brief Three-dimensional circle class.
///
/// A circle is represented by a centerpoint, a radius, and a normal unit
/// vector.
///
/// A point \f$ \mathbf{p} \f$ belongs to the circle
/// \f$ \mathbf{C}(\mathbf{c}, r, \mathbf{\hat n}) \f$
/// if it satisfies simultaneously
/// \f$ \Vert\overrightarrow{\mathbf{pc}}\Vert = r \f$ and
/// \f$ \overrightarrow{\mathbf{pc}} \cdot \mathbf{\hat n} = 0 \f$,
/// where \f$ \mathbf{c} \f$, \f$ r \f$, and \f$ \mathbf{\hat n} \f$ represent
/// the circle centerpoint, radius, and normal, respectively.

class Circle3
{
public:

// LIFECYCLE

  /// Default constructor. Creates a unit circle in the \e xy plane.
  Circle3();

  /// Centerpoint, radius, and normal vector constructor.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  Circle3(const Point3& point,
          const Scalar& radius,
          const Unit3&  normal);

  /// Constructor for sphere and support plane input.
  /// The circle is defined as the intersection of the two surfaces.
  /// If the intersection is null, a run-time exception is thrown.
  Circle3(const Sphere3& sphere, const Plane3& plane);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Circle3& c) const;
  bool operator!=(const Circle3& c) const;

// OPERATIONS

  /// Projects point \a p on current circle.
  Point3 project(const Point3& p) const;

  /// Unsigned distance from point \a p to current circle.
  Scalar distance(const Point3& p) const;

// ACCESS

  Point3 getCenter() const;
  Point3& getCenterRef();
  const Point3& getCenterRef() const;

  Scalar getRadius() const;
  Scalar& getRadiusRef();
  const Scalar& getRadiusRef() const;

  Unit3 getNormal() const;
  Unit3& getNormalRef();
  const Unit3& getNormalRef() const;

  Plane3 getSupportPlane() const;

  void setCenter(const Point3& center);

  /// Sets circle radius.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setRadius(const Scalar& radius);

  void setNormal(const Unit3& normal);

  /// Sets circle parameters.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setValue(const Point3& center,
                const Scalar& radius,
                const Unit3&  normal);


// INQUIRY

  /// Returns \e true if circle has null radius.
  bool isSingular() const;


private:

// MEMBERS

/// Circle center
Point3 m_center;

/// Circle radius
Scalar m_radius;

/// Circle normal direction
Unit3 m_normal;

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&  os,
                         const Circle3& c);


// FUNCTIONS

/// Projects point \a p on circle \a c.
Point3 project(const Point3&  p,
               const Circle3& c);

/// Distance from point \a p to circle \a c.
Scalar distance(const Point3&  p,
                const Circle3& c);

/// Distance from point \a p to circle \a c.
Scalar distance(const Circle3& s,
                const Point3&  c);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Circle3::Circle3() :

                        m_center(0.0, 0.0, 0.0),
                        m_radius(1.0),
                        m_normal(0.0, 0.0, 1.0) {}


inline Circle3::Circle3(const Point3& center,
                        const Scalar& radius,
                        const Unit3&  normal) :

                        m_center(center),
                        m_radius(abs(radius)),
                        m_normal(normal) {}


inline Circle3::Circle3(const Sphere3& sphere, const Plane3& plane)
{
  // Sphere parameters
  const Point3 center(sphere.getCenter());
  const Scalar radius(sphere.getRadius());

  // Projection of sphere center on plane
  const Point3 proj_center(plane.project(center));

  // Distance from sphere center to plane
  const Scalar dist(length(proj_center - center));

  // Checks that plane and sphere intersect
  util::Assert(dist <= radius, Exception("Cannot construct circle, \
input plane and sphere do not intersect"));

  // Constructs circle
  const Scalar circle_radius(sqrt(sq(radius) - sq(dist)));
  setValue(proj_center,
           circle_radius,
           plane.getNormal());

}


// OPERATORS

inline bool Circle3::operator==(const Circle3& c) const
{
  return (m_center == c.m_center) &&
         (m_radius == c.m_radius) &&
         (m_normal == c.m_normal || m_normal == -c.m_normal);
}


inline bool Circle3::operator!=(const Circle3& c) const
{
  return !(*this == c);
}


// OPERATIONS

inline Point3 Circle3::project(const Point3& p) const
{
  // Projection of input point on circle support plane
  const Plane3 sup = getSupportPlane();
  Point3 proj_p(sup.project(p));

  // If the point to be projected coincides with circle center, its value is
  // perturbed to obtain a particular projection point.
  const value_t ep = std::numeric_limits<value_t>::epsilon();
  if (getValue(m_center.distance(proj_p)) < ep)
  {
    // Perturbation vector (its direction is perpendicular to circle normal)
    const Vector3 pert = Scalar(1000.0) * ep * sup.getSupportVector();
    proj_p += pert;
  }

  if (m_radius == Scalar(0.0))
  {
    // Singular sphere
    return m_center;
  }
  else
  {
    // Non-singular circle
    const Unit3 dist_center(proj_p - m_center);
    return m_center + m_radius * dist_center;
  }
}


inline Scalar Circle3::distance(const Point3& p) const
{
  // Projection of input point on circle support plane
  const Plane3 sup = getSupportPlane();
  const Point3 proj_p(sup.project(p));

  if (proj_p != m_center)
  {
    return length(p - project(p));
  }
  else {
    const Scalar proj_p_dist(sup.distance(p));
    return sqrt(sq(m_radius) + sq(proj_p_dist));
  }
}


// ACCESS

inline Point3 Circle3::getCenter() const
{
  return m_center;
}


inline Point3& Circle3::getCenterRef()
{
  return m_center;
}


inline const Point3& Circle3::getCenterRef() const
{
  return m_center;
}


inline Scalar Circle3::getRadius() const
{
  return m_radius;
}


inline Scalar& Circle3::getRadiusRef()
{
  return m_radius;
}


inline const Scalar& Circle3::getRadiusRef() const
{
  return m_radius;
}


inline Unit3 Circle3::getNormal() const
{
  return m_normal;
}


inline Unit3& Circle3::getNormalRef()
{
  return m_normal;
}


inline const Unit3& Circle3::getNormalRef() const
{
  return m_normal;
}


inline Plane3 Circle3::getSupportPlane() const
{
  return Plane3(m_normal, m_center);
}


inline void Circle3::setCenter(const Point3& center)
{
  m_center = center;
}


inline void Circle3::setRadius(const Scalar& radius)
{
  m_radius = abs(radius);
}


inline void Circle3::setNormal(const Unit3& normal)
{
  m_normal = normal;
}


inline void Circle3::setValue(const Point3& center,
                              const Scalar& radius,
                              const Unit3&  normal)
{
  m_center = center;
  m_radius = abs(radius);
  m_normal = normal;
}


// INQUIRY

inline bool Circle3::isSingular() const
{
  return m_radius == Scalar(0.0);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&  os,
                                const Circle3& c)
{
  return os << "center: "        << c.getCenter() << ' '
            << "radius: "        << c.getRadius() << ' '
            << "normal: "        << c.getNormal();
}


// FUNCTIONS

inline Point3 project(const Point3&  p,
                      const Circle3& c)
{
  return c.project(p);
}


inline Scalar distance(const Point3&  p,
                       const Circle3& c)
{
  return c.distance(p);
}


inline Scalar distance(const Circle3& c,
                       const Point3&  p)
{
  return c.distance(p);
}

} // mt

#endif // MT_CIRCLE3_H
