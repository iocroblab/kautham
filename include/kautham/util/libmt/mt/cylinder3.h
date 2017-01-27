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
#ifndef MT_CYLINDER3_H
#define MT_CYLINDER3_H

// C++ STANDARD HEADERS
#include <iostream>
#include <limits>

// MT LIBRARY HEADERS
#include <mt/exception.h>
#include <mt/line3.h>
#include <mt/point3.h>
#include <mt/scalar.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup elements

/// \ingroup elements
/// \brief Cylinder class.
///
/// This class represents a cylinder with circular cross-section of infinite
/// length.
///
/// Cylinders are represented by a line (axis) and a radius.
/// A point \f$ \mathbf{p} \f$ belongs to the cylinder
/// \f$ \mathbf{C}(\mathcal{L}, r) \f$
/// if \f$ d(\mathbf{p}, \mathcal{L}) = r \f$, where \f$ \mathcal{L} \f$ and
/// \f$ r \f$ represent the cylinder axis and radius, respectively.

class Cylinder3
{
public:

// LIFECYCLE

  /// Default constructor. Creates a cylinder with unit radius along the \e z
  /// axis.
  Cylinder3();

  /// Axis and radius constructor.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  Cylinder3(const Line3& axis, const Scalar& radius);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Cylinder3& c) const;
  bool operator!=(const Cylinder3& c) const;

// OPERATIONS

  /// Projects point \a p on current cylinder.
  Point3 project(const Point3& p) const;

  /// \e Signed distance from point \a p to current cylinder.
  ///
  /// Positive distance -> Point lies \e outside the cylinder.
  /// Null distance -> Point is \e contained in cylinder.
  /// Negative distance -> Point lies \e inside the cylinder.
  Scalar distance(const Point3& p) const;

// ACCESS

  Line3 getAxis() const;
  Line3& getAxisRef();
  const Line3& getAxisRef() const;

  Scalar getRadius() const;
  Scalar& getRadiusRef();
  const Scalar& getRadiusRef() const;

  void setAxis(const Line3& axis);

  /// Sets cylinder radius.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setRadius(const Scalar& radius);

  /// Sets cylinder parameters.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setValue(const Line3& axis, const Scalar& radius);


// INQUIRY

  /// Returns \e true if cylinder has null radius.
  bool isSingular() const;


private:

// MEMBERS

/// Cylinder center
Line3 m_axis;

/// Cylinder radius
Scalar m_radius;

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&    os,
                         const Cylinder3& c);


// FUNCTIONS

/// Projects point \a p on cylinder \a c.
Point3 project(const Point3&    p,
               const Cylinder3& c);

/// Distance from point \a p to cylinder \a c.
Scalar distance(const Point3&    p,
                const Cylinder3& c);

/// Distance from point \a p to cylinder \a c.
Scalar distance(const Sphere3&    s,
                const Cylinder3&  c);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Cylinder3::Cylinder3() :

                        m_axis(Line3()),
                        m_radius(1.0) {}

inline Cylinder3::Cylinder3(const Line3& axis, const Scalar& radius) :

                        m_axis(axis),
                        m_radius(abs(radius)) {}


// OPERATORS

inline bool Cylinder3::operator==(const Cylinder3& c) const
{
  return (m_axis == c.m_axis) && (m_radius == c.m_radius);
}


inline bool Cylinder3::operator!=(const Cylinder3& c) const
{
  return !(*this == c);
}


// OPERATIONS

inline Point3 Cylinder3::project(const Point3& p) const
{
  // Projection of input point on cylinder axis
  Point3 proj_axis(m_axis.project(p));

  // If the point to be projected coincides with the cylinder axis, its value is
  // perturbed to obtain a particular projection point.
  const value_t ep = std::numeric_limits<value_t>::epsilon();
  if (getValue(m_axis.distance(proj_axis)) < ep)
  {
    // Perturbation vector (its direction is perpendicular to cylinder axis)
    const Plane3  P(m_axis.getDirection(), 0.0);
    const Vector3 pert = Scalar(1000.0) * ep * P.getSupportVector();
    proj_axis += pert;
  }

  // Projection of point on cylinder
  if (m_radius == Scalar(0.0))
  {
    // Singular cylinder
    return proj_axis;
  }
  else
  {
    // Non-singular cylinder
    const Unit3 dist_axis(p - proj_axis);
    return proj_axis + m_radius * dist_axis;
  }
}


inline Scalar Cylinder3::distance(const Point3& p) const
{
  // Projection of input point on cylinder axis
  const Point3 proj_axis(m_axis.project(p));

  // Signed point-cylinder distance
  const Scalar dist_axis(length(p - proj_axis));
  return dist_axis - m_radius;
}


// ACCESS

inline Line3 Cylinder3::getAxis() const
{
  return m_axis;
}


inline Line3& Cylinder3::getAxisRef()
{
  return m_axis;
}


inline const Line3& Cylinder3::getAxisRef() const
{
  return m_axis;
}


inline Scalar Cylinder3::getRadius() const
{
  return m_radius;
}


inline Scalar& Cylinder3::getRadiusRef()
{
  return m_radius;
}


inline const Scalar& Cylinder3::getRadiusRef() const
{
  return m_radius;
}


inline void Cylinder3::setAxis(const Line3& axis)
{
  m_axis = axis;
}


inline void Cylinder3::setRadius(const Scalar& radius)
{
  m_radius = abs(radius);
}


inline void Cylinder3::setValue(const Line3& axis, const Scalar& radius)
{
  m_axis   = axis;
  m_radius = abs(radius);
}


// INQUIRY

inline bool Cylinder3::isSingular() const
{
  return m_radius == Scalar(0.0);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&    os,
                                const Cylinder3& c)
{
  return os << "axis: "   << c.getAxis() << ' '
            << "radius: " << c.getRadius();
}


// FUNCTIONS

inline Point3 project(const Point3&    p,
                      const Cylinder3& c)
{
  return c.project(p);
}


inline Scalar distance(const Point3&    p,
                       const Cylinder3& c)
{
  return c.distance(p);
}


inline Scalar distance(const Cylinder3& c,
                       const Point3&    p)
{
  return c.distance(p);
}

} // mt

#endif // MT_CYLINDER3_H
