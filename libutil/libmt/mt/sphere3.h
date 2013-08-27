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
#ifndef MT_SPHERE3_H
#define MT_SPHERE3_H

// C++ STANDARD HEADERS
#include <iostream>
#include <limits>

// MT LIBRARY HEADERS
#include <mt/exception.h>
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
/// \brief Sphere class.
///
/// Spheres are represented by a centerpoint and a radius.
/// A point \f$ \mathbf{p} \f$ belongs to the sphere
/// \f$ \mathbf{S}(\mathbf{c}, r) \f$ if
/// \f$ d(\mathbf{p}, \mathbf{c}) = r \f$, where \f$ \mathbf{c} \f$ and
/// \f$ r \f$ represent the sphere centerpoint and radius, respectively.

class Sphere3
{
public:

// LIFECYCLE

  /// Default constructor. Creates the unit sphere.
  Sphere3();

  /// Centerpoint and radius constructor.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  Sphere3(const Point3& center, const Scalar& radius);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Sphere3& s) const;
  bool operator!=(const Sphere3& s) const;

// OPERATIONS

  /// Projects point \a p on current sphere.
  Point3 project(const Point3& p) const;

  /// \e Signed distance from point \a p to current sphere.
  ///
  /// Positive distance -> Point lies \e outside the sphere.
  /// Null distance -> Point is \e contained in sphere.
  /// Negative distance -> Point lies \e inside the sphere.
  Scalar distance(const Point3& p) const;

// ACCESS

  Point3 getCenter() const;
  Point3& getCenterRef();
  const Point3& getCenterRef() const;

  Scalar getRadius() const;
  Scalar& getRadiusRef();
  const Scalar& getRadiusRef() const;

  void setCenter(const Point3& center);

  /// Sets sphere radius.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setRadius(const Scalar& radius);

  /// Sets sphere parameters.
  /// Since the radius \e must be a positive value, its absolute value is
  /// taken (just in case) \e without issuing warnings or errors.
  void setValue(const Point3& center, const Scalar& radius);


// INQUIRY

  /// Returns \e true if sphere has null radius.
  bool isSingular() const;


private:

// MEMBERS

/// Sphere center
Point3 m_center;

/// Sphere radius
Scalar m_radius;

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&  os,
                         const Sphere3& s);


// FUNCTIONS

/// Projects point \a p on sphere \a s.
Point3 project(const Point3&  p,
               const Sphere3& s);

/// Distance from point \a p to sphere \a s.
Scalar distance(const Point3&  p,
                const Sphere3& s);

/// Distance from point \a p to sphere \a s.
Scalar distance(const Sphere3& s,
                const Point3&  p);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Sphere3::Sphere3() :

                        m_center(0.0, 0.0, 0.0),
                        m_radius(1.0) {}

inline Sphere3::Sphere3(const Point3& center, const Scalar& radius) :

                        m_center(center),
                        m_radius(abs(radius)) {}


// OPERATORS

inline bool Sphere3::operator==(const Sphere3& s) const
{
  return (m_center == s.m_center) && (m_radius == s.m_radius);
}


inline bool Sphere3::operator!=(const Sphere3& s) const
{
  return !(*this == s);
}


// OPERATIONS

inline Point3 Sphere3::project(const Point3& p) const
{
  // If the point to be projected coincides with sphere center, its value is
  // perturbed to obtain a particular projection point.

  Point3 p_temp = p;
  const value_t ep = std::numeric_limits<value_t>::epsilon();
  if (getValue(m_center.distance(p)) < ep)
  {
    // Perturbation vector (its direction is perpendicular to circle normal)
    const Vector3 pert = Scalar(1000.0) * ep * Unit3(1.0, 0.0, 0.0);
    p_temp += pert;
  }

  if (m_radius == Scalar(0.0))
  {
    // Singular sphere
    return m_center;
  }
  else
  {
    // Non-singular sphere
    const Unit3 dist_center(p_temp - m_center);
    return m_center + m_radius * dist_center;
  }
}


inline Scalar Sphere3::distance(const Point3& p) const
{
  const Scalar dist_center(length(p - m_center));
  return dist_center - m_radius;
}


// ACCESS

inline Point3 Sphere3::getCenter() const
{
  return m_center;
}


inline Point3& Sphere3::getCenterRef()
{
  return m_center;
}


inline const Point3& Sphere3::getCenterRef() const
{
  return m_center;
}


inline Scalar Sphere3::getRadius() const
{
  return m_radius;
}


inline Scalar& Sphere3::getRadiusRef()
{
  return m_radius;
}


inline const Scalar& Sphere3::getRadiusRef() const
{
  return m_radius;
}


inline void Sphere3::setCenter(const Point3& center)
{
  m_center = center;
}


inline void Sphere3::setRadius(const Scalar& radius)
{
  m_radius = abs(radius);
}


inline void Sphere3::setValue(const Point3& center, const Scalar& radius)
{
  m_center = center;
  m_radius = abs(radius);
}


// INQUIRY

inline bool Sphere3::isSingular() const
{
  return m_radius == Scalar(0.0);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&  os,
                                const Sphere3& s)
{
  return os << "center: " << s.getCenter() << ' '
            << "radius: " << s.getRadius();
}


// FUNCTIONS

inline Point3 project(const Point3&  p,
                      const Sphere3& s)
{
  return s.project(p);
}


inline Scalar distance(const Point3&  p,
                       const Sphere3& s)
{
  return s.distance(p);
}


inline Scalar distance(const Sphere3& s,
                       const Point3&  p)
{
  return s.distance(p);
}

} // mt

#endif // MT_SPHERE3_H
