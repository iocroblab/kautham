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
#ifndef MT_ELLIPSE3_H
#define MT_ELLIPSE3_H

// C++ STANDARD HEADERS
#include <iostream>
#include <limits>
#include <utility>

// MT LIBRARY HEADERS
#include <mt/cylinder3.h>
#include <mt/exception.h>
#include <mt/line3.h>
#include <mt/matrix3x3.h>
#include <mt/transform.h>
#include <mt/plane3.h>
#include <mt/point3.h>
#include <mt/scalar.h>
#include <mt/unit3.h>
#include <mt/vector3.h>
#include <mt/relation/rel_line_plane.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup elements

/// \ingroup elements
/// \brief Three-dimensional ellipse class.
///
/// An ellipse is represented by a centerpoint \f$ \mathbf{c} \f$,
/// the semimajor and semiminor axis \f$ a \f$ and \f$ b \f$, and their
/// respective directions \f$ \mathbf{\hat d}_a \f$ and
/// \f$ \mathbf{\hat d}_b \f$.
///
/// A point \f$ \mathbf{p} \f$ belongs to the ellipse
/// \f$ \mathbf{E}(\mathbf{c}, \mathbf{\hat d}_a, \mathbf{\hat d}_b, a, b) \f$
/// if it satisfies
/// \f$ \frac{x^2}{a^2} + \frac{y^2}{b^2} = 1 \f$ where
/// \f$ \mathbf{\hat d}_a \f$ and \f$ \mathbf{\hat d}_b \f$ are perpendicular.


class Ellipse3
{
public:

// LIFECYCLE

  /// Default constructor. Creates a unit circle in the \e xy plane.
  Ellipse3();

  /// Centerpoint and semimajor-semiminor axis constructor.
  /// The axis are defined by non-unit vectors whose directions determine the
  /// ellipse plane, and whose length determines the ellipse size and shape.
  /// Also, the axis directions \e must be perpendicular, and if this
  /// condition is not satisfied an exception is thrown.
  Ellipse3(const Point3&  center,
           const Vector3& axis1,
           const Vector3& axis2);

  /// Constructor for plane and cylinder inputs.
  /// The ellipse is defined as the intersection of the two surfaces.
  /// If the elements do not intersect in an ellipse, an exception is thrown.
  Ellipse3(const Plane3& plane, const Cylinder3& cylinder);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Ellipse3& e) const;
  bool operator!=(const Ellipse3& e) const;

// OPERATIONS

  /// Projects point \a p on current ellipse.
  /// This function uses an interval-based numeric solver to obtain its value,
  /// so convergence is guarranteed, and rarely takes more than 5-6 iterations.
  Point3 project(const Point3& p) const;

  /// Unsigned distance from point \a p to current ellipse.
  Scalar distance(const Point3& p) const;

// ACCESS

  Point3 getCenter() const;
  Point3& getCenterRef();
  const Point3& getCenterRef() const;

  Unit3& getAxisDirRef(const size_t i = 1);
  const Unit3& getAxisDirRef(const size_t i = 1) const;

  Scalar& getAxisLengthRef(const size_t i = 1);
  const Scalar& getAxisLengthRef(const size_t i = 1) const;

  /// Gets ellipse axis.
  /// If \a i = 1, the function returns the first axis, and in all other cases
  /// it returns the second one.
  Vector3 getAxis(const size_t i = 1) const;

  /// Gets ellipse support plane
  Plane3 getSupportPlane() const;

  /// Gets one of the two possible cylinders that when intersected with the
  /// support plane gives rise to the current ellipse.
  Cylinder3 getSupportCylinder() const;

  Unit3 getNormal() const;

  void setCenter(const Point3& center);

  /// Sets ellipse semimajor and semiminor axis.
  /// If the axis vectors are not perpendicular or have null length, an
  /// exception is thrown.
  void setAxis(const Vector3& axis1,
               const Vector3& axis2);

  /// Sets ellipse parameters.
  /// If the axis vectors are not perpendicular or have null length, an
  /// exception is thrown.
  void setValue(const Point3& center,
                const Vector3& axis1,
                const Vector3& axis2);

  /// Sets ellipse parameters from plane and cylinder input
  /// If the elements do not intersect in an ellipse, an exception is thrown.
  void setValue(const Plane3&    plane,
                const Cylinder3& cylinder);


private:

// MEMBERS

/// Ellipse center
Point3 m_center;

/// Ellipse axis direction
Unit3 m_axis_dir1;

/// Ellipse axis direction
Unit3 m_axis_dir2;

/// Ellipse axis length
Scalar m_axis_len1;

/// Ellipse axis length
Scalar m_axis_len2;


// OPERATIONS

/// Projects input 2D point on standard position ellipse.
/// An ellipse in standard position has a center at the origin, and axis
/// aligned with the coordinate axes.
/// Any point \f$ \mathbf{P} = (x, y) \f$ on the ellipse satisfies
/// \f[ \frac{x^2}{a^2} + \frac{y^2}{b^2} = 1 \f]
///
/// The nearest point \f$ \mathbf{P}_n \f$ on the ellipse has the property that
/// the line from \f$ \mathbf{P}_n \f$ to \f$ \mathbf{P} \f$ is normal to the
/// ellipse.
/// Points on the ellipse can be parameterized by \f$ t \f$, to have the form
/// \f$ (a \cos(t), b \sin(T)) \f$.
///
/// The tangent vector to the ellipse has the form
///  \f$ (-a \sin(t), b \cos(t)) \f$.
/// At \f$ \mathbf{P}_n \f$, the dot product of this vector with
/// \f$ ( \mathbf{P} - \mathbf{P}_n ) \f$ must be zero:
/// \f[ -a \sin(t)(x - a \cos(t)) + b \cos(t) (y - b \sin(t)) = 0 \f]
/// This nonlinear equation for \f$ t \f$ can be solved by a bracketing method
/// such as regula falsi.
///
/// This function is based on the ELLIPSE_POINT_NEAR_2D function implemented by
/// John Burkardt as part of the GEOMETRY library
/// http://people.scs.fsu.edu/~burkardt/cpp_src/geometry/geometry.html
///
/// \param p 2D point to be projected on the ellipse
/// \param tol Tolerance for root-finding process
/// \return 2D projection point of \a p on ellipse

std::pair<Scalar, Scalar> project2d(const std::pair<Scalar, Scalar>& p,
                                    const Scalar&                    tol) const;

/// Evaluates the function whose root is to be found in the project2d function,
/// namely
/// \f[ -a \sin(t)(x - a \cos(t)) + b \cos(t) (y - b \sin(t)) = 0 \f]
Scalar project2dCore(const Scalar& t,
                     const Scalar& x,
                     const Scalar& y) const;
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&   os,
                         const Ellipse3& e);


// FUNCTIONS

/// Projects point \a p on ellipse \a e.
Point3 project(const Point3&   p,
               const Ellipse3& e);

/// Distance from point \a p to ellipse \a e.
Scalar distance(const Point3&   p,
                const Ellipse3& e);

/// Distance from point \a p to ellipse \a e.
Scalar distance(const Ellipse3& s,
                const Point3&   e);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Ellipse3::Ellipse3() :

                          m_center(0.0, 0.0, 0.0),
                          m_axis_dir1(Unit3(1.0, 0.0, 0.0)),
                          m_axis_dir2(Unit3(0.0, 1.0, 0.0)),
                          m_axis_len1(1.0),
                          m_axis_len2(1.0) {}


inline Ellipse3::Ellipse3(const Point3&  center,
                          const Vector3& axis1,
                          const Vector3& axis2)
{
  setValue(center, axis1, axis2);
}


inline Ellipse3::Ellipse3(const Plane3& plane, const Cylinder3& cylinder)
{
  setValue(plane, cylinder);
}


// OPERATORS

inline bool Ellipse3::operator==(const Ellipse3& e) const
{
  return (m_center    == e.m_center) &&
         (m_axis_dir1 == e.m_axis_dir1 || m_axis_dir1 == -e.m_axis_dir1) &&
         (m_axis_dir2 == e.m_axis_dir2 || m_axis_dir2 == -e.m_axis_dir2) &&
         (m_axis_len1 == e.m_axis_len1) &&
         (m_axis_len2 == e.m_axis_len2);
}


inline bool Ellipse3::operator!=(const Ellipse3& e) const
{
  return !(*this == e);
}


// OPERATIONS

inline Point3 Ellipse3::project(const Point3& p) const
{
  // Projection of input point on ellipse support plane
  const Plane3 sup = getSupportPlane();
  const Point3 p_pl(sup.project(p));

  // Orthonormal basis associated to ellipse
  Unit3 v;
  Unit3 w;
  orthonormalBasis(m_axis_dir1, m_axis_dir2, v, w);

  // Transformations between reference and ellipse frames
  const Matrix3x3 M(m_axis_dir1[0], v[0], w[0],
                    m_axis_dir1[1], v[1], w[1],
                    m_axis_dir1[2], v[2], w[2]);

  const Rotation  R(M);
  const Transform Tr_inv(R, m_center);
  const Transform Tr = Tr_inv.inverse();

  // Point proj_p in ellipse frame coordinates
  const Point3 p_el = Tr(p_pl);
  #ifdef MT_USE_BASIC_SCALAR
  util::Assert((p_el[2] == 0.0), Exception("Error projecting point on ellipse."));
  #endif

  // Projection in ellipse coordinates
  std::pair<Scalar, Scalar> p_el_2d(p_el[0], p_el[1]);

  const Scalar tol = Scalar(100.0) * std::numeric_limits<value_t>::epsilon();

  std::pair<Scalar, Scalar> p_proj_el_2d = project2d(p_el_2d, tol);

  const Point3 p_proj_el(p_proj_el_2d.first, p_proj_el_2d.second, 0.0);

  // Projection in reference frame coordinates
  return Tr_inv(p_proj_el);
}


inline Scalar Ellipse3::distance(const Point3& p) const
{
  // Projection of input point on ellipse support plane
  const Plane3 sup = getSupportPlane();
  const Point3 proj_p(sup.project(p));

  return length(p - project(p));
}


inline std::pair<Scalar, Scalar>
Ellipse3::project2d(const std::pair<Scalar, Scalar>& p,
                    const Scalar&                    tol) const
{
  // Point to be projected on ellipse
  Scalar x = abs(p.first);
  Scalar y = abs(p.second);

  // If point is inside the ellipse and contained in at least one of the
  // x or y axis, its value is perturbated so it does not belong to a
  // singular configuration with more than one solution
  if (y == 0.0 && x < m_axis_len1)
  {
    y = tol;
  }
  if (x == 0.0 && y < m_axis_len2)
  {
    x = tol;
  }

  // Parameter initialization for root finding
  Scalar ta = 0.0;          // Ellipse parameter (first bound)
  Scalar tb = HALF_PI;      // Ellipse parameter (second bound)
  Scalar tc;                // Ellipse parameter (evaluation point)
  Scalar fa = project2dCore(ta, x, y);  // Function evaluated at point a
  Scalar fb = project2dCore(tb, x, y);  // Function evaluated at point b
  Scalar fc;                // Function evaluated at point c (new point)
  size_t i;                 // Counter
  const size_t imax = 1000; // Maximum number of iterations

  // Rounds off fa and fb around zero (for stability of numerical method)
  if (getValue(abs(fa)) <= getValue(tol))
  {
    fa = 0.0;
  }
  if (getValue(abs(fb)) <= getValue(tol))
  {
    fb = 0.0;
  }

  // Checks that fa and fb have different signs, so a root bracketing method
  // can be applied
  util::Assert((getValue(fa) * getValue(fb) <= 0.0),
  Exception("Cannot find projection of point on ellipse. \
Root finding conditions not met"));

  // Regula falsi root finding algorithm
  for (i = 0 ; i < imax; ++i)
  {
    // Calculate new point and evaluate it in function
    tc = tb - fb * (tb - ta) / (fb - fa);
    fc = project2dCore(tc, x, y);

    // Update bracketing bounds
    if (getValue(fa) * getValue(fc) < 0.0)
    {
      tb = tc;
      fb = fc;
    }
    else
    {
      ta = tc;
      fa = fc;
    }

    // Convergence test
    if (getValue(abs(fc)) <= getValue(tol))
    {
      break;
    }
  }

  // Maximum iterations test
  util::Assert((i <= imax), Exception("Cannot find projection of point on \
ellipse. Reached iteration limit"));

  //  Take care of case where the point was in another quadrant
  const Scalar sgn_xp = sgn(p.first);
  const Scalar sgn_yp = sgn(p.second);

  // Projection point coordinates
  const Scalar xp = sgn_xp * m_axis_len1 * cos(tc);
  const Scalar yp = sgn_yp * m_axis_len2 * sin(tc);

  return std::pair<Scalar, Scalar>(xp, yp);
}


inline Scalar Ellipse3::project2dCore(const Scalar& t,
                                      const Scalar& x,
                                      const Scalar& y) const
{
  const Scalar ct = cos(t);
  const Scalar st = sin(t);

  // Function whose roots are to be found
  return (x - m_axis_len1 * ct) * m_axis_len1 * st
       - (y - m_axis_len2 * st) * m_axis_len2 * ct;
}


// ACCESS

inline Point3 Ellipse3::getCenter() const
{
  return m_center;
}


inline Point3& Ellipse3::getCenterRef()
{
  return m_center;
}


inline const Point3& Ellipse3::getCenterRef() const
{
  return m_center;
}


inline Unit3& Ellipse3::getAxisDirRef(const size_t i)
{
  return (i == 1) ? m_axis_dir1 : m_axis_dir2;
}


inline const Unit3& Ellipse3::getAxisDirRef(const size_t i) const
{
  return (i == 1) ? m_axis_dir1 : m_axis_dir2;
}


inline Scalar& Ellipse3::getAxisLengthRef(const size_t i)
{
  return (i == 1) ? m_axis_len1 : m_axis_len2;
}


inline const Scalar& Ellipse3::getAxisLengthRef(const size_t i) const
{
  return (i == 1) ? m_axis_len1 : m_axis_len2;
}


inline Vector3 Ellipse3::getAxis(const size_t i) const
{
  if (i == 1)
  {
    return m_axis_len1 * m_axis_dir1;
  }
  else
  {
    return m_axis_len2 * m_axis_dir2;
  }
}


inline Plane3 Ellipse3::getSupportPlane() const
{
  const Unit3 norm = getNormal();
  return Plane3(norm, m_center);
}

inline Cylinder3 Ellipse3::getSupportCylinder() const
{
  // Minimum and maximun axis
  Scalar min_len;
  Scalar max_len;
  Unit3  min_dir;
  Unit3  max_dir;

  if (m_axis_len1 < m_axis_len2)
  {
    min_len = m_axis_len1;
    max_len = m_axis_len2;
    min_dir = m_axis_dir1;
    max_dir = m_axis_dir2;
  }
  else
  {
    min_len = m_axis_len2;
    max_len = m_axis_len1;
    min_dir = m_axis_dir2;
    max_dir = m_axis_dir1;
  }

  // Angle between cylinder and support plane
  const Scalar angle = asin(min_len / max_len);

  // Cylinder axis
  const Rotation rot(min_dir, angle);
  const Unit3 dir = rot(max_dir);
  const Line3 axis(dir, m_center);

  // Support cylinder
  return Cylinder3(axis, min_len);
}


inline Unit3 Ellipse3::getNormal() const
{
  return cross(m_axis_dir1, m_axis_dir2);
}


inline void Ellipse3::setCenter(const Point3& center)
{
  m_center = center;
}


inline void Ellipse3::setAxis(const Vector3& axis1,
                              const Vector3& axis2)
{
  // Verifies that ellipse axis are perpendicular
  #ifdef MT_USE_BASIC_SCALAR
  const Scalar dotp = dot(Unit3(axis1), Unit3(axis2));
  util::Assert((dotp == 0.0),
  Exception("Cannot construct ellipse, axis are not perpendicular"));
  #endif

  // Updates axis-related data
  m_axis_dir1 = axis1;
  m_axis_dir2 = axis2;
  m_axis_len1 = axis1.length();
  m_axis_len2 = axis2.length();
}


inline void Ellipse3::setValue(const Point3& center,
                               const Vector3& axis1,
                               const Vector3& axis2)
{
  setCenter(center);
  setAxis(axis1, axis2);
}


inline void Ellipse3::setValue(const Plane3&    plane,
                               const Cylinder3& cylinder)
{
  // Cylinder parameters
  const Line3  axis(cylinder.getAxis());
  const Scalar radius(cylinder.getRadius());

  // Verifies that cylinder axis and plane intersect
  const RelLinePlane rel(axis, plane);
  const bool is_intersecting = rel.getType().test(INTERSECTING);
  util::Assert(is_intersecting, Exception("Cannot construct ellipse, \
  input cylinder and sphere do not intersect"));

  // Constructs ellipse
  const Scalar ang   = rel.getAngle();
  const Unit3  Pnorm = plane.getNormal();
  const Unit3  Ldir  = axis.getDirection();

  if (rel.getType().test(PERPENDICULAR))
  {
    m_axis_dir1 = plane.getSupportVector();
  }
  else
  {
    m_axis_dir1 = cross(Pnorm, Ldir);
  }
  m_axis_dir2 = cross(m_axis_dir1, Pnorm);
  m_axis_len1 = radius;
  m_axis_len2 = radius / sin(ang);
  m_center = rel.getIntersectionPoint();
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&  os,
                                const Ellipse3& e)
{
  const Vector3 axis1 = e.getAxis(1);
  const Vector3 axis2 = e.getAxis(2);

  const Unit3 axis_dir1 = axis1;
  const Unit3 axis_dir2 = axis2;

  const Scalar axis_len1 = axis1.length();
  const Scalar axis_len2 = axis2.length();

  return os << "center: " << e.getCenter() << ' '
            << "axis 1: " << axis_dir1     << " length " << axis_len1 << ' '
            << "axis 2: " << axis_dir2     << " length " << axis_len2;
}


// FUNCTIONS

inline Point3 project(const Point3&   p,
                      const Ellipse3& e)
{
  return e.project(p);
}


inline Scalar distance(const Point3&   p,
                       const Ellipse3& e)
{
  return e.distance(p);
}


inline Scalar distance(const Ellipse3& e,
                       const Point3&   p)
{
  return e.distance(p);
}

} // mt

#endif // MT_ELLIPSE3_H

