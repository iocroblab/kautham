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
#ifndef MT_PLANE3_H
#define MT_PLANE3_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/direction_type.h>
#include <mt/line3.h>
#include <mt/exception.h>
#include <mt/point3.h>
#include <mt/scalar.h>
#include <mt/unit3.h>
#include <mt/vector3.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup elements

/// \ingroup elements
/// \brief Three-dimensional plane class.
///
/// Planes are represented using the Hessian normal form
///
/// \f[ \hat{\mathbf{n}} \mathbf{x} + d = 0 \f]
///
/// where
///
/// - \f$ \hat{\mathbf{n}} \f$ is the plane normal unit vector
///
/// - \f$ d \f$ is the \e signed distance from the plane to the coordinate
/// system origin. The sign of this distance is determined by the direction of
/// the plane normal \f$ \hat{\mathbf{n}} \f$
///
/// - \f$ x \f$ is the cartesian representation of a point contained in the
/// plane
///
/// Additionally, a plane can be considered to be directed or undirected.
/// If two \e directed planes differ only in that their normal vectors are
/// antiparallel, they are considered different.
/// If at least one of the planes is \e undirected, then they are considered to
/// be equal. If left unspecified in the constructor, all planes are considered
/// \a directed except for planes constructed from three support points (see
/// example below)
///
/// After a plane is constructed, it is tested to ensure it was well defined.
/// For example, after constructing a plane with a normal vector and support
/// point inputs, the support point is tested to ensure it is contained in the
/// plane. Another case that can yield an ill-defined situation is a plane
/// constructed with three almost-collinear points.
///
/// This is an example of how to use the Plane3 class:
///
/// \code
/// using namespace mt;
///
/// // Some necessary geometric elements
/// Scalar d = -2.0;
/// Unit3  nor(1.0, 0.0, 0.0);
/// Point3 p1(2.0, 0.0, 0.0);
/// Point3 p2(2.0, 1.0, 0.0);
/// Point3 p3(2.0, 0.0, 1.0);
/// Point3 p4(2.0, 0.0, 2.0);
///
/// // Plane constructors
/// Plane3 P1;
/// Plane3 P2(nor, d);
/// Plane3 P3(nor, p1, Plane3::DIRECTED);
/// Plane3 P4 = -P2;
/// Plane3 P5(-nor, d, Plane3::UNDIRECTED);
/// Plane3 P6(p1, p2, p3);
/// Plane3 P7(p1, p3, p4); // oops!
///
/// // Planes P2 and P3 are exactly the same, although they were constructed
/// // with different input values. For example, cout << P2 will produce:
/// // normal: [1 0 0] dist. to origin: -2 (directed)
///
/// bool test;
/// test = (P2 == P3); // test = true
///
/// // Plane P4 differs from planes P2-P3 in that its normal vector is inverted.
/// // Since these planes are directed, they are not considered equal.
///
/// test = (P2 == P4); // test = false
/// test = (P3 == P4); // test = false
///
/// // Like P4, plane P5 has an inverted normal vector but it is also
/// // undirected, so equality tests with the above planes will return true.
///
/// test = (P2 == P5); // test = true
/// test = (P4 == P5); // test = true
///
/// // The normal chosen for plane P6 depends on the values of the input
/// // support points. For this reason, the constructor for three points sets
/// // the direction type to \a undirected by default.
/// // Plane P7 cannot be constructed since the supplied support points are
/// // collinear, and a mt::BadPlane exception is thrown.
/// \endcode

class Plane3
{
public:

/// Ill-defined plane exception.
class BadPlane : public mt::Exception
{
public:
  BadPlane() : Exception(
  "Ill-defined plane. Input support point is not contained in the \
constructed plane") {}
};


// LIFECYCLE

  /// Default constructor. Creates a directed plane coincident with the
  /// xy plane.
  Plane3();

  /// Constructor for unit normal vector and distance to the origin input.
  Plane3(const Unit3&         norm,
         const Scalar&        dist,
         const DirectionType& type = DIRECTED);

  /// Constructor for non-unit normal vector and distance to the origin
  /// input.
  Plane3(const Vector3&       norm,
         const Scalar&        dist,
         const DirectionType& type = DIRECTED);

  /// Constructor for unit normal vector and support point input.
  Plane3(const Unit3&         norm,
         const Point3&        sup,
         const DirectionType& type = DIRECTED);

  /// Constructor for non-unit normal vector and support point input.
  Plane3(const Vector3&       norm,
         const Point3&        sup,
         const DirectionType& type = DIRECTED);

  /// Constructor for three support points.
  Plane3(const Point3&        sup1,
         const Point3&        sup2,
         const Point3&        sup3,
         const DirectionType& type = UNDIRECTED);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Plane3& P) const;
  bool operator!=(const Plane3& P) const;


// OPERATIONS

  /// Projects point \a p on current plane.
  Point3 project(const Point3& p) const;

  /// \e Unsigned distance from point \a p to current plane.
  Scalar distance(const Point3& p) const;


// ACCESS

  /// Gets plane normal unit vector.
  Unit3 getNormal() const;
  Unit3& getNormalRef();
  const Unit3& getNormalRef() const;

  /// Gets \e signed distance from the plane to the coordinate system origin.
  /// The sign of this distance is determined by the direction of the plane
  /// normal \f$ \hat{\mathbf{n}} \f$
  Scalar getDistOrig() const;
  Scalar& getDistOrigRef();
  const Scalar& getDistOrigRef() const;

  /// Gets plane support point that is closest to the coordinate system
  /// origin.
  Point3 getSupport() const;

  /// Gets plane direction type.
  DirectionType getDirectionType() const;
  DirectionType& getDirectionTypeRef();
  const DirectionType& getDirectionTypeRef() const;

  /// Gets a unit vector perpendicular to the plane normal.
  ///
  /// If the coordinate system axis closest to the plane normal is \e x, the
  /// return vector will point in the direction of the \e y axis projected on
  /// the plane. Otherwise it will point in the direction of the \e y axis
  /// projected on the plane.
  Unit3 getSupportVector() const;

  /// Returns a plane parallel to the current plane separated by \a d.
  Plane3 getParallelPlane(const Scalar& d) const;

  /// Sets plane normal unit vector.
  void setNormal(const Unit3& norm);

  /// Sets \e signed distance from the plane to the coordinate system origin.
  void setDistOrig(const Scalar& dist);

  /// Sets plane support point.
  void setSupport(const Point3& sup);

  /// Sets plane direction type.
  void setDirectionType(const DirectionType& type);

  /// Sets plane with normal, distance to origin, and direction type.
  void setValue(const Unit3&         norm,
                const Scalar&        dist,
                const DirectionType& type = DIRECTED);

  /// Sets plane with normal, support point, and direction type.
  void setValue(const Unit3&         norm,
                const Point3&        sup,
                const DirectionType& type = DIRECTED);

   /// Sets plane with three support points.
  void setValue(const Point3&        sup1,
                const Point3&        sup2,
                const Point3&        sup3,
                const DirectionType& type = DIRECTED);

private:

// MEMBERS
Unit3         m_normal;
Scalar        m_dist_orig;
DirectionType m_dir_type;

// METHODS
void setNormal  (const Point3& sup1,
                 const Point3& sup2,
                 const Point3& sup3);

void setSupport(const Point3& sup1,
                const Point3& sup2,
                const Point3& sup3);

};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

Plane3 operator+(const Plane3& P);
Plane3 operator-(const Plane3& P);

std::ostream& operator<<(std::ostream& os,
                         const Plane3& P);


// FUNCTIONS

/// Projects point \a p on plane \a P.
Point3 project(const Point3& p,
               const Plane3& P);

/// Distance from point \a p to plane \a P.
Scalar distance(const Point3& p,
                const Plane3& P);

/// Distance from point \a p to plane \a P.
Scalar distance(const Line3&  L,
                const Plane3& P);


/////////////////////////////// INLINE METHODS ///////////////////////////////

inline Plane3::Plane3() : m_normal(),
                          m_dist_orig(0.0),
                          m_dir_type(DIRECTED) {}


inline Plane3::Plane3(const Unit3&         norm,
                      const Scalar&        dist,
                      const DirectionType& type) :

                      m_normal(norm),
                      m_dist_orig(dist),
                      m_dir_type(type) {}


inline Plane3::Plane3(const Vector3&       norm,
                      const Scalar&        dist,
                      const DirectionType& type) :

                      m_normal(norm),
                      m_dist_orig(dist),
                      m_dir_type(type) {}


inline Plane3::Plane3(const Unit3&         norm,
                      const Point3&        sup,
                      const DirectionType& type) :

                      m_normal(norm),
                      m_dir_type(type)
{
  setSupport(sup);

  // Verifies that input point is contained in plane
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup) == 0.0, BadPlane());
  #endif
}


inline Plane3::Plane3(const Vector3&       norm,
                      const Point3&        sup,
                      const DirectionType& type) :

                      m_normal(norm),
                      m_dir_type(type)
{
  setSupport(sup);

  // Verifies that input point is contained in plane
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup) == 0.0, BadPlane());
  #endif
}


inline Plane3::Plane3(const Point3&        sup1,
                      const Point3&        sup2,
                      const Point3&        sup3,
                      const DirectionType& type)
{
  setValue(sup1, sup2, sup3, type);
}


// OPERATORS

inline bool Plane3::operator==(const Plane3& P) const
{
  bool equal;

  // Tests normal vector equality
  bool same_normal = (m_normal == P.getNormal());

  // Tests distance to origin equality
  bool same_dist_orig = (m_dist_orig == P.getDistOrig());

  if (same_normal && same_dist_orig)
  {
    // Planes are equal regardless of their direction type
    equal = true;
  }
  else
  {
    // Tests if both planes are directed
    const bool both_directed =
    (m_dir_type == DIRECTED) && (P.getDirectionType() == DIRECTED);

    if (!both_directed)
    {
      // At least one of the planes is undirected
      same_normal =    (m_normal == -P.getNormal());
      same_dist_orig = (m_dist_orig == -P.getDistOrig());
      equal = (same_normal && same_dist_orig);
    }
  }
  return equal;
}


inline bool Plane3::operator!=(const Plane3& P) const
{
  return !(*this == P);
}


// OPERATIONS

inline Point3 Plane3::project(const Point3& p) const
{
  return Point3(p - (dot(p, m_normal) + m_dist_orig) * m_normal);
}


inline Scalar Plane3::distance(const Point3& p) const
{
  return abs((dot(p, m_normal) + m_dist_orig));
}


// ACCESS

inline Unit3 Plane3::getNormal() const
{
  return m_normal;
}


inline Unit3& Plane3::getNormalRef()
{
  return m_normal;
}


inline const Unit3& Plane3::getNormalRef() const
{
  return m_normal;
}


inline Scalar Plane3::getDistOrig() const
{
  return m_dist_orig;
}


inline Scalar& Plane3::getDistOrigRef()
{
  return m_dist_orig;
}


inline const Scalar& Plane3::getDistOrigRef() const
{
  return m_dist_orig;
}


inline Point3 Plane3::getSupport() const
{
  return -(m_dist_orig * m_normal);
}


inline DirectionType Plane3::getDirectionType() const
{
  return m_dir_type;
}


inline DirectionType& Plane3::getDirectionTypeRef()
{
  return m_dir_type;
}


inline const DirectionType& Plane3::getDirectionTypeRef() const
{
  return m_dir_type;
}


inline Unit3 Plane3::getSupportVector() const
{
  Point3 ref;
  ref = (m_normal.closestAxis() == 0) ?
        Point3(0.0, 1.0, 0.0) : Point3(1.0, 0.0, 0.0);

  const Point3 sup1(getSupport());
  const Point3 sup2(project(sup1 + ref));

  const Unit3 u(sup2 - sup1);
  return u;
}


inline Plane3 Plane3::getParallelPlane(const Scalar& d) const
{
  const Scalar dnew = m_dist_orig - d;
  return Plane3(m_normal, dnew, m_dir_type);
}


inline void Plane3::setNormal(const Unit3& norm)
{
  m_normal = norm;
}


inline void Plane3::setDistOrig(const Scalar& dist)
{
  m_dist_orig = dist;
}


inline void Plane3::setSupport(const Point3& sup)
{
  m_dist_orig = -dot(m_normal, sup);
}


inline void Plane3::setDirectionType(const DirectionType& type)
{
  m_dir_type = type;
}


inline void Plane3::setValue(const Unit3&         norm,
                             const Scalar&        dist,
                             const DirectionType& type)
{
  setNormal(norm);
  setDistOrig(dist);
  setDirectionType(type);
}


inline void Plane3::setValue(const Unit3&         norm,
                             const Point3&        sup,
                             const DirectionType& type)
{
  setNormal(norm);
  setSupport(sup);
  setDirectionType(type);
}


inline void Plane3::setValue(const Point3&        sup1,
                             const Point3&        sup2,
                             const Point3&        sup3,
                             const DirectionType& type)
{
  // Sets normal
  setNormal(sup1, sup2, sup3);

  // Sets support point
  setSupport(sup1, sup2, sup3);

  // Sets direction type
  setDirectionType(type);
}


// PROTECTED METHODS

inline void Plane3::setNormal(const Point3& sup1,
                              const Point3& sup2,
                              const Point3& sup3)
{
  // Tests that input points are not collinear
  const Unit3 dir[3] = {(sup2 - sup1),
                        (sup3 - sup2),
                        (sup1 - sup3)};

  const Vector3 cos_vec(abs(angleCos(dir[0], dir[1])),
                        abs(angleCos(dir[1], dir[2])),
                        abs(angleCos(dir[2], dir[0])));

  //const Scalar max_cos = cos_vec.max();

  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(max_cos != 1.0,
                Exception("Ill-defined plane. Input points are collinear"));
  #endif

  // Sets normal vector  with support unit vectors that lie at an angle with
  // value closest to pi/2
  const size_t min_cos_idx = cos_vec.minAxis();
  const size_t t1 = ( min_cos_idx      % 3) + 1;
  const size_t t2 = ((min_cos_idx + 1) % 3) + 1;
  m_normal = cross(dir[t1], dir[t2]);
}


inline void Plane3::setSupport(const Point3& sup1,
                               const Point3& sup2,
                               const Point3& sup3)
{
  // Sets point closest to the origin as support point
  const Point3 p[3] = {sup1,
                       sup2,
                       sup3};

  const Vector3 len_p (p[0].length2(),
                       p[1].length2(),
                       p[2].length2());

  const size_t t = len_p.minAxis();
  setSupport(p[t]);

  // Verifies that input points are contained in plane
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup1) == 0.0, BadPlane());
    util::Assert(distance(sup2) == 0.0, BadPlane());
    util::Assert(distance(sup3) == 0.0, BadPlane());
  #endif
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Plane3 operator+(const Plane3& P)
{
  return P;
}


inline Plane3 operator-(const Plane3& P)
{
  return Plane3(-P.getNormal(),
                 P.getDistOrig(),
                 P.getDirectionType());
}


inline std::ostream& operator<<(std::ostream& os,
                                const Plane3& P)
{
  return os << "norm: " << P.getNormal()     << ' '
            << "dist: " << P.getDistOrig()   << " - "
                        << P.getDirectionType();
}


// FUNCTIONS

inline Point3 project(const Point3& p,
                      const Plane3& P)
{
 return P.project(p);
}


inline Scalar distance(const Point3& p,
                       const Plane3& P)
{
  return P.distance(p);
}


inline Scalar distance(const Plane3& P,
                       const Point3& p)
{
  return P.distance(p);
}


} // mt

#endif // MT_PLANE3_H
