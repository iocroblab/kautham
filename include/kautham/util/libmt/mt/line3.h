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
#ifndef MT_LINE3_H
#define MT_LINE3_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/direction_type.h>
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
///
/// \brief Three-dimensional line class.
///
/// Lines are represented using the parametric line equation
///
/// \f[ \mathbf{x} = \mathbf{p} + \hat{\mathbf{d}}t \f]
///
/// where
///
/// - \f$ \mathbf{p} \f$ is the line support point
///
/// - \f$ \hat{\mathbf{d}} \f$ is the line direction unit vector
///
/// - \f$ t \f$ is the independent parameter
///
/// - \f$ x \f$ is the cartesian representation of a point contained in the line
///
/// A given line can be represented in infinitely different ways depending
/// on the selected support point. Since the interest is to have a \e unique
/// line representation regardless of the parameters it was constructed with,
/// a \e normalized representation of the parametric line equation is used,
/// in which the support point is the line point closest to the current
/// coordinate system.
/// Keep this in mind because when querying for the line support point you
/// will be getting the \e normalized point instead of the value supplied in
/// the constructor (unless they coincide).
///
/// Additionally, a line can be considered to be directed or undirected.
/// If two \e directed lines differ only in that their direction vectors are
/// antiparallel, they are considered different.
/// If at least one of the lines is \e undirected, then they are considered to
/// be equal. If left unspecified in the constructor, all lines are considered
/// \a directed.
///
/// After a line is constructed, it is tested to ensure it was well defined.
/// For example, after constructing a line with a direction vector and support
/// point inputs, the support point is tested to ensure it is contained in the
/// line. Another case that can yield an ill-defined situation is a line
/// constructed with two almost-coincident points.
///
/// This is an example of how to use the Line3 class:
///
/// \code
/// using namespace mt;
///
/// // Some necessary geometric elements
/// Unit3  dir(1.0, 0.0, 0.0);
/// Point3 p1(0.0, 2.0, 0.0);
/// Point3 p2(1.0, 2.0, 0.0);
///
/// // Line constructors
/// Line3 L1;
/// Line3 L2( dir, p1);
/// Line3 L3( dir, p2);
/// Line3 L4(  p1, p2, Line3::DIRECTED);
/// Line3 L5 = -L2;
/// Line3 L6(-dir, p2, Line3::UNDIRECTED);
///
/// // Lines L2, L3, and L4 are exactly the same, although they were constructed
/// // with different input values. For example, cout << L2 will produce:
/// // direction: [1 0 0] support: [0 2 0] (directed)
///
/// bool test;
/// test = (L2 == L3); // test = true
/// test = (L3 == L4); // test = true
///
/// // Line L5 differs from lines L2-L4 in that its direction vector is inverted.
/// // Since these lines are directed, they are not considered equal.
///
/// test = (L2 == L5); // test = false
/// test = (L4 == L5); // test = false
///
/// // Like L5, line L6 has an inverted direction vector but it is also
/// // undirected, so equality tests with the above lines will return true.
/// // cout << L6 will produce:
/// // direction: [-1 0 0] support: [0 2 0] (undirected)
///
/// test = (L2 == L6); // test = true
/// test = (L5 == L6); // test = true
/// \endcode

class Line3
{
public:

/// Ill-defined line exception.
class BadLine : public Exception
{
public:
  BadLine() : Exception(
  "Ill-defined line. Input support point is not contained in the constructed \
line") {}
};


// LIFECYCLE

  /// Default constructor. Creates a directed line coincident with the
  /// \e z axis.
  Line3();


  /// Constructor for unit direction vector and support point input.
  Line3(const Unit3&         dir,
        const Point3&        sup,
        const DirectionType& type = DIRECTED);

  /// Constructor for non-unit direction vector and support point input.
  Line3(const Vector3&       dir,
        const Point3&        sup,
        const DirectionType& type = DIRECTED);

  /// Constructor for two support point input.
  Line3(const Point3& sup1,
        const Point3& sup2,
        const DirectionType& type = DIRECTED);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Line3& L) const;
  bool operator!=(const Line3& L) const;

// OPERATIONS

  /// Projects point \a p on current line.
  Point3 project(const Point3& p) const;

  /// Distance from point \a p to current line.
  Scalar distance(const Point3& p) const;


// ACCESS

  /// Gets line direction unit vector.
  Unit3 getDirection() const;
  Unit3& getDirectionRef();
  const Unit3& getDirectionRef() const;

  /// Gets \e normalized line support point. Refer to the class description
  /// for the meaning of \e normalized.
  Point3 getSupport() const;
  Point3& getSupportRef();
  const Point3& getSupportRef() const;

  /// Gets line direction type.
  DirectionType getDirectionType() const;
  DirectionType& getDirectionTypeRef();
  const DirectionType& getDirectionTypeRef() const;

  /// Gets the cartesian point associated to the line parameter \a t
  /// according to \f$ \mathbf{x} = \mathbf{p} + \hat{\mathbf{d}}t \f$.
  Point3 getPoint(const Scalar& t) const;

  /// Gets line parameter associated to a cartesian point.
  Scalar getParameter(const Point3& p) const;

  /// Sets line direction unit vector.
  void setDirection(const Unit3& dir);

  /// Sets line support point.
  void setSupport(const Point3& sup);

  /// Sets line direction type.
  void setDirectionType(const DirectionType& type);

  /// Sets line with direction vector, support point, and direction type.
  void setValue(const Unit3&         dir,
                const Point3&        sup,
                const DirectionType& type = DIRECTED);


private:

// MEMBERS
Unit3         m_direction;
Point3        m_support;
DirectionType m_dir_type;

};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

Line3 operator+(const Line3& L);
Line3 operator-(const Line3& L);

std::ostream& operator<<(std::ostream& os,
                         const Line3& L);


// FUNCTIONS

/// Projects point \a p on line \a L.
Point3 project(const Point3& p,
               const Line3&  L);

/// Distance from point \a p to line \a L.
Scalar distance(const Point3& p,
                const Line3&  L);

/// Distance from point \a p to line \a L.
Scalar distance(const Line3&  L,
                const Point3& p);



/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Line3::Line3() : m_direction(),
                        m_support(),
                        m_dir_type(DIRECTED) {}


inline Line3::Line3(const Unit3&         dir,
                    const Point3&        sup,
                    const DirectionType& type) :

                    m_direction(dir),
                    m_dir_type(type)
{
  setSupport(sup);

  // Verifies that input point is contained in line
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup) == 0.0, BadLine());
  #endif
}


inline Line3::Line3(const Vector3&       dir,
                    const Point3&        sup,
                    const DirectionType& type) :

                    m_direction(dir),
                    m_dir_type(type)
{
  setSupport(sup);

  // Verifies that input point is contained in line
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup) == 0.0, BadLine());
  #endif
}


inline Line3::Line3(const Point3&        sup1,
                    const Point3&        sup2,
                    const DirectionType& type) :

                    m_dir_type(type)
{
  // Verifies that the input points are not coincident
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(mt::distance(sup1, sup2) != 0.0,
                Exception("Cannot define line with two coincident points."));
  #endif

  // Point that is closest to the origin of the current coordinate system
  const Point3 p_closest = (sup1.length() < sup2.length()) ? sup1 : sup2;

  // Sets remaining line parameters
  setDirection(sup2 - sup1);
  setSupport(p_closest);

  // Verifies that input points are contained in line
  #ifdef MT_USE_BASIC_SCALAR
    util::Assert(distance(sup1) == 0.0, BadLine());
    util::Assert(distance(sup2) == 0.0, BadLine());
  #endif
}

// OPERATORS

inline bool Line3::operator==(const Line3& L) const
{
  // Tests direction vector equality
  bool same_direction = (m_direction == L.getDirection());

  // Tests support point equality
  const bool same_support = (m_support == L.getSupport());

  // Tests if both lines are directed
  const bool both_directed =
  (m_dir_type == DIRECTED) && (L.getDirectionType() == DIRECTED);

  if (!both_directed)
  {
    // At least one of the lines is undirected
    const bool opposite_direction = (m_direction == -L.getDirection());
    same_direction |= opposite_direction;
  }
  return (same_support && same_direction);
}

inline bool Line3::operator!=(const Line3& L) const
{
  return !(*this == L);
}


// ACCESS


inline Unit3 Line3::getDirection() const
{
  return m_direction;
}


inline Unit3& Line3::getDirectionRef()
{
  return m_direction;
}


inline const Unit3& Line3::getDirectionRef() const
{
  return m_direction;
}


inline Point3 Line3::getSupport() const
{
  return m_support;
}


inline Point3& Line3::getSupportRef()
{
  return m_support;
}


inline const Point3& Line3::getSupportRef() const
{
  return m_support;
}


inline DirectionType Line3::getDirectionType() const
{
  return m_dir_type;
}


inline DirectionType& Line3::getDirectionTypeRef()
{
  return m_dir_type;
}


inline const DirectionType& Line3::getDirectionTypeRef() const
{
  return m_dir_type;
}


inline Point3 Line3::getPoint(const Scalar& t) const
{
  return Point3(m_support + t * m_direction);
}


inline Scalar Line3::getParameter(const Point3& p) const
{
  // Component of direction vector with highest absolute value
  const size_t i = m_direction.closestAxis();

  // Parameter value
  return (p[i] - m_support[i]) / m_direction[i];
}


inline void Line3::setDirection(const Unit3& dir)
{
  m_direction = dir;
}


inline void Line3::setSupport(const Point3& sup)
{
  const Point3 origin(0.0, 0.0, 0.0);
  m_support = sup;
  m_support = project(origin);
}


inline void Line3::setDirectionType(const DirectionType& type)
{
  m_dir_type = type;
}


inline void Line3::setValue(const Unit3&         dir,
                            const Point3&        sup,
                            const DirectionType& type)
{
  setDirection(dir);
  setSupport(sup);
  setDirectionType(type);
}

// OPERATIONS

inline Point3 Line3::project(const Point3& p) const
{
  const Point3 diff(p - m_support);
  const Scalar t = dot(diff, m_direction);
  return getPoint(t);
}


inline Scalar Line3::distance(const Point3& p) const
{
  const Vector3 diff(p - project(p));
  return length(diff);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Line3 operator+(const Line3& L)
{
  return L;
}


inline Line3 operator-(const Line3& L)
{
  return Line3(-L.getDirection(),
                L.getSupport(),
                L.getDirectionType());
}


inline std::ostream& operator<<(std::ostream& os,
                                const Line3&   L)
{
  return os << "dir: " << L.getDirection() << ' '
            << "sup: " << L.getSupport()   << " - "
                       << L.getDirectionType();

}

// FUNCTIONS

inline Point3 project(const Point3& p,
                      const Line3&  L)
{
  return L.project(p);
}


inline Scalar distance(const Point3& p,
                       const Line3&  L)
{
  return L.distance(p);
}


inline Scalar distance(const Line3&  L,
                       const Point3& p)
{
  return L.distance(p);
}


} // mt

#endif // MT_LINE3_H
