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
#ifndef MT_TRANSFORM_H
#define MT_TRANSFORM_H

// C++ STANDARD HEADERS
#include <iostream>

// MT LIBRARY HEADERS
#include <mt/matrix3x3.h>
#include <mt/rotation.h>
#include <mt/scalar.h>
#include <mt/unit3.h>
#include <mt/point3.h>

/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief Rigid transformation class.
///
/// The Transform class provides the usual operators and functions used for
/// expressing and manipulating rigid transformations.
///
/// This is an example of how to use the Transform class:
///
/// \code
/// using namespace mt;
///
/// // Constructor
/// const Unit3 axis(1.0, 0.0, 0.0);
/// const Scalar angle = HALF_PI;
/// const Rotation rot(axis, angle);  // Rotational part
///
/// const Point3 tr(2.0, -1.0, 1.0); // Translational part
///
/// const Transform T1(rot, tr);
/// const Transform T2 = T1 * T1;
///
/// // Applying a transform to a vector
/// const Point3 p_initial(0.0, 0.0, 1.0);
/// Point3 p_final;
///
/// p_final = T1(p_initial);           // p_final = [2.0, -2.0,  1.0]
/// p_final = T2(p_initial);           // p_final = [4.0, -2.0, -1.0]
/// p_final = (T1 * T1)(p_initial);    // Same as above
/// p_final = T2 * p_initial;          // Same as above
/// p_final = T1 * T1 * p_initial;     // Same as above
/// \endcode

class Transform
{
public:

// LIFECYCLE

  /// Default constructor. Creates identity transform.
  Transform();

  /// Constructor for rotation and translation input.
  Transform(const Rotation& rotation,
            const Point3&   translation);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  /// Obtains the composition of the current and input transforms.
  Transform& operator*=(const Transform& t);

  /// Applies transform to input point.
  Point3 operator()(const Point3& p) const;

  bool operator==(const Transform& t) const;

  bool operator!=(const Transform& t) const;


// OPERATIONS

  /// Inverse transform.
  Transform inverse() const;


// ACCESS

  /// Gets translational component of transform.
  Point3 getTranslation() const;
  Point3& getTranslationRef();
  const Point3& getTranslationRef() const;

  /// Gets rotational component of transform.
  Rotation getRotation() const;
  Rotation& getRotationRef();
  const Rotation& getRotationRef() const;

  /// Sets translational component of transform.
  void setTranslation(const Point3& translation);

  /// Sets rotational component of transform.
  void setRotation(const Rotation& rotation);

  /// Sets identity transform.
  void setIdentity();


private:

// MEMBERS

Rotation m_rotation;
Point3   m_translation;

};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

/// Applies transform to input point.
Point3 operator*(const Transform& t,
                 const Point3&    p);

/// Obtains the composition of the current and input transforms.
Transform operator*(const Transform& t1,
                    const Transform& t2);

/// Outputs the transform.
std::ostream& operator<<(std::ostream&    os,
                         const Transform& t);


// FUNCTIONS

/// Interpolation/extrapolation between input transforms.
///
/// The translational and rotational components of the resulting transform
/// are obtained through linear and spherical-linear
/// interpolation/extapolation, respectively.

Transform interpolate(const Transform& t1,
                      const Transform& t2,
                      const Scalar&    s);


/// Transform inverse.
Transform inverse(const Transform& t);

/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Transform::Transform()
{
  setIdentity();
}


inline Transform::Transform(const Rotation& rotation,
                            const Point3&   translation) :

                            m_rotation(rotation),
                            m_translation(translation) {}


// OPERATORS

inline Transform& Transform::operator*=(const Transform& t)
{
  m_translation += (m_rotation(t.m_translation));
  m_rotation    *= t.m_rotation;
  return *this;
}


inline Point3 Transform::operator()(const Point3& p) const
{
  return Point3(m_rotation(p) + m_translation);
}


inline bool Transform::operator==(const Transform& t) const
{
  return (m_rotation    == t.getRotation() &&
          m_translation == t.getTranslation());
}


inline bool Transform::operator!=(const Transform& t) const
{
  return !(*this == t);
}


// OPERATIONS

inline Transform Transform::inverse() const
{
  const Rotation inv(m_rotation.inverse());

  return Transform(inv, inv(-m_translation));
}


// ACCESS


inline Point3 Transform::getTranslation() const
{
  return m_translation;
}


inline Point3& Transform::getTranslationRef()
{
  return m_translation;
}


inline const Point3& Transform::getTranslationRef() const
{
  return m_translation;
}


inline Rotation Transform::getRotation() const
{
  return m_rotation;
}


inline Rotation& Transform::getRotationRef()
{
  return m_rotation;
}


inline const Rotation& Transform::getRotationRef() const
{
  return m_rotation;
}


inline void Transform::setTranslation(const Point3& translation)
{
  m_translation = translation;
}


inline void Transform::setRotation(const Rotation& rotation)
{
  m_rotation = rotation;
}


inline void Transform::setIdentity()
{
  m_rotation.setValue(Scalar(0.0), Scalar(0.0), Scalar(0.0), Scalar(1.0));
  m_translation.setValue(Scalar(0.0), Scalar(0.0), Scalar(0.0));
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Point3 operator*(const Transform& t,
                        const Point3&    p)
{
  return t(p);
}


inline Transform operator*(const Transform& t1,
                           const Transform& t2)
{
  const Rotation rotation    (t1.getRotation() * t2.getRotation());
  const Point3   translation (t1 * t2.getTranslation());

  return Transform(rotation, translation);
}


inline std::ostream& operator<<(std::ostream&    os,
                                const Transform& t)
{
  return os << "Rotation:    " << t.getRotation() << "\n"
            << "Translation: " << t.getTranslation();
}


// FUNCTIONS

inline Transform interpolate(const Transform& t1,
                             const Transform& t2,
                             const Scalar&    s)
{
  const Point3 p = lerp(t1.getTranslation(),
                        t2.getTranslation(),
                        s);

  const Rotation r = slerp(t1.getRotation(),
                           t2.getRotation(),
                           s);

  return Transform(r, p);
}


inline Transform inverse(const Transform& t)
{
  return t.inverse();
}

}// mt

#endif // MT_TRANSFORM_H
