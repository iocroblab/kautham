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
#ifndef MT_ROTATION_H
#define MT_ROTATION_H

// MT LIBRARY HEADERS
#include <mt/exception.h>
#include <mt/unit3.h>
#include <mt/matrix3x3.h>
#include <mt/plane3.h>
#include <mt/quaternion.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief Singularity-free spatial rotations class.
///
/// The Rotation class permits defining 3D rotations from quaternion, rotation
/// matrix, axis-angle, and yaw, pitch, and roll representations.
/// Also, any of the above representations can be extracted from an existing
/// Rotation instance.
///
/// Rotations can be applied to a vector using the () operator, and can be
/// combined by means of the product operator.
///
/// The Rotation class is implemented using unit quaternions, so it provides the
/// usual operators and functions used for expressing and manipulating
/// quaternions, including spherical linear interpolation/extrapolation (SLERP).
///
/// The main difference from Quaternion, its base class, is that the quaternion
/// length is \e always guaranteed to be unity.
///
/// Let \f$ \mathbf{q} \f$ be a unit quaternion, then the following notations
/// are analogous:
/// \f$ \mathbf{q} = [ x, y, z, w ] = xi + yj + zk + w \f$
///
/// with \f$ \Vert \mathbf{q} \Vert = 1 \f$.
///
/// This is an example of how to use the Rotation class:
///
/// \code
/// using namespace mt;
///
/// // Constructors
/// Rotation R1(0.0, 0.0, 0.0, 1.0);      // Quaternion input
///
/// const Unit3 axis(1.0, 0.0, 0.0);
/// const Scalar angle = HALF_PI;
/// Rotation R2(axis, angle);             // Axis-angle input
///
/// Scalar yaw   = -PI / Scalar(3.0);     // Angle around fixed z
/// Scalar pitch = 0.0;                   // Angle around fixed y
/// Scalar roll  = -TWO_PI;               // Angle around fixed x
/// Rotation R3(yaw, pitch, roll);        // Yaw, pitch, and roll constructor
///
/// // Accesors
/// const Matrix3x3 M(R1.getMatrix());    // M is the identity 3x3 matrix
/// R3.setAxisAngle(axis, angle);         // R3 and R2 are now equivalent
///
/// // Applying a rotation
/// R3 *= R2;                             // R3 = rotation of PI about the x axis
/// Vector3 position(0.0, 0.0, 1.0);
/// position = R3(position);              // position = [0.0, 0.0, -1.0]
///
/// // Interpolating rotations
/// R1 = slerp(R1, R2, 0.5);              // R1 = rotation of PI/4 about the x axis
/// \endcode

class Rotation : public Quaternion
{
public:

// LIFECYCLE

  /// Default constructor. Creates the [0.0, 0.0, 0.0, 1.0] quaternion.
  Rotation();

  /// Constructor for four input values.
  ///
  /// The input values need not define a quaternion of unit length.
  Rotation(const Scalar& x,
           const Scalar& y,
           const Scalar& z,
           const Scalar& w);

  /// Constructor for pointer input.
  ///
  /// The input values need not define a quaternion of unit length.
  explicit Rotation(const Scalar* r);

  /// Constructor for axis-angle input.
  Rotation(const Unit3&   axis,
           const Scalar&  angle);

  /// Constructor for yaw, pitch, and roll angle input.
  /// Rotations angles are measured about the \e fixed reference frame.
  ///
  /// \param yaw %Rotation around the \e z axis
  /// \param pitch %Rotation around the \e y axis
  /// \param roll %Rotation around the \e x axis
  Rotation(const Scalar& yaw,
           const Scalar& pitch,
           const Scalar& roll);

  /// Constructor for rotation matrix.
  Rotation(const Matrix3x3& mat);

  /// Constructor for two pairs of independent unit vectors.
  Rotation(const Unit3& u,
           const Unit3& v,
           const Unit3& r,
           const Unit3& s);

  /// Constructor for two unit vectors (obtains rotation that transforms \a u
  /// into \a v ).
  Rotation(const Unit3& u,
           const Unit3& v);

  /// Constructor for Quaternion input.
  Rotation(const Quaternion& q);

  // Compiler generated copy constructor for Rotation input is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator for Rotation input is being used

  /// Assignment operator.
  /// @param q The Rotation to assign to this object.
  /// @return A reference to this object.
  Rotation& operator=(const Quaternion& q);

  /// Applies rotation to input vector.
  Vector3 operator()(const Vector3& v) const;

  /// Equality operator. The comparison criterion is that the angle between
  /// the two quaternions must be equal to zero.
  bool operator==(const Rotation& r) const;

  bool operator!=(const Rotation& r) const;

// OPERATIONS

  /// Cosine of the angle between unit quatenions.
  Scalar angleCos(const Rotation& r) const;

  /// Angle between unit quatenions in the range \f$ [0, \pi] \f$ expressed in
  /// radians.
  Scalar angle(const Rotation& r) const;

  /// Quaternion inverse.
  Rotation inverse() const;

// ACCESS

  /** Gets axis-angle representation of rotation, where:

  \f[ \alpha = 2 acos(q_w) \f]

  and

  \f{eqnarray*}
    u_x & = & q_x / s \\
    u_y & = & q_y / s \\
    u_z & = & q_z / s
  \f}

  where \f$ s = \sqrt{1 - q_w^2} \f$.

  Of the two possible rotation angles, the one belonging to
  \f$ [0, \pi] \f$ is chosen.
  */
  void getAxisAngle(Unit3&  axis,
                    Scalar& angle) const;

  /** Gets \a yaw, \a pitch, and \a roll angles representation of rotation
  according to:

  \f{eqnarray*}
    pitch & = & asin(2(q_y q_w - q_x q_z)) \\

    yaw   & = & atan2
              \left(
              \frac{2(q_x q_y + q_z q_w)}{q_x^2 - q_y^2 - q_z^2 + q_w^2}
              \right) \\

    roll  & = & atan2
              \left(
              \frac{2(q_x q_w + q_y q_z)}{-q_x^2 - q_y^2 + q_z^2 + q_w^2}
              \right) \\
  \f}

  Angular values belong to the interval \f$ [0, 2\pi) \f$. Since the above
  equation for the \a pitch angle has two solutions in the mentioned
  interval, the solution closest to the initial value of \a pitch is
  chosen.

  Singularities (i.e., gimbal lock) arise when the \a pitch angle equals
  \f$ \pi / 2 \f$ or \f$ 3 \pi / 2 \f$. In such cases the initial value of
  the \a roll angle is used, and the \a yaw angle is calculated as follows:

  If \f$ pitch = \pi / 2 \f$,
  \f[ yaw = roll -atan2(\frac{-r_{23}}{r_{13}}) \f]

  If \f$ pitch = 3 \pi / 2 \f$,
  \f[ yaw = -roll + atan2(\frac{-r_{23}}{-r_{13}}) \f]

  where

  \f{eqnarray*}
    r_{13} & = & s (q_x q_z + q_y q_w) \\
    r_{23} & = & s (q_y q_z + q_x q_w) \\
    s & = & 2 / \Vert \mathbf{q} \Vert ^ 2
  \f}

  Rotations angles are measured about the \e fixed reference frame.
  \param yaw %Rotation around the \e z axis
  \param pitch %Rotation around the \e y axis
  \param roll %Rotation around the \e x axis
  */
  void getYpr(Scalar& yaw,
              Scalar& pitch,
              Scalar& roll) const;

  /** Gets matrix representation of rotation according to:

  \f[
    \left[
    \begin{array}{c c c}
      1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_z q_w) & 2(q_x q_z + q_y q_w) \\
      2(q_x q_y + q_z q_w) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_x q_w) \\
      2(q_x q_z - q_y q_w) & 2(q_y q_z + q_x q_w) & 1 - 2(q_x^2 + q_y^2) \\
    \end{array}
    \right]
  \f]
  */
  Matrix3x3 getMatrix() const;

  /// Sets unit quaternion values.
  void setValue(const Scalar& x,
                const Scalar& y,
                const Scalar& z,
                const Scalar& w);

  /// Sets unit quaternion values from pointer input.
  void setValue(const Scalar* r);

  /** Sets quaternion from axis-angle input according to:

  \f{eqnarray*}
    q_x & = & u_x sin(\alpha / 2) \\
    q_y & = & u_y sin(\alpha / 2) \\
    q_z & = & u_z sin(\alpha / 2) \\
    q_w & = & cos(\alpha / 2)
  \f}

  where \f$ \hat{\mathbf{u}} \f$ represents the rotation axis and
  \f$ \alpha \f$ represents the rotation angle.
  */
  void setAxisAngle(const Unit3&   axis,
                    const Scalar&  angle);

  /** Sets quaternion from \a yaw, \a pitch, and \a roll input according to:

  \f{eqnarray*}
    q_x & = & cos(yaw / 2) cos(pitch / 2) sin(roll / 2) -
              sin(yaw / 2) sin(pitch / 2) cos(roll / 2) \\

    q_y & = & cos(yaw / 2) sin(pitch / 2) cos(roll / 2) +
              sin(yaw / 2) cos(pitch / 2) sin(roll / 2) \\

    q_z & = & sin(yaw / 2) cos(pitch / 2) cos(roll / 2) -
              cos(yaw / 2) sin(pitch / 2) sin(roll / 2) \\

    q_w & = & cos(yaw / 2) cos(pitch / 2) cos(roll / 2) +
              sin(yaw / 2) sin(pitch / 2) sin(roll / 2)
  \f}

  The rotation matrix associated to the \a yaw, \a pitch and \a roll
  convention is:

  \f[
    R = R_y R_p R_r \left[
    \begin{array}{c c c}
      c_y c_p   &   c_y s_p s_r - s_y c_r   &   c_y s_p c_r + s_y s_r \\
      s_y c_p   &   s_y s_p s_r + c_y c_r   &   s_y s_p c_r - c_y s_r \\
         -s_p   &                 c_p s_r   &                 c_p c_r
    \end{array}
    \right]
  \f]

  with \f$ s_\alpha = sin(\alpha) \f$ and \f$ c_\alpha = cos(\alpha) \f$.

  The \a yaw, \a pitch, and \a roll angles represent rotations about the
  \e fixed \e z, \e y, and \e x axes, respectively.

  Other names given to this rotation convention are
  <em> X-Y-Z fixed angles </em> and <em> 3-2-1 Euler Angle Sequence </em>
  */
  void setYpr(const Scalar& yaw,
              const Scalar& pitch,
              const Scalar& roll);

  /** Sets quaternion from rotation matrix.

  Given a rotation matrix

  \f[
    R = \left[
    \begin{array}{c c c}
      r_{11} & r_{12} & r_{13} \\
      r_{21} & r_{22} & r_{23} \\
      r_{31} & r_{32} & r_{33} \\
    \end{array}
    \right]
  \f]

  the associated quaternion \f$ \mathbf{q} \f$ is given by one of the
  following relations:

  - If \f$ (r_{22} \geq -r_{33}) \wedge (r_{11} \geq -r_{22}) \wedge (r_{11} \geq -r_{33}) \f$
  \f[
    \mathbf{q} =
    \left[
    \begin{array}{c}
      (r_{32} - r_{23}) / s \\
      (r_{13} - r_{31}) / s \\
      (r_{21} - r_{12}) / s \\
      1 / (4s)
    \end{array}
    \right]
  \f]
  with \f$ s = 2 \sqrt{1 + r_{11} + r_{22} + r_{33}} \f$

  - If \f$ (r_{22} < -r_{33}) \wedge (r_{11} \geq r_{22}) \wedge (r_{11} \geq r_{33}) \f$
  \f[
    \mathbf{q} =
    \left[
    \begin{array}{c}
      1 / (4s)              \\
      (r_{21} + r_{12}) / s \\
      (r_{13} + r_{31}) / s \\
      (r_{32} - r_{23}) / s
    \end{array}
    \right]
  \f]
  with \f$ s = 2 \sqrt{1 + r_{11} - r_{22} - r_{33}} \f$

  - If \f$ (r_{22} \geq r_{33}) \wedge (r_{11} < r_{22}) \wedge (r_{11} < -r_{33}) \f$
  \f[
    \mathbf{q} =
    \left[
    \begin{array}{c}
      (r_{21} + r_{12}) / s \\
      1 / (4s)              \\
      (r_{32} + r_{23}) / s \\
      (r_{13} - r_{31}) / s
    \end{array}
    \right]
  \f]
  with \f$ s = 2 \sqrt{1 - r_{11} + r_{22} - r_{33}} \f$

  - If \f$ (r_{22} < r_{33}) \wedge (r_{11} < -r_{22}) \wedge (r_{11} < r_{33}) \f$
  \f[
    \mathbf{q} =
    \left[
    \begin{array}{c}
      (r_{13} + r_{31}) / s \\
      (r_{32} + r_{23}) / s \\
      1 / (4s)              \\
      (r_{21} - r_{12}) / s
    \end{array}
    \right]
  \f]
  with \f$ s = 2 \sqrt{1 - r_{11} - r_{22} + r_{33}} \f$

  */
  void setMatrix(const Matrix3x3& mat);

  /** Sets quaternion from two pairs of unit vectors.

  Given two pairs of independent unit vectors
  \f$ \{ \hat\mathbf{u}, \hat\mathbf{v} \} \f$ and
  \f$ \{ \hat\mathbf{r}, \hat\mathbf{s} \} \f$, the rotation
  \f$ \mathrm{R} (\hat\mathbf{u}, \hat\mathbf{v} \rightarrow
  \hat\mathbf{r}, \hat\mathbf{s})\f$
  places \f$ \hat\mathbf{u} \f$ parallel to \f$ \hat\mathbf{r} \f$ and
  \f$ \hat\mathbf{v} \f$ in the plane defined by \f$ \hat\mathbf{r} \f$ and
  \f$ \hat\mathbf{s} \f$.
  If \f$ \hat\mathbf{u} \cdot \hat\mathbf{v} =
  \hat\mathbf{r} \cdot \hat\mathbf{s} \f$,
  then \f$ \hat\mathbf{v} \f$ will additionally be parallel to
  \f$ \hat\mathbf{s} \f$.

  \f[
  \mathrm{R} (\hat\mathbf{u}, \hat\mathbf{v} \rightarrow
  \hat\mathbf{r}, \hat\mathbf{s}) =
  \left[ \begin{array}{c c c}
    \hat\mathbf{r}   &
    \hat\mathbf{s}_r &
    \hat\mathbf{t}
  \end{array} \right]
  \left[ \begin{array}{c c c}
    \hat\mathbf{u}   &
    \hat\mathbf{v}_u &
    \hat\mathbf{w}
  \end{array} \right]^T
  \f]

  where \f$ \{ \hat\mathbf{u}, \hat\mathbf{v}_u, \hat\mathbf{w} \} \f$ and
  \f$ \{ \hat\mathbf{r}, \hat\mathbf{s}_r, \hat\mathbf{t} \} \f$ are the
  orthonormal basis associated to
  \f$ \{ \hat\mathbf{u}, \hat\mathbf{v} \} \f$ and
  \f$ \{ \hat\mathbf{r}, \hat\mathbf{s} \} \f$, respectively.
  */
  void setTwoPairs(const Unit3& u,
                   const Unit3& v,
                   const Unit3& r,
                   const Unit3& s);

  /// Sets quaternion from one pair of unit vectors.
  /// The resulting rotation transforms vector \a u into vector \a v.
  /// This is equivalent to setting the rotation through an axis-angle pair,
  /// where the axis is a vector perpendicular to both \a u and \a v, and the
  /// angle is the angle between \a u and \a v.
  void setOnePair(const Unit3& u,
                  const Unit3& v);
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// FUNCTIONS

/// Cosine of the angle between unit quaternions.
Scalar angleCos(const Rotation& r1,
                const Rotation& r2);

/// Angle between unit quaternions in the range [0, pi] expressed in 
/// radians.
Scalar angle(const Rotation& r1,
             const Rotation& r2);

/// Quaternion inverse.
Rotation inverse(const Rotation& r);


// OPERATORS
std::ostream& operator<<(std::ostream&   os,
                         const Rotation& q);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Rotation::Rotation() : Quaternion() {}


inline Rotation::Rotation(const Scalar& x,
                          const Scalar& y,
                          const Scalar& z,
                          const Scalar& w)
{
  setValue(x, y, z, w);
}


inline Rotation::Rotation(const Scalar* r)
{
  setValue(r);
}


inline Rotation::Rotation(const Unit3&   axis,
                          const Scalar&  angle)
{
  setAxisAngle(axis, angle);
}


inline Rotation::Rotation(const Scalar& yaw,
                          const Scalar& pitch,
                          const Scalar& roll)
{
  setYpr(yaw, pitch, roll);
}


inline Rotation::Rotation(const Matrix3x3& mat)
{
  setMatrix(mat);
}


inline Rotation::Rotation(const Unit3& u,
                          const Unit3& v,
                          const Unit3& r,
                          const Unit3& s)
{
  setTwoPairs(u, v, r, s);
}


inline Rotation::Rotation(const Unit3& u,
                          const Unit3& v)
{
  setOnePair(u, v);
}


inline Rotation::Rotation(const Quaternion& q) : Quaternion(q)
{
  normalize();
}


// OPERATORS

inline Rotation& Rotation::operator=(const Quaternion& q)
{
  if (this != &q)
  {
    // Assignation
    Quaternion::operator=(q);
    normalize();
  }
  return *this;
}


inline Vector3 Rotation::operator()(const Vector3& v) const
{
  const Quaternion q(*this * (v * conjugate()));
  return Vector3(q[0], q[1], q[2]);
}


inline bool Rotation::operator==(const Rotation& r) const
{
  return (angle(r) == 0.0);
}


inline bool Rotation::operator!=(const Rotation& r) const
{
  return !(*this == r);
}


// OPERATIONS

inline Scalar Rotation::angleCos(const Rotation& r) const
{
  Scalar ang_cos(dot(r));
  return ang_cos;
}


inline Scalar Rotation::angle(const Rotation& r) const
{
  const Scalar ang_cos(angleCos(r));
  return acos(ang_cos);
}


inline Rotation Rotation::inverse() const
{
  return conjugate();
}


// ACCESS

inline void Rotation::getAxisAngle(Unit3&  axis,
                                   Scalar& angle) const
{
  angle = Scalar(2.0) * acos(m_co[3]);

  if (abs(m_co[3]) != Scalar(1.0))
  {
    // Angle is nonzero
    const Scalar den(sqrt(Scalar(1.0) - sq(m_co[3])));

    axis.setValue(m_co[0] / den,
                  m_co[1] / den,
                  m_co[2] / den);
  }
  // If the angle equals zero, the current rotation axis is maintained
}


inline void Rotation::getYpr(Scalar& yaw,
                             Scalar& pitch,
                             Scalar& roll) const
{
  const Scalar s_pitch(Scalar(2.0) * (m_co[1] * m_co[3] - m_co[0] * m_co[2]));
  pitch = asin(s_pitch); // belongs to [-pi/2, pi/2]

  if (abs(s_pitch) == Scalar(1.0))
  {
    // Singularity
    const Scalar s(Scalar(2.0) / length2());
    const Scalar r_13(s * (m_co[0] * m_co[2] + m_co[1] * m_co[3]));
    const Scalar r_23(s * (m_co[1] * m_co[2] - m_co[0] * m_co[3]));


    if (s_pitch == Scalar(1.0))
    {
      // Pitch = pi/2
      yaw = roll - atan2(-r_23, r_13);
    }
    else
    {
      // Pitch = -pi/2
      yaw = atan2(-r_23, -r_13) - roll;
    }
  }
  else
  {
    // No singularities
    const Scalar s_yaw (Scalar(2.0) * (m_co[0] * m_co[1] + m_co[2] * m_co[3]));
    const Scalar s_roll(Scalar(2.0) * (m_co[0] * m_co[3] + m_co[1] * m_co[2]));

    const Scalar c_yaw ( sq(m_co[0]) - sq(m_co[1]) - sq(m_co[2]) +
                         sq(m_co[3]));
    const Scalar c_roll(-sq(m_co[0]) - sq(m_co[1]) + sq(m_co[2]) +
                         sq(m_co[3]));

    yaw  = atan2(s_yaw , c_yaw );
    roll = atan2(s_roll, c_roll);
  }

  // Normalizes angles to the interval [0, 2pi)
  // Pitch has been already normalized
  yaw   = mt::normalize(yaw,   Scalar(0.0), TWO_PI);
  pitch = mt::normalize(pitch, Scalar(0.0), TWO_PI);
  roll  = mt::normalize(roll,  Scalar(0.0), TWO_PI);
}


inline Matrix3x3 Rotation::getMatrix() const
{
  const Scalar xx(Scalar(2.0) * m_co[0] * m_co[0]);
  const Scalar yy(Scalar(2.0) * m_co[1] * m_co[1]);
  const Scalar zz(Scalar(2.0) * m_co[2] * m_co[2]);
  // const Scalar ww(2.0 * m_co[3] * m_co[3]);

  const Scalar xy(Scalar(2.0) * m_co[0] * m_co[1]);
  const Scalar xz(Scalar(2.0) * m_co[0] * m_co[2]);
  const Scalar xw(Scalar(2.0) * m_co[0] * m_co[3]);
  const Scalar yz(Scalar(2.0) * m_co[1] * m_co[2]);
  const Scalar yw(Scalar(2.0) * m_co[1] * m_co[3]);
  const Scalar zw(Scalar(2.0) * m_co[2] * m_co[3]);

  return Matrix3x3(Scalar(1.0) - yy - zz,          xy - zw,          xz + yw,
                         xy + zw,    Scalar(1.0) - xx - zz,          yz - xw,
                         xz - yw,          yz + xw,    Scalar(1.0) - xx - yy);
}


inline void Rotation::setValue(const Scalar& x,
                               const Scalar& y,
                               const Scalar& z,
                               const Scalar& w)
{
  Quaternion::setValue(x, y, z, w);
  normalize();
}


inline void Rotation::setValue(const Scalar* r)
{
  Quaternion::setValue(r);
  normalize();
}


inline void Rotation::setAxisAngle(const Unit3&   axis,
                                   const Scalar&  angle)
{
  // Normalizes angle to the interval [0, 2pi)
  const Scalar angle_n (mt::normalize(angle, Scalar(0.0), TWO_PI));

  const Scalar s = sin(angle_n * Scalar(0.5));
  const Scalar c = cos(angle_n * Scalar(0.5));

  setValue(axis[0] * s,
           axis[1] * s,
           axis[2] * s,
           c);
}


inline void Rotation::setYpr(const Scalar& yaw,
                             const Scalar& pitch,
                             const Scalar& roll)
{

  // Normalizes yaw, pitch and roll angles to the interval [0, 2pi)
  const Scalar yaw_n  (mt::normalize(yaw,   Scalar(0.0), TWO_PI));
  const Scalar pitch_n(mt::normalize(pitch, Scalar(0.0), TWO_PI));
  const Scalar roll_n (mt::normalize(roll,  Scalar(0.0), TWO_PI));

  const Scalar half_yaw   = yaw_n   * Scalar(0.5);
  const Scalar half_pitch = pitch_n * Scalar(0.5);
  const Scalar half_roll  = roll_n  * Scalar(0.5);

  const Scalar cy = cos(half_yaw);
  const Scalar sy = sin(half_yaw);

  const Scalar cp = cos(half_pitch);
  const Scalar sp = sin(half_pitch);

  const Scalar cr = cos(half_roll);
  const Scalar sr = sin(half_roll);

  setValue(sr * cp * cy  -  cr * sp * sy,
           cr * sp * cy  +  sr * cp * sy,
           cr * cp * sy  -  sr * sp * cy,
           cr * cp * cy  +  sr * sp * sy);

}


inline void Rotation::setMatrix(const Matrix3x3& mat)
{

  // Checks that matrix is orthogonal and represents a right-handed coordinate
  // system
  #ifdef MT_USE_BASIC_SCALAR
    const Scalar det(mat.determinant());
    const bool is_compliant = det == Scalar(1.0);
    util::Assert(is_compliant, Exception("Matrix does not comply with \
orthogonality and right-hand system requirements."));
  #endif

  if (mat[1][1] >= -mat[2][2] &&
      mat[0][0] >= -mat[1][1] &&
      mat[0][0] >= -mat[2][2])
  {
    const Scalar k = Scalar(1.0) + mat[0][0] + mat[1][1] + mat[2][2];
    const Scalar s = Scalar(2.0) * sqrt(k);
    m_co[0] = (mat[2][1] - mat[1][2]) / s;
    m_co[1] = (mat[0][2] - mat[2][0]) / s;
    m_co[2] = (mat[1][0] - mat[0][1]) / s;
    m_co[3] = Scalar(0.25) * s;
  }
  else if (mat[1][1] < -mat[2][2] &&
           mat[0][0] >=  mat[1][1] &&
           mat[0][0] >=  mat[2][2])
  {
    const Scalar k = Scalar(1.0) + mat[0][0] - mat[1][1] - mat[2][2];
    const Scalar s = Scalar(2.0) * sqrt(k);
    m_co[0] = Scalar(0.25) * s;
    m_co[1] = (mat[1][0] + mat[0][1]) / s;
    m_co[2] = (mat[0][2] + mat[2][0]) / s;
    m_co[3] = (mat[2][1] - mat[1][2]) / s;
  }
  else if (mat[1][1] >=  mat[2][2] &&
           mat[0][0] <  mat[1][1] &&
           mat[0][0] < -mat[2][2])
  {
    const Scalar k = Scalar(1.0) - mat[0][0] + mat[1][1] - mat[2][2];
    const Scalar s = Scalar(2.0) * sqrt(k);
    m_co[0] = (mat[1][0] + mat[0][1]) / s;
    m_co[1] = Scalar(0.25) * s;
    m_co[2] = (mat[2][1] + mat[1][2]) / s;
    m_co[3] = (mat[0][2] - mat[2][0]) / s;
  }
  else
  {
    const Scalar k = Scalar(1.0) - mat[0][0] - mat[1][1] + mat[2][2];
    const Scalar s = Scalar(2.0) * sqrt(k);
    m_co[0] = (mat[0][2] + mat[2][0]) / s;
    m_co[1] = (mat[2][1] + mat[1][2]) / s;
    m_co[2] = Scalar(0.25) * s;
    m_co[3] = (mat[1][0] - mat[0][1]) / s;
  }

}


inline void Rotation::setTwoPairs(const Unit3& u,
                                  const Unit3& v,
                                  const Unit3& r,
                                  const Unit3& s)
{
  // Checks that pairs of input vectors are independent
  #ifdef MT_USE_BASIC_SCALAR
    const Scalar dot_uv = abs(u.dot(v));
    const Scalar dot_rs = abs(r.dot(s));

    util::Assert((dot_uv != 1.0 && dot_rs != 1.0),
    Exception("Pairs of unit vectors are not independent."));
  #endif

  // Orthonormal basis associated to u and v
  Unit3 vu;
  Unit3 w;
  orthonormalBasis(u, v, vu, w);

  // Orthonormal basis associated to r and s
  Unit3 sr;
  Unit3 t;
  orthonormalBasis(r, s, sr, t);

  const Matrix3x3 M1(r[0], sr[0], t[0],
                     r[1], sr[1], t[1],
                     r[2], sr[2], t[2]);

  const Matrix3x3 M2(u[0], vu[0], w[0],
                     u[1], vu[1], w[1],
                     u[2], vu[2], w[2]);

  const Matrix3x3 Mrot = timesTranspose(M1, M2);
  setMatrix(Mrot);
}


inline void Rotation::setOnePair(const Unit3& u,
                                 const Unit3& v)
{
  // Rotation angle
  const Scalar ang_cos = u.angleCos(v);
  const Scalar ang     = acos(ang_cos);

  // Rotation axis
  Unit3 axis;

  if (abs(ang_cos) == 1.0)
  {
    // (Anti)parallel vectors
    // Selects a vector normal to v from the infinite possibilities
    Plane3 P;
    P.setNormal(v);
    axis = P.getSupportVector();
  }
  else
  {
    axis = cross(u, v);
  }

  // Sets rotation value
  setAxisAngle(axis, ang);
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&   os,
                                const Rotation& r)
{
  Unit3 axis;
  Scalar angle;

  r.getAxisAngle(axis, angle);

  return os << "axis: " << axis << ", angle: " << angle;
}


// FUNCTIONS

inline Scalar angleCos(const Rotation& r1,
                       const Rotation& r2)
{
  return r1.angleCos(r2);
}


inline Scalar angle(const Rotation& r1,
                    const Rotation& r2)
{
  return r1.angle(r2);
}


inline Rotation inverse(const Rotation& r)
{
  return r.inverse();
}


} // mt

#endif // MT_ROTATION_H
