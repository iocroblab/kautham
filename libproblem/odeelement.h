#include <mt/point3.h>
#include <mt/transform.h>

using namespace std;

/** \addtogroup libProblem
 *  @{
 */

//! Class containing origin data
class ode_origin {
public:
    mt::Point3 xyz;//!< Translation in mm, defaults to zero vector
    double r;//!< Roll angle around x axis in rad
    double p;//!< Pitch angle around y axis in rad
    double y;//!< Yaw angle around z axis in rad
    mt::Transform transform;//!< Transform

    ode_origin ();//!< Class constructor
};

//! Class containing inertia data
class ode_inertia {
public:
    double ixx;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    double ixy;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    double ixz;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    double iyy;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    double iyz;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    double izz;//!< Above-diagonal element of the matrix inertia in kg·m², defaults to zero
    mt::Matrix3x3 matrix;//!< 3x3 rotational inertia matrix, represented in the inertia frame, defaults to zero matrix

    ode_inertia ();//!< Class constructor
};

//! Class containing the inertial properties of the link
class ode_inertial {
public:
    //! Pose of the inertial reference frame, relative to the link reference frame.
    /*! The origin of the inertial reference frame needs to be at the center of gravity.
        The axes of the inertial reference frame do not need to be aligned with the principal axes of the inertia.
    */
    ode_origin origin;
    double mass;//!< Mass in kg, defaults to zero
    ode_inertia inertia;//!< Inertia

    ode_inertial ();//!< Class constructor
};

//! Class containing joint's dynamics data
class ode_dynamics {
public:
    //! Physical static friction, defaults to zero
    /*! For prismatic joints, in N and for revolving joints, in N·m
    */
    double friction;
    //! Physical damping value, defaults to zero
    /*! For prismatic joints, in N·s/m and for revolving joints, N·m·s/rad
    */
    double damping;

    ode_dynamics ();//!< class constructor
};

//! Class that defines how the links behave when they are in contact with one another.
class ode_contact_coefficients {
public:
    double mu;//!< Friction coefficient, defaults to zero
    double kp;//!< Stiffness coefficient, defaults to zero
    double kd;//!< Dampening coefficient, defaults to zero

    ode_contact_coefficients ();//!< Class constructor
};

//! Class containing the limits of the joint
class ode_limit {
public:
    double lower;//!< Lower joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double upper;//!< Upper joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double effort;//!< Enforces the maximum joint effort (|applied effort| < |effort|), defaults to zero
    double velocity;//!< Enforces the maximum joint velocity, defaults to zero

    ode_limit ();//!< Class constructor
};

//! Class containing all data needed for dynamic simulation
class ode_element {
public:
    ode_inertial inertial;//!< Inertial propierties from the link
    ode_dynamics dynamics;//!< Dynamics of the joint
    ode_contact_coefficients contact_coefficients;//!< Contact coefficients of the link
    ode_limit limit;//!< Limits of the joint
};

/** @}   end of Doxygen module "libProblem" */
