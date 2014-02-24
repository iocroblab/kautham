#include <mt/point3.h>
#include <mt/transform.h>

using namespace std;

class ode_origin {//class containing origin data
public:
    mt::Point3 xyz;//translation in mm, defaults to zero vector
    double r;//roll angle around x axis in rad
    double p;//pitch angle around y axis in rad
    double y;//yaw angle around z axis in rad
    mt::Transform transform;//transform

    ode_origin ();//class constructor
};

class ode_inertia {//class containing inertia data
public:
    double ixx, ixy, ixz, iyy, iyz, izz;//above-diagonal elements of the matrix inertia in kg·m², defaults to zero
    mt::Matrix3x3 matrix;//3x3 rotational inertia matrix, represented in the inertia frame, defaults to zero matrix

    ode_inertia ();//class constructor
};

class ode_inertial {//class containing the inertial properties of the link
public:
    ode_origin origin;//pose of the inertial reference frame, relative to the link reference frame.
    //The origin of the inertial reference frame needs to be at the center of gravity.
    //The axes of the inertial reference frame do not need to be aligned with the principal axes of the inertia.
    double mass;//mass in kg, defaults to zero
    ode_inertia inertia;//inertia

    ode_inertial ();//class constructor
};

class ode_dynamics {//class containing joint's dynamics data
public:
    double friction;//physical static friction, defaults to zero
    //For prismatic joints, in N and for revolving joints, in N·m
    double damping;//physical damping value, defaults to zero
    //For prismatic joints, in N·s/m and for revolving joints, N·m·s/rad

    ode_dynamics ();//class constructor
};

class ode_contact_coefficients {//class that defines how the links behave when they are in contact with one another.
    //This is done with a subelement of the collision tag called contact_coefficients.
public:
    double mu;//friction coefficient, defaults to zero
    double kp;//stiffness coefficient, defaults to zero
    double kd;//dampening coefficient, defaults to zero

    ode_contact_coefficients ();//class constructor
};

class ode_limit {//class containing the limits of the joint
public:
    double lower;//lower joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double upper;//upper joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double effort;//enforces the maximum joint effort (|applied effort| < |effort|), defaults to zero
    double velocity;//enforces the maximum joint velocity, defaults to zero

    ode_limit ();//class constructor
};

class ode_element {
public:
    ode_inertial inertial;//inertial propierties from the link
    ode_dynamics dynamics;//dynamics of the joint
    ode_contact_coefficients contact_coefficients;//contact coefficients of the link
    ode_limit limit;//limits of the joint
};
