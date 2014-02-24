#include <libpugixml/pugixml.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <mt/point3.h>
#include <mt/rotation.h>
#include <mt/transform.h>

using namespace std;
using namespace pugi;

class urdf_origin {//class containing origin data
public:
    mt::Point3 xyz;//translation in mm, defaults to zero vector
    double r;//roll angle around x axis in rad
    double p;//pitch angle around y axis in rad
    double y;//yaw angle around z axis in rad
    mt::Transform transform;//transform

    urdf_origin ();//class constructor
    void fill (xml_node *node);//fills variables given an origin node
};

class urdf_inertia {//class containing inertia data
public:
    double ixx, ixy, ixz, iyy, iyz, izz;//above-diagonal elements of the matrix inertia in kg·m², defaults to zero
    mt::Matrix3x3 matrix;//3x3 rotational inertia matrix, represented in the inertia frame, defaults to zero matrix

    urdf_inertia ();//class constructor
    void fill (xml_node *node);//fills variables given an inertia node
};

class urdf_inertial {//class containing the inertial properties of the link
public:
    urdf_origin origin;//pose of the inertial reference frame, relative to the link reference frame.
    //The origin of the inertial reference frame needs to be at the center of gravity.
    //The axes of the inertial reference frame do not need to be aligned with the principal axes of the inertia.
    double mass;//mass in kg, defaults to zero
    urdf_inertia inertia;//inertia

    urdf_inertial ();//class constructor
    void fill (xml_node * node);//fills variables given an inertial node
};

class urdf_geometry {//class containing the shape of the object
public:
    string ivfile;//path to the Inventor file that contains the solid
    double scale;//model scale, defaults to one

    urdf_geometry();//class constructor
};

class urdf_dynamics {//class containing joint's dynamics data
public:
    double friction;//physical static friction, defaults to zero
    //For prismatic joints, in N and for revolving joints, in N·m
    double damping;//physical damping value, defaults to zero
    //For prismatic joints, in N·s/m and for revolving joints, N·m·s/rad

    urdf_dynamics ();//class constructor
    void fill (xml_node *node);//fills variables given a dynamics node
};

class urdf_contact_coefficients {//class that defines how the links behave when they are in contact with one another.
    //This is done with a subelement of the collision tag called contact_coefficients.
public:
    double mu;//friction coefficient, defaults to zero
    double kp;//stiffness coefficient, defaults to zero
    double kd;//dampening coefficient, defaults to zero

    urdf_contact_coefficients ();//class constructor
    void fill (xml_node *node);//fills variables given a contact_coefficients node
};

class urdf_limit {//class containing the limits of the joint
public:
    double lower;//lower joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double upper;//upper joint limit (rad for revolute joints, m for prismatic joints), defaults to zero
    double effort;//enforces the maximum joint effort (|applied effort| < |effort|), defaults to zero
    double velocity;//enforces the maximum joint velocity, defaults to zero

    urdf_limit ();//class constructor
    void fill (xml_node *node);//fills variables given a limit node
};

class urdf_link {//class that describes a link
public:
    string name;//name of the link itself
    string joint;//name of the joint whose child link is this link
    string type;//type of joint, where type can be one of the following:
    //revolute - a hinge joint that rotates along the axis and has a limited
    //range specified by the upper and lower limits.
    //continuous - a continuous hinge joint that rotates around the axis and
    //has no upper and lower limits
    //prismatic - a sliding joint that slides along the axis, and has a limited
    //range specified by the upper and lower limits.
    //fixed - This is not really a joint because it cannot move. All degrees of freedom are locked.
    //This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
    //floating - This joint allows motion for all 6 degrees of freedom.
    //planar - This joint allows motion in a plane perpendicular to the axis.
    string parent;//name of the link that is the parent of this link in the robot tree structure
    urdf_geometry visual;//visual properties of the link.
    //This element specifies the shape of the object for visualization purposes.
    urdf_geometry collision;//collision properties of a link.
    //Note that this can be different from the visual properties of a link, for example,
    //simpler collision models are often used to reduce computation time.
    urdf_inertial inertial;//inertial propierties from the link
    urdf_origin origin;//transform from the parent link frane to the joint frame
    //The joint is located at the origin of the child link
    mt::Unit3 axis;//defaults to (1,0,0)
    //The joint axis specified in the joint frame. This is the axis of rotation for revolute joints,
    //the axis of translation for prismatic joints, and the surface normal for planar joints.
    //The axis is specified in the joint frame of reference. Fixed and floating joints do not use the axis field.
    urdf_dynamics dynamics;//dynamics of the joint
    urdf_contact_coefficients contact_coefficients;//contact coefficients of the link
    urdf_limit limit;//limits of the joint
    bool is_base;//says if this links is the base from the robot tree structure, defaults to false

    urdf_link ();//class constructor
    void fill (xml_node *node);//fills variables given a robot node
    mt::Transform transform (double theta);//returns the transform from the parent link to the child link given
    //the value of the joint's degree of freedom, only for revolute joints
};

class urdf_robot {
public:
    string name;//name of the robot
    string type;//type of the robot, where type can be one of the following:
    //"Freeflying", "Chain" or "Tree"
    int num_links;//number of links forming the robot structure
    urdf_link *link;//

    urdf_robot ();//class constructor
    void fill (xml_node *node);//fills variables given a robot node
    void print();//prints robot information
};
