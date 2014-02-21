#include <mt/point3.h>
#include <mt/rotation.h>
#include <mt/transform.h>

using namespace std;

class ode_origin {
public:
    mt::Point3 xyz;
    double r, p, y;
    mt::Transform transform;

    ode_origin ();
};

class ode_inertia {
public:
    double ixx, ixy, ixz, iyy, iyz, izz;
    mt::Matrix3x3 matrix;

    ode_inertia ();
};

class ode_inertial {
public:
    ode_origin origin;
    double mass;
    ode_inertia inertia;

    ode_inertial ();
};

class ode_dynamics {
public:
    double friction, damping;

    ode_dynamics ();
};

class ode_limit {
public:
    double lower, upper, effort, velocity;

    ode_limit ();
};

class ode_element {
public:
    ode_inertial inertial;
    ode_dynamics dynamics;
    ode_limit limit;
};
