#include "odeelement.h"


#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <mt/point3.h>
#include <mt/rotation.h>
#include <mt/transform.h>

using namespace std;


ode_origin::ode_origin () {
    r = 0.;
    p = 0.;
    y = 0.;
};

ode_inertia::ode_inertia () {
    ixx = 0.;
    ixy = 0.;
    ixz = 0.;
    iyy = 0.;
    iyz = 0.;
    izz = 0.;
};

ode_inertial::ode_inertial () {
    mass = 0.;
};

ode_dynamics::ode_dynamics () {
    friction = 0.;
    damping = 0.;
};

ode_limit::ode_limit () {
    lower = 0.;
    upper = 0.;
    effort = 0.;
    velocity = 0.;
};
