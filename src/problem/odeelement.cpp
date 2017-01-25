/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#include <kautham/problem/odeelement.h>


using namespace std;


namespace Kautham {
ode_origin::ode_origin () {
    r = 0.;
    p = 0.;
    y = 0.;
}

ode_inertia::ode_inertia () {
    ixx = 0.;
    ixy = 0.;
    ixz = 0.;
    iyy = 0.;
    iyz = 0.;
    izz = 0.;
}

ode_inertial::ode_inertial () {
    mass = 0.;
}

ode_dynamics::ode_dynamics () {
    friction = 0.;
    damping = 0.;
}

ode_contact_coefficients::ode_contact_coefficients() {
    mu = 0.;
    kp = 0.;
    kd = 0.;
}

ode_limit::ode_limit () {
    lower = 0.;
    upper = 0.;
    effort = 0.;
    velocity = 0.;
}
}
