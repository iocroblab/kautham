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

class urdf_origin {
public:
    mt::Point3 xyz;
    double r, p, y;
    mt::Transform transform;

    urdf_origin ();
    void fill (xml_node *node);
};

class urdf_inertia {
public:
    double ixx, ixy, ixz, iyy, iyz, izz;
    mt::Matrix3x3 matrix;

    urdf_inertia ();
    void fill (xml_node *node);
};

class urdf_inertial {
public:
    urdf_origin origin;
    double mass;
    urdf_inertia inertia;

    urdf_inertial ();
    void fill (xml_node * node);
};

class urdf_geometry {
public:
    string ivfile;
    double scale;

    urdf_geometry();
};

class urdf_dynamics {
public:
    double friction, damping;

    urdf_dynamics ();
    void fill (xml_node *node);
};

class urdf_limit {
public:
    double lower, upper, effort, velocity;

    urdf_limit ();
    void fill (xml_node *node);
};

class urdf_link {
public:
    string name, joint, type, parent;
    urdf_geometry visual, collision;
    urdf_inertial inertial;
    urdf_origin origin;
    mt::Unit3 axis;
    urdf_dynamics dynamics;
    urdf_limit limit;
    bool is_base;

    urdf_link ();
    void fill (xml_node *node);
    mt::Transform transform (double theta);
};

class urdf_robot {
public:
    string name, type;
    int num_links;
    urdf_link *link;

    urdf_robot ();
    void fill (xml_node *node);
    void print();
};
