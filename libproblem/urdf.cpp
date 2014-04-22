/***************************************************************************
 *   Copyright (C) 2014 by Nestor Garcia Hidalgo                           *
 *   nestor.garcia.hidalgo@upc.edu                                         *
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


#include "urdf.h"

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


urdf_origin::urdf_origin () {
    r = 0.;
    p = 0.;
    y = 0.;
}

void urdf_origin::fill (xml_node *node) {
    string tmpString;
    if (node->attribute("xyz").as_string()) {
        double tmpDouble[3];
        tmpString = node->attribute("xyz").as_string();
        istringstream ss( tmpString );
        getline(ss,tmpString,' ');
        tmpDouble[0] = atof(tmpString.c_str());
        getline(ss,tmpString,' ');
        tmpDouble[1] = atof(tmpString.c_str());
        getline(ss,tmpString,' ');
        tmpDouble[2] = atof(tmpString.c_str());
        xyz = 1000. * mt::Point3(tmpDouble[0],tmpDouble[1],tmpDouble[2]);
    }
    if (node->attribute("rpy").as_string()) {
        tmpString = node->attribute("rpy").as_string();
        istringstream ss(tmpString);
        getline(ss,tmpString,' ');
        r = atof(tmpString.c_str());
        getline(ss,tmpString,' ');
        p = atof(tmpString.c_str());
        getline(ss,tmpString,' ');
        y = atof(tmpString.c_str());
    }
    transform = mt::Transform(mt::Rotation(y,p,r),xyz);
}

urdf_inertia::urdf_inertia () {
    ixx = 0.;
    ixy = 0.;
    ixz = 0.;
    iyy = 0.;
    iyz = 0.;
    izz = 0.;
}

void urdf_inertia::fill (xml_node *node) {
    ixx = node->attribute("ixx").as_double();
    ixy = node->attribute("ixy").as_double();
    ixz = node->attribute("ixz").as_double();
    iyy = node->attribute("iyy").as_double();
    iyz = node->attribute("iyz").as_double();
    izz = node->attribute("izz").as_double();
    matrix = mt::Matrix3x3(ixx,ixy,ixz,ixy,iyy,iyz,ixz,iyz,izz);
}

urdf_inertial::urdf_inertial () {
    mass = 0.;
}

void urdf_inertial::fill (xml_node * node) {
    xml_node tmpNode;
    if (node->child("origin")) {
        tmpNode = node->child("origin");
        origin.fill(&tmpNode);
    }
    mass = node->child("mass").attribute("value").as_double();
    tmpNode = node->child("inertia");
    inertia.fill(&tmpNode);
}

urdf_geometry::urdf_geometry () {
    scale = 1.;
}

void urdf_geometry::fill(xml_node *node) {
    xml_node tmpNode = node->first_child();
    string type = tmpNode.name();
    if (type == "box") {
        ivfile = "box " + string(tmpNode.attribute("size").as_string());
    } else if (type == "cylinder") {
        ivfile = "cylinder " + string(tmpNode.attribute("radius").as_string())
                + string(tmpNode.attribute("length").as_string());
    } else if (type == "sphere") {
        ivfile = "sphere " + string(tmpNode.attribute("radius").as_string());
    } else if (type == "mesh") {
        ivfile = tmpNode.attribute("filename").as_string();
        if (tmpNode.attribute("scale")) {
            scale = tmpNode.attribute("scale").as_double();
        }
    }
}

urdf_dynamics::urdf_dynamics () {
    friction = 0.;
    damping = 0.;
}

void urdf_dynamics::fill (xml_node *node) {
    if (node->attribute("damping")){
        damping = node->attribute("damping").as_double();
    }
    if (node->attribute("friction")){
        friction = node->attribute("friction").as_double();
    }
}

urdf_contact_coefficients::urdf_contact_coefficients() {
    mu = 0.;
    kp = 0.;
    kd = 0.;
}

void urdf_contact_coefficients::fill(xml_node *node) {
    if (node->attribute("mu")){
        mu = node->attribute("mu").as_double();
    }
    if (node->attribute("kp")){
        kd = node->attribute("kd").as_double();
    }
    if (node->attribute("kp")){
        kp = node->attribute("kp").as_double();
    }
}

urdf_limit::urdf_limit () {
    lower = 0.;
    upper = 0.;
    effort = 0.;
    velocity = 0.;
}

void urdf_limit::fill (xml_node *node) {
    if (node->attribute("lower")) {
        lower = node->attribute("lower").as_double();
    }
    if (node->attribute("upper")) {
        upper = node->attribute("upper").as_double();
    }
    effort = node->attribute("effort").as_double();
    velocity = node->attribute("velocity").as_double();
}

urdf_link::urdf_link () {
    axis = mt::Unit3(1,0,0);
    is_base = false;
    weight = 1.0;
}

void urdf_link::fill (xml_node *node) {
    xml_node tmpNode;

    name = node->attribute("name").as_string();

    tmpNode = node->child("visual").child("geometry");
    visual.fill(&tmpNode);

    if (node->child("collision").child("geometry")) {
        tmpNode = node->child("visual").child("geometry");
        collision.fill(&tmpNode);
    } else {
        collision.ivfile = visual.ivfile;
        collision.scale = visual.scale;
    }
    if (node->child("collision").child("contact_coefficients")) {
        tmpNode = node->child("collision").child("contact_coefficients");
        contact_coefficients.fill(&tmpNode);
    }
    if (node->child("inertial")) {
        tmpNode = node->child("inertial");
        inertial.fill(&tmpNode);
    }

    tmpNode = node->parent().child("joint");
    node = &tmpNode;
    while (name != node->child("child").attribute("link").as_string() && node->next_sibling("joint")) {
        *node = node->next_sibling("joint");
    }
    if (name == node->child("child").attribute("link").as_string()) {
        joint = node->attribute("name").as_string();
        type = node->attribute("type").as_string();
        xml_node tmpNode;
        if (node->child("origin")) {
            tmpNode = node->child("origin");
            origin.fill(&tmpNode);
        }
        if (node->child("weight").attribute("value")) {
            weight = node->child("weight").attribute("value").as_double();
        }
        parent = node->child("parent").attribute("link").as_string();
        if (node->child("axis")) {
            string tmpString = "";
            double tmpDouble[3];
            tmpString = node->child("axis").attribute("xyz").as_string();
            istringstream ss(tmpString);
            getline(ss,tmpString,' ');
            tmpDouble[0] = atof(tmpString.c_str());
            getline(ss,tmpString,' ');
            tmpDouble[1] = atof(tmpString.c_str());
            getline(ss,tmpString,' ');
            tmpDouble[2] = atof(tmpString.c_str());
            axis = mt::Unit3(tmpDouble[0],tmpDouble[1],tmpDouble[2]);
        }
        if (node->child("dynamics")) {
            tmpNode = node->child("dynamics");
            dynamics.fill(&tmpNode);
        }
        tmpNode = node->child("limit");
        limit.fill(&tmpNode);
    } else {
        //link is robot's base
        //origin, parent, axis, dynamics and limit need to be filled in another way
        is_base = true;
    }
}

mt::Transform urdf_link::transform (double theta) {
    return(mt::Transform(mt::Rotation(axis,theta),mt::Point3()) * origin.transform);
}

urdf_robot::urdf_robot () {
    num_links = 0;
    link = NULL;
}

void urdf_robot::fill (xml_node *node) {
    name = node->attribute("name").as_string();

    xml_node tmpNode = node->child("link");

    while (tmpNode) {
        num_links += 1;

        tmpNode = tmpNode.next_sibling("link");
    }

    link = new urdf_link[num_links];

    int i;
    tmpNode = node->child("link");
    for (i = 0; i < num_links; i++) {
        link[i].fill(&tmpNode);

        tmpNode = tmpNode.next_sibling("link");
    }


    if (num_links > 1) {
        //find base's index
        i = 0;
        while (!link[i].is_base){
            i++;
        }

        //count links in the kinematic chain from the base
        int links = 1;
        bool end = false;
        int j;
        bool child_found;
        while (!end && links <= num_links) {
            child_found = false;
            j = 0;
            while (!child_found && j < num_links) {
                if (link[i].name == link [j].parent) {
                    child_found = true;
                } else {
                    j++;
                }
            }
            if (child_found) {
                i = j;
                links++;
            } else {
                end = true;
            }
        }

        //determine robot's type
        if (links == num_links) {
            type = "Chain";
        } else {
            type = "Tree";
        }
    } else {
        type = "Freeflying";
    }
}

void urdf_robot::print() {
    cout << "robot: " << name << endl;
    cout << "number of links: " << num_links << endl;
    for (int i = 0; i < num_links; i++) {
        cout << endl;
        cout << "link: " << link[i].name << endl;
        cout << "inertial:" << endl;
        cout << "  origin: xyz=(" << link[i].inertial.origin.xyz[0] << ", " << link[i].inertial.origin.xyz[1]
             << ", " << link[i].inertial.origin.xyz[2] << ") rpy=(" << link[i].inertial.origin.r << ", "
             << link[i].inertial.origin.p << ", " << link[i].inertial.origin.y << ")" << endl;
        cout << "  mass: " << link[i].inertial.mass << endl;
        cout << "  inertia: ixx=" << link[i].inertial.inertia.ixx << " ixy=" << link[i].inertial.inertia.ixy
             << " ixz=" << link[i].inertial.inertia.ixz << " iyy=" << link[i].inertial.inertia.iyy << " iyz="
             << link[i].inertial.inertia.iyz << " izz=" << link[i].inertial.inertia.izz << endl;
        cout << "visual: ivfile=" << link[i].visual.ivfile << " scale=" << link[i].visual.scale << endl;
        cout << "collision: ivfile=" << link[i].collision.ivfile << " scale=" << link[i].collision.scale << endl;
        cout << "parent: " << link[i].parent << endl;
        cout << "joint: " << link[i].joint << endl;
        cout << "type: " << link[i].type << endl;
        cout << "origin: xyz=(" << link[i].origin.xyz[0] << ", " << link[i].origin.xyz[1] << ", "
             << link[i].origin.xyz[2] << ") rpy=(" << link[i].origin.r << ", " << link[i].origin.p
             << ", " << link[i].origin.y << ")" << endl;
        cout << "axis: (" << link[i].axis[0] << ", " << link[i].axis[1] << ", " << link[i].axis[2] << ")" << endl;
        cout << "dynamics: damping=" << link[i].dynamics.damping << " friction=" << link[i].dynamics.friction << endl;
        cout << "contact coefficients: mu=" << link[i].contact_coefficients.mu << " kp="
             << link[i].contact_coefficients.kp << " kd" << link[i].contact_coefficients.kd << endl;
        cout << "limit: lower=" << link[i].limit.lower << " upper=" << link[i].limit.upper << " effort="
             << link[i].limit.effort << " velocity=" << link[i].limit.velocity << endl;
    }
}

void urdf_obstacle::fill (xml_node *node) {
    xml_node tmpNode;

    visual.ivfile = node->child("visual").child("geometry").child("mesh").attribute("filename").as_string();
    if (node->child("visual").child("geometry").child("mesh").attribute("scale")) {
        visual.scale = node->child("visual").child("geometry").child("mesh").attribute("scale").as_double();
    }
    if (node->child("collision").child("geometry").child("mesh").attribute("filename")) {
        collision.ivfile = node->child("collision").child("geometry").child("mesh").attribute("filename").as_string();
        if (node->child("collision").child("geometry").child("mesh").attribute("scale")){
            collision.scale = node->child("collision").child("geometry").child("mesh").attribute("scale").as_double();
        }
    } else {
        collision.ivfile = visual.ivfile;
        collision.scale = visual.scale;
    }
    if (node->child("collision").child("contact_coefficients")) {
        tmpNode = node->child("collision").child("contact_coefficients");
        contact_coefficients.fill(&tmpNode);
    }
    if (node->child("inertial")) {
        xml_node tmpNode = node->child("inertial");
        inertial.fill(&tmpNode);
    }
}

void urdf_obstacle::print() {
    cout << "obstacle: " << endl;
    cout << "inertial:" << endl;
    cout << "  origin: xyz=(" << inertial.origin.xyz[0] << ", " << inertial.origin.xyz[1]
         << ", " << inertial.origin.xyz[2] << ") rpy=(" << inertial.origin.r << ", "
         << inertial.origin.p << ", " << inertial.origin.y << ")" << endl;
    cout << "  mass: " << inertial.mass << endl;
    cout << "  inertia: ixx=" << inertial.inertia.ixx << " ixy=" << inertial.inertia.ixy
         << " ixz=" << inertial.inertia.ixz << " iyy=" << inertial.inertia.iyy << " iyz="
         << inertial.inertia.iyz << " izz=" << inertial.inertia.izz << endl;
    cout << "visual: ivfile=" << visual.ivfile << " scale=" << visual.scale << endl;
    cout << "collision: ivfile=" << collision.ivfile << " scale=" << collision.scale << endl;
    cout << "contact coefficients: mu=" << contact_coefficients.mu << " kp="
         << contact_coefficients.kp << " kd" << contact_coefficients.kd << endl;
}
