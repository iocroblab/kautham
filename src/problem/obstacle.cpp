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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#include "obstacle.h"
#include <pugixml.hpp>
#include "urdf.h"


namespace Kautham {


Obstacle::Obstacle(string modFile, KthReal pos[3], KthReal ori[4], KthReal scale, LIBUSED lib, bool flagCol){
    enablecollisions = flagCol;
    libs = lib;
    string::size_type loc = modFile.find( ".urdf", 0 );
    string dir;
    if( loc != string::npos ) {
        dir = modFile.substr(0,modFile.find_last_of("/")+1);

        // Opening the file with the new pugiXML library.
        xml_document doc;
        xml_parse_result result = doc.load_file(modFile.c_str());

        //Parse the urdf file
        if(result){
            //if file could be loaded
            urdf_obstacle obstacle;
            xml_node tmpNode = doc.child("robot").child("link"); //node containing obstacle information

            obstacle.fill(&tmpNode,dir); //fill obstacle information

            switch(libs){
            case IVPQP:
            case IVSOLID:
                element = new IVPQPElement(obstacle.visual.model,
                                           obstacle.collision.model,
                                           scale,false);

                //Set ode parameters
                element->ode.inertial.inertia.ixx = obstacle.inertial.inertia.ixx;
                element->ode.inertial.inertia.ixy = obstacle.inertial.inertia.ixy;
                element->ode.inertial.inertia.ixz = obstacle.inertial.inertia.ixz;
                element->ode.inertial.inertia.iyy = obstacle.inertial.inertia.iyy;
                element->ode.inertial.inertia.iyz = obstacle.inertial.inertia.iyz;
                element->ode.inertial.inertia.izz = obstacle.inertial.inertia.izz;
                element->ode.inertial.inertia.matrix = obstacle.inertial.inertia.matrix;
                element->ode.inertial.mass = obstacle.inertial.mass;
                element->ode.inertial.origin.xyz = obstacle.inertial.origin.xyz;
                element->ode.inertial.origin.r = obstacle.inertial.origin.r;
                element->ode.inertial.origin.p = obstacle.inertial.origin.p;
                element->ode.inertial.origin.y = obstacle.inertial.origin.y;
                element->ode.inertial.origin.transform = obstacle.inertial.origin.transform;
                element->ode.contact_coefficients.mu = obstacle.contact_coefficients.mu;
                element->ode.contact_coefficients.kp = obstacle.contact_coefficients.kp;
                element->ode.contact_coefficients.kd = obstacle.contact_coefficients.kd;
            }
        } else {// the result of the file pasers is bad
            cout << "The robot file: " << modFile << " can not be read." << std::endl;
        }
    } else {
        switch(libs){
        case IVPQP:
        case IVSOLID:
            element = new IVPQPElement(modFile,modFile,scale,false);
        }
    }
    element->setPosition(pos);
    element->setOrientation(ori);
    for(int i=0; i<3; i++)
        linVel[i] = angVel[i] = (KthReal) 0.0;
}

	void Obstacle::setLinVelocity(KthReal vel[3]){
    for(int i=0; i<3; i++)
			linVel[i] = vel[i];
	}

    void Obstacle::setAngVelocity(KthReal vel[3]){
		for(int i=0; i<3; i++)
			angVel[i] = vel[i];
	}

  void* Obstacle::getModel(bool tran){
    switch(libs){
      case IVPQP:
      case IVSOLID:
        return (void*)((IVElement*)element)->ivModel(tran);
        break;
      default:
        return NULL;
    }
  }

  void* Obstacle::getCollisionModel(bool tran){
    switch(libs){
      case IVPQP:
      case IVSOLID:
        return (void*)((IVElement*)element)->collision_ivModel(tran);
        break;
      default:
        return NULL;
    }
  }

  void* Obstacle::getModelFromColl(bool tran){
    switch(libs){
      case IVPQP:
        return (void*)((IVPQPElement*)element)->getIvFromPQPModel(tran);
        break;
      case IVSOLID:
        break;
      default:
        return NULL;
    }
    return NULL;
  }
}


