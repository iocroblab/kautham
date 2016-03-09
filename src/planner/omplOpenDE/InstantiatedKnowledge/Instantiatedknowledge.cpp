
/*************************************************************************\
   Copyright 2015 Institute of Industrial and Control Engineering (IOC)
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

/* Author: Muhayyuddin  */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#include "Instantiatedknowledge.h"

namespace Kautham
{
namespace omplcplanner
{
Region::Region()
{
//    x_min = -90;
//    x_max =  90;
//    y_min = -100;
//    y_max = -80;
}
///////////////////RigidBody Class Functions//////////////
void RigidBody::setManipulationRegion(Region region)
{
    ManipulationRegion.x_max = region.x_max;
    ManipulationRegion.x_min = region.x_min;
    ManipulationRegion.y_min = region.y_min;
    ManipulationRegion.y_max = region.y_max;

}

bool RigidBody::isCollisionAllowed(double x, double y)
{
    if((x > ManipulationRegion.x_min && y > ManipulationRegion.y_min &&
        (x < ManipulationRegion.x_max && y < ManipulationRegion.y_max)))
        collisionAllowed=true;
    else
        collisionAllowed=false;
    return collisionAllowed;

}

///////////////////Inst. Knowledge Class Functions///////////////
InstantiatedKnowledge::InstantiatedKnowledge()
{

}

void InstantiatedKnowledge::addRigidBody(RigidBody rb,dGeomID geom)
{
    _rigidBody.push_back(rb);
    rigidBodyProperties.insert(pair<dGeomID,RigidBody>(geom,rb));
}

RigidBody InstantiatedKnowledge::getManipulationConstraints(dGeomID geom)
{
    return rigidBodyProperties.at(geom);
}

}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL


