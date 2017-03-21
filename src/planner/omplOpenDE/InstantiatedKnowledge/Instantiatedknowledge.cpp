
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
#include <kautham/planner/omplOpenDE/InstantiatedKnowledge/Instantiatedknowledge.h>

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
  ManipulationRegions.push_back(region);
//    ManipulationRegion.x_max = region.x_max;
//    ManipulationRegion.x_min = region.x_min;
//    ManipulationRegion.y_min = region.y_min;
//    ManipulationRegion.y_max = region.y_max;

}

bool RigidBody::isCollisionAllowed(double x, double y)
{
    for(unsigned int i=0;i<ManipulationRegions.size();i++)
    {
    if((x > ManipulationRegions.at(i).x_min && y > ManipulationRegions.at(i).y_min &&
        (x < ManipulationRegions.at(i).x_max && y < ManipulationRegions.at(i).y_max)))
    {
        //std::cout<<"Region is :"<<ManipulationRegions.at(i).x_min<<" , "<<ManipulationRegions.at(i).x_max<<" , "
                  // <<ManipulationRegions.at(i).y_min<<" , "<<ManipulationRegions.at(i).y_max<<std::endl;
        return collisionAllowed=true;
    }
    }

    return collisionAllowed=false;

}
bool RigidBody::isContainManipulationRegions()
{
    if(ManipulationRegions.size()>0)
        return true;
    else
        return false;
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

double InstantiatedKnowledge::isRobotInManipulationRegion(double x, double y)
{
    for(unsigned int i=0;i<_rigidBody.size();i++)
    {
        if(_rigidBody.at(i).isContainManipulationRegions())
        {
           if(_rigidBody.at(i).isCollisionAllowed(x,y))
           return _rigidBody.at(i).getMass();
        }
    }
    return -1;
}

void InstantiatedKnowledge::updateKnowledge(std::vector<dBodyID> body)
{
    for(unsigned int i=0;i<_rigidBody.size();i++)
    {
        if(_rigidBody.at(i).isContainManipulationRegions())
        {
            const dReal *pos=dBodyGetPosition(body.at(i));
            if(_rigidBody.at(i).getRigidBodyType()=="coManipulatable")
            {
            for(unsigned int j=0;j<_rigidBody.at(i).getManipulationRegions().size();j++)
            {


                    if(_rigidBody.at(i).getManipulationRegions().at(j).regionDirection == "x" ||
                            _rigidBody.at(i).getManipulationRegions().at(j).regionDirection =="X")
                    {
                        _rigidBody.at(i).getManipulationRegions().at(j).x_min=pos[0]+_rigidBody.at(i).getDim()[0]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).y_min=pos[1]-_rigidBody.at(i).getDim()[1]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).x_max=_rigidBody.at(i).getManipulationRegions().at(j).x_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[0];
                        _rigidBody.at(i).getManipulationRegions().at(j).y_max= _rigidBody.at(i).getManipulationRegions().at(j).y_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[1];

                    }
                    if(_rigidBody.at(i).getManipulationRegions().at(j).regionDirection == "y" ||
                            _rigidBody.at(i).getManipulationRegions().at(j).regionDirection =="Y")
                    {
                        _rigidBody.at(i).getManipulationRegions().at(j).x_min=pos[0]-_rigidBody.at(i).getDim()[0]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).y_min=pos[1]+_rigidBody.at(i).getDim()[1]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).x_max= _rigidBody.at(i).getManipulationRegions().at(j).x_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[0];
                        _rigidBody.at(i).getManipulationRegions().at(j).y_max=_rigidBody.at(i).getManipulationRegions().at(j).y_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[1];

                    }
                    if(_rigidBody.at(i).getManipulationRegions().at(j).regionDirection == "-x" ||
                            _rigidBody.at(i).getManipulationRegions().at(j).regionDirection =="-X")
                    {
                        _rigidBody.at(i).getManipulationRegions().at(j).x_min=pos[0]-_rigidBody.at(i).getDim()[0]/2-_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[0];
                        _rigidBody.at(i).getManipulationRegions().at(j).y_min=pos[1]-_rigidBody.at(i).getDim()[1]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).x_max=_rigidBody.at(i).getManipulationRegions().at(j).x_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[0];
                        _rigidBody.at(i).getManipulationRegions().at(j).y_max=_rigidBody.at(i).getManipulationRegions().at(j).y_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[1];

                    }
                    if(_rigidBody.at(i).getManipulationRegions().at(j).regionDirection == "-y" ||
                            _rigidBody.at(i).getManipulationRegions().at(j).regionDirection =="-Y")
                    {
                        _rigidBody.at(i).getManipulationRegions().at(j).x_min=pos[0]-_rigidBody.at(i).getDim()[0]/2;
                        _rigidBody.at(i).getManipulationRegions().at(j).y_min=pos[1]-_rigidBody.at(i).getDim()[1]/2-_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[1];
                        _rigidBody.at(i).getManipulationRegions().at(j).x_max= _rigidBody.at(i).getManipulationRegions().at(j).x_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[0];
                        _rigidBody.at(i).getManipulationRegions().at(j).y_max= _rigidBody.at(i).getManipulationRegions().at(j).y_min+_rigidBody.at(i).getManipulationRegions().at(j).getRegionDim()[1];
                    }
                }
            }
            else
                if(_rigidBody.at(i).getRigidBodyType()=="freeManipulatable")
                {
                    double rx=_rigidBody.at(i).getDim()[0]*2;
                    double ry=_rigidBody.at(i).getDim()[1]*2;
                    double x=rx/2;
                    double y=ry/2;
                    //Points of daigonal
                    _rigidBody.at(i).getManipulationRegions().at(0).x_min=pos[0]-x;
                    _rigidBody.at(i).getManipulationRegions().at(0).y_min=pos[1]-y;
                    _rigidBody.at(i).getManipulationRegions().at(0).x_max = _rigidBody.at(i).getManipulationRegions().at(0).x_min+rx;
                    _rigidBody.at(i).getManipulationRegions().at(0).y_max = _rigidBody.at(i).getManipulationRegions().at(0).y_min+ry;
                }
        }
    }
}
}
}
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL


