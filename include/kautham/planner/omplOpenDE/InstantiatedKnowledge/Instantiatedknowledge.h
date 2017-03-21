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

/* Author: Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_InstantiatedKnowledge_H)
#define _Knowledge_H
#define dDOUBLE

#include <kautham/problem/workspace.h>
#include <ode/ode.h>
#include <iostream>
#include <vector>

namespace Kautham {
namespace omplcplanner
{

//Contains the information about the region in which the collision between robot and
//obstacle is valid.
class Region
{
public:
//compute the limit of the valid region along x-axis and y-axis. if the value of x & y
//components of the robot is between the (x_min,y_min) and (x_max,y_max) than the region
//is valid for collision.
    string regionDirection;
    string regionType;
    std::vector<double> regionDim;
    double x_min,x_max;
    double y_min,y_max;

    Region();
    inline void setXmin(const double xmin){x_min=xmin;}
    inline void setXmax(const double xmax){x_max=xmax;}
    inline void setYmin(const double ymin){y_min=ymin;}
    inline void setYmax(const double ymax){y_max=ymax;}
    inline double getXmin(){return x_min;}
    inline double getXmax(){return x_max;}
    inline double getYmin(){return y_min;}
    inline double getYmax(){return y_max;}
    inline string getRegionType(){return regionType;}
    inline void setRegionType(string rt){regionType=rt;}
    inline string getRegionDirection(){return regionDirection;}
    inline void setRegionDirection(string rD){regionDirection=rD;}
    inline void setRegionDim(std::vector<double> rDim){regionDim=rDim;}
    inline std::vector<double> getRegionDim(){return regionDim;}

};

//This class contains the properties associated with the rigid body, such as
//the type of body (fixed or manipulatable), manipulation constraints associated with the
//body, collision between robot and body is allowed or not etc.
class RigidBody
{

    //type of body can be fixed, manipulatable, or constraint oriented manipulatable body.
    std::string type;
    double mass;
    std::vector<double> bodyDimension;
    //contain the rectengular region in which the collision with the obstacle is allowed.
    std::vector<Region> ManipulationRegions;
    //true if collision between two geometries is allowed false otherwise.
    bool collisionAllowed;
    //std::vector<Region> ManipulationRegions;
public:
    inline void setRigidBodyType(string typ){type=typ;}
    inline string getRigidBodyType(){return type;}
    inline std::vector<Region> getManipulationRegions(){return ManipulationRegions;}
    inline Region getManipulationRegions(unsigned int i){return ManipulationRegions[i];}
    inline void setCollisionAllowed(bool val){collisionAllowed=val;}
    inline bool getCollisionAllowed(){return collisionAllowed;}
    void setManipulationRegion(Region region);
    bool isCollisionAllowed( double x,  double y);
    bool isContainManipulationRegions();
    inline void setMass(double m){mass=m;}
    inline double getMass(){return mass;}
    inline void setDim(std::vector<double> rbD){bodyDimension=rbD;}
    inline std::vector<double> getDim(){return bodyDimension;}

};

//Contains the instantiated knowledge about the enviroment, like types of the bodies in the
//environment, goal is occupied or not, and the mapping between ODE geometries and the associated
//constraints etc.
class InstantiatedKnowledge
{
public:
    //true if the goal is occupied by the obstacle, false otherwise.
    bool goalOccupied;    
    //containt the cherestristics of the body i.e type of body manipulation regions...
    std::vector<RigidBody> _rigidBody;
    //map of ODE geometries and the rigid body properties.
    std::map<dGeomID,RigidBody> rigidBodyProperties;
    //Critical region is the region that should not be occupied by the obstacle.
    Region CriticalRegion;//TODO:add a function that determine critical region is occupied or not.
    bool isOccupied;
    InstantiatedKnowledge();
    inline bool IsGoalOccupied(){return goalOccupied;}
    inline void setGoalAsOccupied(){goalOccupied=true;}
    inline std::vector<RigidBody>  getRigidBody(){return _rigidBody;}
    inline void  setBody(std::vector<RigidBody> bdy){_rigidBody=bdy;}
    inline RigidBody getRigidBody(int index){return _rigidBody[index];}
    void addRigidBody(RigidBody rb, dGeomID geom);
    RigidBody getManipulationConstraints(dGeomID geom);
    double isRobotInManipulationRegion(double x, double y);
    void updateKnowledge(std::vector<dBodyID> body);
};

}
}
#endif  //_Knowledge_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
