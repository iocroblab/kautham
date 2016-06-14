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

/* Author: Joan Fontanals Martinez, Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#define SCALE 1
#include <problem/workspace.h>
#include <sampling/sampling.h>
#include <boost/bind/mem_fn.hpp>
#include "KauthamOpenDEEnvironment.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <util/kthutil/kauthamdefs.h>

using namespace std;

namespace Kautham {

namespace omplcplanner{

// When you create the Environment calling the contructor. Reserves a chain (KinematicChain) that once filled by way of calling methods buildKinematicChain, will be within the chainMap mapped by name string.
// The buildKinematicChain would be the way Alfredo with ODIN parsing XML documents, as here Kautham already read the scene above all this information copy of the workspace without preocuparnos for XML documents.
// Constructor to pass all the bodies of the scene (workspace) and reading their information (position, orientation, vertices, etc ...)
// Once filled chainMap worldCreate method is called.
//! Constructor
KauthamDEEnvironment::KauthamDEEnvironment(WorkSpace *wkspace, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain): oc::OpenDEEnvironment()
{

   // ReadManipulationKnowledge();
    manipulationQuery=new ManipulationQuery();
    Instknowledge= new InstantiatedKnowledge();
    KinematicChain* chain(new KinematicChain);
    vector<KthReal> basePos;
    _NumLinksFirstRobot=wkspace->getRobot(0)->getNumLinks();
    _maxspeed=maxspeed;
    _maxContacts=maxContacts;
    _minControlSteps=minControlsteps;
    _maxControlSteps=maxControlsteps;
    _propagationStepSize=0.05;
    _erp=erp;
    _cfm=cfm;
   trimesh=false;
    for (unsigned int i=0; i < wkspace->getNumRobots(); i++)
    {

        basePos = baseGetPos(wkspace->getRobot(i));
        buildKinematicChain(chain, wkspace->getRobot(i),SCALE,basePos);
        chainMap.insert(pair<string,KinematicChain>(chain->name,*chain));
        chain->objects.clear();
        chain->joints.clear();
        chain->jointsOrdered.clear();
        chain->motors.clear();

    }
    for (unsigned int i=0; i < wkspace->getNumObstacles(); i++)
    {


        basePos = baseGetPos(wkspace->getObstacle(i));
        buildKinematicChain(chain, wkspace->getObstacle(i),SCALE,basePos);
        //buildKinematicChain(chain, wkspace->getObstacle(i),SCALE,i);
        chainMap.insert(pair<string,KinematicChain>(chain->name,*chain));

        chain->objects.clear();
        chain->joints.clear();
        chain->jointsOrdered.clear();
        chain->motors.clear();

    }

    createWorld(wkspace);



    if(wkspace->getRobot(0)->getName()=="SimpleCar")
    {
        std::vector<double> defaulttorque;
        defaulttorque.push_back(2);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(2);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(2);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(2);//+(1.5*0.7*9.8*1));
        mkinematics= new Manipulatorkinematics(defaulttorque);
    }


    if(wkspace->getRobot(0)->getNumJoints()>1 && wkspace->getRobot(0)->getName()!="SimpleCar")
    {
        std::vector<double> defaulttorque;
        defaulttorque.push_back(10);
        defaulttorque.push_back(10);
        mkinematics= new Manipulatorkinematics(2,linklength,defaulttorque);

    }
    chainMap.clear();
    stateBodiesmap_.clear();

}
// A createWorld was constantly creating odebody that by using the makeTriMesh makePrimitive or fill your data structures based on information taken from the map chainMap.
// Says Alfredo in this case never would use the function makePrimitive but I make it look longer taken into account if at any time it is deemed necessary to use it.
// Once created every body goes to stateBodiesMap Body with the same name as that structure was in chainMap.
// ******* makeTriMesh not work very well for this part is not working properly discussed as a way to determine the mass of dBodys.
// From createWorld calling methods fillstatebodies ---> only fills the vector StateBodies_ from stateBodiesmap_ making bodies of the first objects are the robot plan.
// also call the setJoints and Motors2bodies which I'm not 100% sure of its operation.
void KauthamDEEnvironment::createWorld(WorkSpace *wkspace)
{

    bodyworld=dWorldCreate();
    _OpenDEspace = dHashSpaceCreate(0);
    //_OpenDEspace =dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );
    dWorldSetGravity(bodyworld, 0,0,-9.8);
    dWorldSetCFM (bodyworld, 1e-5);
    dWorldSetERP (bodyworld, 0.8);
    //dWorldStep (bodyworld,0.5);
   // dWorldSetQuickStepNumIterations (bodyworld,20);

if(wkspace->getRobot(0)->getNumJoints()>1 && wkspace->getRobot(0)->getName()!="SimpleCar")
    trimesh=true;
    for (int i=0;i < (int(wkspace->getNumRobots())); i++)
    {
        for (unsigned int j=0;j< (wkspace->getRobot(i)->getNumLinks()); j++)
        {
            // attempt to infer that an object is a Primitive and not a trimesh depending on the size of your vector vertex in odeserver :: buildObject req.parameters as this decision is that you seems to be equivalent to the vertices
            //if (chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mesh==false)
            if(! trimesh)
            {

                dBodyID odebody;
                //odebody = makePrimitive(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes);
                const vector<double> position= chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position;
                const vector<double> orientation=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation;
                //const vector<double> vertexes=  chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes;
                const vector<double> bodyDimension=  chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].bodydimension;

                double mass=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass;
                std::string name = (wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName());

                odebody = makePrimitive(position,orientation,mass,bodyDimension,name);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())),odebody));
                bodies.push_back(odebody);

            }
            else
            {

                dBodyID odebody;
                //odebody = makeTriMesh(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].indexes,0.1);//chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass);
                const vector<double> position= chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position;
                const vector<double> orientation=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation;
                const vector<double> vertexes=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes;
                const vector<unsigned int> indexes = chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].indexes;
                const vector<double> bodyDimension=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].bodydimension;
                const vector<float> color=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].color;
                double mass= wkspace->getRobot(i)->getLink(j)->getOde().inertial.mass;
                std::string name = (wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName());
                std::string name1 = wkspace->getRobot(i)->getName();
                //double mass=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass;
                // std::string name = Instknowledge->getBody()[0].type;
                std::cout<<"Name of Rob Part: " <<name<<std::endl;

                odebody = makeTriMesh(position,orientation,vertexes,indexes,mass,name,bodyDimension, color);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())),odebody));
                bodies.push_back(odebody);

                //bodymap.insert(pair<string,dBodyID>(wkspace->getRobot(i)->getName(),odebody));
               // getBody.insert(pair<string,dBodyID>(wkspace->getRobot(i)->getName(),odebody));

            }
        }

    }
    for (int i=0;i < (int(wkspace->getNumObstacles()));i++)
    {
        for (unsigned int j=0;j < (wkspace->getObstacle(i)->getNumLinks()); j++)
        {
            //if (chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mesh==false)
            if(! trimesh)
            {

                dBodyID odebody;
                //odebody = makePrimitive(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes);
                const vector<double> position= chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position;
                const vector<double> orientation=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation;
                const vector<double> vertexes=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes;
                const vector<double> bodyDimension=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].bodydimension;
                double mass=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass;

                std::string name = wkspace->getObstacle(i)->getName();

                odebody = makePrimitive(position,orientation,mass,bodyDimension,name);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())),odebody));
                bodies.push_back(odebody);
                getBody.insert(pair<string,dBodyID>(wkspace->getObstacle(i)->getName(),odebody));

            }
            else
            {

                dBodyID odebody;
                const vector<double> position= chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position;
                const vector<double> orientation=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation;
                const vector<double> vertexes=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes;
                const vector<unsigned int> indexes = chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes;
                const vector<double> bodyDimension=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].bodydimension;
                const vector<float> color=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].color;

                //double mass=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass;
                double mass=wkspace->getObstacle(i)->getLink(j)->getOde().inertial.mass;
                //std::string name = (wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName());
                std::string name = wkspace->getObstacle(i)->getName();

               // std::cout<<"Name of Obs Part: " <<name<<std::endl;

                if(mass==0)
                {
                    mass=1;
                }

                // odebody = makeTriMesh(chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass);

                odebody = makeTriMesh(position,orientation,vertexes,indexes,mass,name,bodyDimension,color);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())),odebody));
                std::cout<<"Obs body name"<<((wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName()))<<std::endl;
                bodies.push_back(odebody);
                getBody.insert(pair<string,dBodyID>(wkspace->getObstacle(i)->getName(),odebody));

            }

            // dBodyDestroy(odebody);
        }
    }
    /////////////////////////////////////////////////////////////////////////////
if(wkspace->getRobot(0)->getName()=="SimpleCar")
{
//Car joint construction using Hing2 Joint of ODE
            for (int i=0; i<4; i++)
            {
                cJoint[i] = dJointCreateHinge2 (bodyworld,0);
                dJointAttach (cJoint[i],bodies[0],bodies[i+1]);
                const dReal *a = dBodyGetPosition (bodies[i+1]);
                dJointSetHinge2Anchor (cJoint[i],a[0],a[1],a[2]);
                dJointSetHinge2Axis1 (cJoint[i],0,0,(i<2 ? 1 : -1));
                dJointSetHinge2Axis2 (cJoint[i],0,1,0);
                _Joint.push_back(cJoint[i]);
                std::cout<<"position "<<a[0]<<" , "<<a[1]<< " , "<<a[2]<<std::endl;
                const dReal *ro=dBodyGetRotation(bodies[1]);
                std::cout<<"rotation"<<ro[0]<< " , " <<ro[1]<< " , "<<ro[2] << " , " <<ro[3]<<std::endl;
            }
            // set joint suspension
            for (int i=0; i<4; i++)
            {
                dJointSetHinge2Param (cJoint[i],dParamSuspensionERP,0.4);
                dJointSetHinge2Param (cJoint[i],dParamSuspensionCFM,0.8);
            }
            feedback1= new dJointFeedback;
            feedback2= new dJointFeedback;
            feedback3= new dJointFeedback;
            feedback4= new dJointFeedback;
            dJointSetFeedback(cJoint[0],feedback1);
            dJointSetFeedback(cJoint[1],feedback2);
            dJointSetFeedback(cJoint[2],feedback3);
            dJointSetFeedback(cJoint[3],feedback4);

}
    /////////////////////////////////////////////////////////////////////////////
if(wkspace->getRobot(0)->getNumJoints()>1 && wkspace->getRobot(0)->getName()!="SimpleCar")
{
    setjointsandmotors2bodies(stateBodiesmap_,chainMap,wkspace);

    feedback1= new dJointFeedback;
    feedback2= new dJointFeedback;
    dJointSetFeedback(motor_.at("Chainbase_link+Chainlink1"),feedback1);
    dJointSetFeedback(motor_.at("Chainlink1+Chainlink2"),feedback2);
    dJointID fixedjoint = dJointCreateFixed(bodyworld, 0);
    dJointAttach(fixedjoint, bodies[0], 0);
    dJointSetFixed(fixedjoint);
}
stateBodies_=fillstatebodies(stateBodiesmap_,wkspace);

    // ////////////////////////////////////For Debuging ////////////////////////////
    std::cout<<"Total Bodies are "<<stateBodies_.size()<<std::endl;

    for(unsigned int i=0;i<stateBodies_.size();i++)
    {
        const dReal *pos=  dBodyGetPosition(stateBodies_[i]);
        const dReal *rot= dBodyGetQuaternion(stateBodies_[i]);
        std::cout<<"Q "<<wkspace->getObstacle(0)->getLink(0)->getElement()->getOrientation()[0]<<" , "
                   <<wkspace->getObstacle(0)->getLink(0)->getElement()->getOrientation()[1]<<" , "
                     <<wkspace->getObstacle(0)->getLink(0)->getElement()->getOrientation()[2]<<" , "
                       <<wkspace->getObstacle(0)->getLink(0)->getElement()->getOrientation()[3]<<std::endl;
        std::cout<<"Position of Body "<<i+1<<" is " <<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
        std::cout<<"Orientation of Body "<<i+1<<" is " <<rot[0]<<" "<<rot[1]<<" "<<rot[2]<<" "<<rot[3]<<std::endl;


    }
    ground = dCreatePlane(_OpenDEspace,0,0,1,0);
    geomname.insert(pair<dGeomID,std::string>(ground,"odeGround"));
    GeomID.push_back(ground);

     SetPlanningParameters();


}
void KauthamDEEnvironment::ReadManipulationKnowledge()
{
    std::ifstream manipulaionKnowledge;
    manipulaionKnowledge.open ("knowledge.txt");
    std::vector<string> bodyname;
    std::vector<string> bodytype;
    std::vector<string> mRgnDirection;
    std::vector<bool> collisionflag;
while(!manipulaionKnowledge.eof())
{
    std::string bn,bt,mrd;
    bool cf;
    manipulaionKnowledge>>bn;
    manipulaionKnowledge>>bt;
    manipulaionKnowledge>>cf;

    if(bt=="coManipulatable")
    {
       manipulaionKnowledge>>mrd;
       mRgnDirection.push_back(mrd);
    }
    bodyname.push_back(bn);
    bodytype.push_back(bt);
    collisionflag.push_back(cf);


}
    manipulaionKnowledge.close();
    std::cout<<"Done"<<std::endl;
    unsigned int j=0;
    for(unsigned int i=0;i<bodytype.size();i++)
    {
        std::cout<<"Body Name is [ "<<bodyname[i]<<" ] Body Type is [ "<<bodytype[i]<<" ] collision  is [ "<<collisionflag[i]<<" ] ";//<<std::endl;
        if(bodytype[i]=="coManipulatable")
        {
        std::cout<<"Manipulation direction is [ "<<mRgnDirection [j]<<" ]"<<std::endl;
        j++;
        }
        else
        std::cout<<std::endl;

}
}

string KauthamDEEnvironment::InferenceProcess(string rigidbodyName,dBodyID body, dGeomID geom)
{
    RigidBody rb;
    for(unsigned int i=0;i<bodyname.size();i++)
    {
        if(rigidbodyName == bodyname[i]  )
        {
            if(bodytype[i]=="coManipulatable")
            {
                rb.setRigidBodyType(bodytype[i]);

                Region region;
                const dReal* pos = dBodyGetPosition(body);
                const dReal* q   = dBodyGetQuaternion(body);
                mt::Transform Tbody;
                Tbody.setTranslation(mt::Point3(pos[0],pos[1],pos[2]));
                Tbody.setRotation(mt::Rotation(q[0],q[1],q[2],q[3]));
                mt::Transform offsetTmax, offsetTmin;
                //assume cube has size 20 by 20 so the offset from the origin
                //of the rigid body will be fixed

                offsetTmax.setTranslation(mt::Point3(15.0,60.0,0.0));
                offsetTmin.setTranslation(mt::Point3(-15.0,-60.0,0.0));

                mt::Transform mRegionTmax,mRegionTmin;
                mRegionTmax=Tbody*offsetTmax;
                mRegionTmin=Tbody*offsetTmin;

                mt::Point3 mRegionPmax,mRegionPmin;
                mRegionPmax=mRegionTmax.getTranslation();
                mRegionPmin=mRegionTmin.getTranslation();

                region.x_max=mRegionPmax[0];
                region.y_max=mRegionPmin[1];
                region.x_min=mRegionPmin[0];
                region.y_min=mRegionPmax[1];
                //            region.x_max=41;
                //            region.y_max=74;
                //            region.x_min=24;
                //            region.y_min=6;
                rb.setManipulationRegion(region);
                Instknowledge->addRigidBody(rb,geom);
                std::cout<<"Xmin and xMax are: "<<region.x_min<<" , "<<region.x_max<<std::endl;
                std::cout<<"ymin and ymax are: "<<region.y_min<<" , "<<region.y_max<<std::endl;

                break;
            }

            else
            {
                rb.setRigidBodyType(bodytype[i]);
                rb.setCollisionAllowed(collisionflag[i]);
                Instknowledge->addRigidBody(rb,geom);
                std::cout<<"Body Name is |||"<<bodyname[i]<<" ||| Body Type is ||| "<<bodytype[i]<<" ||| collision  is ||| "<<collisionflag[i]<<" |||"<<std::endl;
                break;
            }
        }
    }

    return rb.getRigidBodyType();
}
string KauthamDEEnvironment::InstknowledgeInference(bodyInfo bodyinfo)
{
    RigidBody rb;
    Region region;

    std::cout<<"The name of body is "<<bodyinfo.bodyName<<std::endl;
    if(bodyinfo.bodyName == "SphereDEbase" || bodyinfo.bodyName == "Chainbase_link"
            || bodyinfo.bodyName == "Chainlink1" || bodyinfo.bodyName == "Chainlink2" || bodyinfo.bodyName=="SimpleCarbase_link"
            ||bodyinfo.bodyName=="SimpleCarfront_left_wheel_link"||bodyinfo.bodyName=="SimpleCarfront_right_wheel_link"||bodyinfo.bodyName=="SimpleCarback_right_wheel_link"||bodyinfo.bodyName=="SimpleCarback_left_wheel_link")
    {
        rb.setMass(bodyinfo.mass);

        rb.setRigidBodyType("robBody");
        rb.setCollisionAllowed(true);
        Instknowledge->addRigidBody(rb,bodyinfo.Geom);
        std::cout<<"Rob Part Added"<<std::endl;

    }
    else if(bodyinfo.bodyName == "Chaingripper"||bodyinfo.bodyName == "Chainfinger1"||bodyinfo.bodyName == "Chainfinger2")
    {
        rb.setMass(bodyinfo.mass);
        rb.setRigidBodyType("tcp");
        rb.setCollisionAllowed(true);
        Instknowledge->addRigidBody(rb,bodyinfo.Geom);
        std::cout<<"Rob Part Added"<<std::endl;
    }
    else
        if(bodyinfo.bodyName == "cubeDECoManipulatable")/* || rigidbodyName == "RedCubeDE1"
                                                    || rigidbodyName == "RedCubeDE3"
                                                    || rigidbodyName == "RedCubeDE4")*/
        {
            rb.setRigidBodyType("coManipulatable");
            const dReal* pos = dBodyGetPosition(bodyinfo.body);
            const dReal* q   = dBodyGetQuaternion(bodyinfo.body);
            mt::Transform Tbody;
            Tbody.setTranslation(mt::Point3(pos[0],pos[1],pos[2]));
            Tbody.setRotation(mt::Rotation(q[0],q[1],q[2],q[3]));
            mt::Transform offsetTmax, offsetTmin;
            for(unsigned int i=0;i<bodyinfo.regionDir.size();i++)
            {
                if(bodyinfo.regionDir[i]=="x" || bodyinfo.regionDir[i]=="X")
                {
                    region.x_min=pos[0]+bodyinfo.dimBody[0]/2;
                    region.y_min=pos[1]-bodyinfo.dimBody[1]/2;
                    region.x_max=region.x_min+bodyinfo.dimRegion[0];
                    region.y_max=region.y_min+bodyinfo.dimRegion[1];
                    region.setRegionType("comR");
                    region.setRegionDirection(bodyinfo.regionDir[i]);
                    rb.setManipulationRegion(region);

                }
                //if(bodyinfo.regionDir[i]=="y" || bodyinfo.regionDir[i]=="Y")
                {
                    region.x_min=pos[0]-bodyinfo.dimBody[0]/2;
                    region.y_min=pos[1]+bodyinfo.dimBody[1]/2;
                    region.x_max=region.x_min+bodyinfo.dimRegion[0];
                    region.y_max=region.y_min+bodyinfo.dimRegion[1];
                    region.setRegionType("comR");
                    region.setRegionDirection(bodyinfo.regionDir[i]);
                    rb.setManipulationRegion(region);

                }
                if(bodyinfo.regionDir[i]=="-x" || bodyinfo.regionDir[i]=="-X")
                {
                    region.x_min=pos[0]-bodyinfo.dimBody[0]/2-bodyinfo.dimRegion[0];
                    region.y_min=pos[1]-bodyinfo.dimBody[1]/2;
                    region.x_max=region.x_min+bodyinfo.dimRegion[0];
                    region.y_max=region.y_min+bodyinfo.dimRegion[1];
                    region.setRegionType("comR");
                    region.setRegionDirection(bodyinfo.regionDir[i]);
                    rb.setManipulationRegion(region);

                }
                //if(bodyinfo.regionDir[i]=="-y" || bodyinfo.regionDir[i]=="-Y")
                {
                    region.x_min=pos[0]-bodyinfo.dimBody[0]/2;
                    region.y_min=pos[1]-bodyinfo.dimBody[1]/2-bodyinfo.dimRegion[1];
                    region.x_max=region.x_min+bodyinfo.dimRegion[0];
                    region.y_max=region.y_min+bodyinfo.dimRegion[1];
                    region.setRegionType("comR");
                    region.setRegionDirection(bodyinfo.regionDir[i]);
                    rb.setManipulationRegion(region);

                }
            }

//            offsetTmax.setTranslation(mt::Point3(4.0,6.0,0.0));
//            offsetTmin.setTranslation(mt::Point3(-4.0,-6.0,0.0));

//            mt::Transform mRegionTmax,mRegionTmin;
//            mRegionTmax=Tbody*offsetTmax;
//            mRegionTmin=Tbody*offsetTmin;

//            mt::Point3 mRegionPmax,mRegionPmin;
//            mRegionPmax=mRegionTmax.getTranslation();
//            mRegionPmin=mRegionTmin.getTranslation();

//            region.x_max=mRegionPmax[0];
//            region.y_max=mRegionPmin[1];
//            region.x_min=mRegionPmin[0];
//            region.y_min=mRegionPmax[1];
            //            region.x_max=41;
            //            region.y_max=74;
            //            region.x_min=24;
            //            region.y_min=6;
            rb.setMass(bodyinfo.mass);
            rb.setDim(bodyinfo.dimBody);
            Instknowledge->addRigidBody(rb,bodyinfo.Geom);
            std::cout<<"Xmin and xMax are: "<<region.x_min<<" , "<<region.x_max<<std::endl;
            std::cout<<"ymin and ymax are: "<<region.y_min<<" , "<<region.y_max<<std::endl;


        }

        else
            if(bodyinfo.bodyName == "DEPlane" || bodyinfo.bodyName == "Plan")
            {
                rb.setMass(bodyinfo.mass);
                rb.setRigidBodyType("floor");
                rb.setCollisionAllowed(true);
                Instknowledge->addRigidBody(rb,bodyinfo.Geom);
                std::cout<<"floor Added"<<std::endl;

            }
            else
                if(bodyinfo.bodyName == "cubeDE"||bodyinfo.bodyName == "cubeDE1" || bodyinfo.bodyName == "cubeDE2" || bodyinfo.bodyName == "cubeDE3" || bodyinfo.bodyName == "cubeDE4" || bodyinfo.bodyName == "cubeDE5"
                        || bodyinfo.bodyName == "cubeDE6" || bodyinfo.bodyName == "cubeDE7" || bodyinfo.bodyName == "cubeDE8")
                {
                    const dReal* pose = dBodyGetPosition(bodyinfo.body);
                    switch(bodyinfo.dimBody.size())
                    {
                    case 1:
                        //sphere todo:
                        bodyinfo.dimBody[0]=bodyinfo.dimBody[0]*2;
                        break;
                    case 2:
                        //cylinder todo:
                        bodyinfo.dimBody[0]=bodyinfo.dimBody[0]*2;
                        bodyinfo.dimBody[1]=bodyinfo.dimBody[1]*2;
                        break;
                    case 3:
                        //to make the manipulation regon around the cube with the size
                        double rx=bodyinfo.dimBody[0]*2;
                        double ry=bodyinfo.dimBody[1]*2;
                        //bodyinfo.dimBody[2]=bodyinfo.dimBody[2]*3;
                        double x=rx/2;
                        double y=ry/2;
                        //Points of daigonal
                        region.x_min=pose[0]-x;
                        region.y_min=pose[1]-y;
                        //P1[2]=pose[2];
                        region.x_max=region.x_min+rx;
                        region.y_max=region.y_min+ry;
                        region.setRegionType("freeR");
                        //P2[2]=P1[2]+dim[2];
                        rb.setManipulationRegion(region);
                        //std::cout<<"Body name is: "<<bodyinfo.bodyName<<std::endl;
//                        std::cout<<"Xmin and xMax are: "<<region.x_min<<" , "<<region.x_max<<std::endl;
//                        std::cout<<"ymin and ymax are: "<<region.y_min<<" , "<<region.y_max<<std::endl;
                        break;

                    }
                    rb.setMass(bodyinfo.mass);
                    rb.setRigidBodyType("freeManipulatable");
                    std::cout<<"Body name is: "<<bodyinfo.bodyName<<" Type is :"<< rb.getRigidBodyType()<<std::endl;

                    rb.setDim(bodyinfo.dimBody);
                    rb.setCollisionAllowed(true);
                    Instknowledge->addRigidBody(rb,bodyinfo.Geom);


                }

                else
                    if(bodyinfo.bodyName == "RedCubeDE1"||bodyinfo.bodyName == "RedCubeDE2"||bodyinfo.bodyName == "RedCubeDE3"||bodyinfo.bodyName == "RedCubeDE4"||bodyinfo.bodyName == "RedCubeDE5"||bodyinfo.bodyName == "RedCubeDE6"||bodyinfo.bodyName == "RedCubeDE7"||bodyinfo.bodyName == "RedCubeDE8")
                    {
                        rb.setMass(bodyinfo.mass);
                        rb.setRigidBodyType("fixed");
                        rb.setCollisionAllowed(true);
                        rb.setDim(bodyinfo.dimBody);
                        Instknowledge->addRigidBody(rb,bodyinfo.Geom);


                    }
                    else
                        if(bodyinfo.bodyName == "cylinder1"||bodyinfo.bodyName == "cylinder2"||bodyinfo.bodyName == "cylinder3"|| bodyinfo.bodyName == "SimpleCar")
                        {
                            rb.setMass(bodyinfo.mass);

                            rb.setRigidBodyType("freeManipulatable");
                            //rb.setRigidBodyType("fixed");
                            rb.setDim(bodyinfo.dimBody);
                            rb.setCollisionAllowed(true);
                            Instknowledge->addRigidBody(rb,bodyinfo.Geom);
                        }
                        else
                            if(bodyinfo.bodyName == "prism"||bodyinfo.bodyName == "prism1"||bodyinfo.bodyName == "prism2"||bodyinfo.bodyName == "prism3"||bodyinfo.bodyName == "prism4")
                            {
                                rb.setMass(bodyinfo.mass);

                                rb.setRigidBodyType("fixed");
                                rb.setCollisionAllowed(false);
                                Instknowledge->addRigidBody(rb,bodyinfo.Geom);
                            }
                            else
                                if(bodyinfo.bodyName == "box1"||bodyinfo.bodyName == "box2"||bodyinfo.bodyName == "box3"||bodyinfo.bodyName == "box5"||bodyinfo.bodyName == "box5")
                                {
                                    const dReal* pose = dBodyGetPosition(bodyinfo.body);
                                    rb.setMass(bodyinfo.mass);
                                    rb.setRigidBodyType("freeManipulatable");
                                    rb.setCollisionAllowed(true);
                                    rb.setDim(bodyinfo.dimBody);
                                    //to make the manipulation regon around the box with the size
                                    double rx=bodyinfo.dimBody[0]*3;
                                    double ry=bodyinfo.dimBody[1]*3;
                                    //bodyinfo.dimBody[2]=bodyinfo.dimBody[2]*3;
                                    double x=rx/2;
                                    double y=ry/2;
                                    //Points of daigonal
                                    region.x_min=pose[0]-x;
                                    region.y_min=pose[1]-y;
                                    //P1[2]=pose[2];
                                    region.x_max=region.x_min+rx;
                                    region.y_max=region.y_min+ry;
                                    region.setRegionType("freeR");
                                    //P2[2]=P1[2]+dim[2];

                                    rb.setManipulationRegion(region);
                                    Instknowledge->addRigidBody(rb,bodyinfo.Geom);


                                }

                        else
                        {
                            rb.setMass(bodyinfo.mass);
                            rb.setDim(bodyinfo.dimBody);
                            rb.setRigidBodyType("fixed");
                            std::cout<<"Body name is: "<<bodyinfo.bodyName<<" Type is :"<< rb.getRigidBodyType()<<std::endl;

                            rb.setCollisionAllowed(false);
                            Instknowledge->addRigidBody(rb,bodyinfo.Geom);
                        }


    return rb.getRigidBodyType();
}

//! void destructor
KauthamDEEnvironment::~KauthamDEEnvironment(){

}

void KauthamDEEnvironment::destroyWorld()
{
    dSpaceDestroy(_OpenDEspace);
    dWorldDestroy(bodyworld);
}

map<string, dBodyID> stateBodiesmap_;

void KauthamDEEnvironment::SetPlanningParameters()
{
    world_=bodyworld;
    collisionSpaces_.push_back(_OpenDEspace);
    geomNames_= geomname;
    stepSize_ =  _propagationStepSize;
    maxContacts_ = _maxContacts;
    minControlSteps_ = _minControlSteps;
    maxControlSteps_ = _maxControlSteps;

}


void KauthamDEEnvironment::setjointsandmotors2bodies(map<string, dBodyID> stateBodiesmap_,map<string, KinematicChain> chainMap, WorkSpace* wkspace)
{
    for (map<string,Joint>::iterator it = chainMap[wkspace->getRobot(0)->getName()].joints.begin(); it!=chainMap[wkspace->getRobot(0)->getName()].joints.end(); ++it)
       {
           dBodyID body1 = stateBodiesmap_[it->second.target1];
           dBodyID body2 = stateBodiesmap_[it->second.target2];
           std::cout<<"Target is "<<it->second.target1+it->second.target2<<std::endl;
           dJointID joint = makeJoint(body1,body2,it->second.type,it->second.position, chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].hiStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].loStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].value);
           //makeMotor(body1,body2,it->second.type,it->second.position,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax);
           //dJointID motorjoint = makeMotor(body1,body2,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].type,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].pos);
           dJointID motor = addMotor2Joint(joint,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax, it->second.value, chainMap[wkspace->getRobot(0)->getName()].joints[it->first].hiStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].loStop);
           _Joint.push_back(joint);
           joint_.insert(pair<std::string,dJointID>(it->second.target1+"+"+it->second.target2,joint));
           motor_.insert(pair<std::string,dJointID>(it->second.target1+"+"+it->second.target2,motor));


       }


}

vector<dBodyID> KauthamDEEnvironment::fillstatebodies(map<string,dBodyID> stateBodiesmap_,WorkSpace *wkspace)
{

    vector<dBodyID> bodies;
    //per si algun cop es carrega una escena només per simular sense robots
    if(wkspace->getNumRobots()<1)
    {
        for(map<string,dBodyID>::iterator it = stateBodiesmap_.begin(); it!=stateBodiesmap_.end(); ++it)
        {
            bodies.push_back(it->second);
            std::cout<<"Hello I am state Body"<<std::endl;
        }
    }
    else
    {
        //comprovem que no sigui 'taula'
        if(wkspace->getRobot(0)->getNumLinks()==1)
        {
            for (unsigned int i=0; i < wkspace->getNumRobots(); i++)
            {
                bodies.push_back(stateBodiesmap_[(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(0)->getName())]);
                std::cout<<"Hello I am Robot Body"<<std::endl;
                std::cout<<(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(0)->getName())<<std::endl;


            }

            for (unsigned int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (unsigned int j=0; j < (wkspace->getObstacle(i)->getNumLinks()); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()]);
                    std::cout<<"Hello I am Obstacle Body"<<std::endl;
                    std::cout<<wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()<<std::endl;


                }
            }

        }
        else
        {
            for (unsigned int i=0; i < wkspace->getNumRobots(); i++)
            {
                for (unsigned int j=0; j < wkspace->getRobot(i)->getNumLinks(); j++ )
                {
                    bodies.push_back(stateBodiesmap_[(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(j)->getName())]);
                    std::cout<<"Name of the Body is "<<(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(j)->getName())<<std::endl;

                }
            }
            for (unsigned int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (unsigned int j=0; j < wkspace->getObstacle(i)->getNumLinks(); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()]);
                    std::cout<<"Name of the Body is "<<wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()<<std::endl;

                }
            }
        }
    }
    return bodies;
}

bool KauthamDEEnvironment::buildKinematicChain(KinematicChain* chain,
                                               Robot *robot,
                                               double scale,
                                               vector< KthReal >& basePos)
{
    chain->name = robot->getName();
    chain->dhType = robot->getDHApproach();
   unsigned int i=0;
   bool rotflag=true;

    if (!basePos.empty())
    {
        odinObject base;
        //base.mass = 2;

        base.mass = robot->getLink(0)->getOde().inertial.mass;
        base.name = (chain->name) + (robot->getLink(0)->getName());
        rotflag=true;
        getTrimesh((SoSeparator*)robot->getLink(i)->getCollisionModel(false),&base, scale);
        getPrimitiveShapes(robot->getLink(i), &base,rotflag);

        for (int j = 0; j < 3; j++)
        {
            base.position.push_back(basePos[j]);
            base.orientation.push_back(basePos[j+3]);
        }
        base.orientation.push_back(basePos[6]);

        chain->objects[base.name] = base;
        i++;
    }
    for(; i < robot->getNumLinks(); i++)
    {
        odinObject obj;
        obj.mass = robot->getLink(i)->getOde().inertial.mass;
        obj.name = (chain->name) + (robot->getLink(i)->getName());
        getTrimesh((SoSeparator*)robot->getLink(i)->getCollisionModel(false),&obj, scale);
        getPrimitiveShapes(robot->getLink(i), &obj,rotflag);
        rotflag=true;
        vector<double> rotAxis;
        getTransformation(chain, robot->getLink(i), &obj, chain->name, rotAxis);
        getMotion(chain, robot->getLink(i), &obj, chain->name, rotAxis);
        chain->objects[obj.name] = obj;
    }

    return true;
}
void KauthamDEEnvironment::getPrimitiveShapes(Link* link, odinObject *obj,bool rotflag)
{
    obj->mesh=false;
    SoSeparator *model = (SoSeparator*)link->getCollisionModel(false)->getChild(1);
    double scale = link->getElement()->getScale();
    for (unsigned int j = 0; j < (unsigned int) model->getNumChildren(); j++)
    {

        std::cout << (model->getChild(j)->getTypeId() == SoSeparator::getClassTypeId()) << std::endl;
        SoSeparator *subModel =  (SoSeparator*)model->getChild(j);
        std::cout << "numChildren " << subModel->getNumChildren() << std::endl;
        for (unsigned int i = 0; i < (unsigned int)subModel->getNumChildren(); ++i) {
            SoNode *node = subModel->getChild(i);

            if (node->getTypeId() == SoTranslation::getClassTypeId())
            {
                SoTranslation *trans = (SoTranslation*)node;
                const SbVec3f pose = trans->translation.getValue();
                std::cout<<"SoSeperator Pos is "<<pose[0]*scale<<" , "<<pose[1]*scale<<" , "<<pose[2]*scale<<std::endl;
//                obj->position.push_back(pose[0]*scale);
//                obj->position.push_back(pose[1]*scale);
//                obj->position.push_back(pose[2]*scale);

            }
            else if (node->getTypeId() == SoRotation::getClassTypeId())
            {
                if(rotflag)
                {
                SoRotation *rot = (SoRotation*)node;
                const SbRotation &rotation = rot->rotation.getValue();
                std::cout<<"SoSeperator Rot is "<<rotation[0]<<" , "<<rotation[1]<<" , "<<rotation[2]<<" , "<<rotation[3]<<std::endl;
//                obj->orientation.push_back(rotation[0]);
//                obj->orientation.push_back(rotation[1]);
//                obj->orientation.push_back(rotation[2]);
//                obj->orientation.push_back(rotation[3]);
                rotflag= false;
                }

            }
            else if (node->getTypeId() == SoCube::getClassTypeId()) {
                SoCube *cube = (SoCube*)node;
                std::cout<<"SoSeperator Cube Size "<<cube->width.getValue()*scale<<" , "<<cube->height.getValue()*scale<<" , "<<cube->depth.getValue()*scale<<std::endl;
                obj->bodydimension.push_back(cube->width.getValue()*scale);
                obj->bodydimension.push_back(cube->height.getValue()*scale);
                obj->bodydimension.push_back(cube->depth.getValue()*scale);

            }
            else if (node->getTypeId() == SoSphere::getClassTypeId()) {
                SoSphere *sphere = (SoSphere*)node;
                obj->bodydimension.push_back( sphere->radius.getValue()*scale);
                std::cout<<"SoSeperator Sphere Size "<<sphere->radius.getValue()*scale<<std::endl;
            }
            else if (node->getTypeId()  == SoCylinder::getClassTypeId()) {
                SoCylinder *cylinder = (SoCylinder*)node;
                obj->bodydimension.push_back( cylinder->radius.getValue()*scale);
                obj->bodydimension.push_back( cylinder->height.getValue()*scale);
                linklength.push_back(cylinder->height.getValue()*scale);
                std::cout<<"link length is  "<<cylinder->height.getValue()*scale<<std::endl;
            }
            else if (node->getTypeId()  == SoMaterial::getClassTypeId())
            {
                //SoScale *scale = (SoScale*)node;
                SoMaterial *color=(SoMaterial*)node;
                SbColor col = color->diffuseColor[0];
                obj->color.push_back(col[0]);
                obj->color.push_back(col[1]);
                obj->color.push_back(col[2]);

                std::cout<<"RGB Value is  =============="<<col[0]<<" "<<col[1]<<" "<<col[2]<<std::endl;

            }
            else
                std::cout<<" Type "<< node->getTypeId().getName()<<std::endl;

        }


    }
}

void KauthamDEEnvironment::getTrimesh(SoSeparator *ivmodel, trimeshD* obj, double scale)
{
    SoScale * sca = new SoScale();
    sca->scaleFactor.setValue((float)scale,(float)scale,(float)scale);
    ivmodel->addChild(sca);
    vector<double> vec;
    SoCallbackAction triAction;
    triAction.addTriangleCallback(SoShape::getClassTypeId(), triang_CB, &vec);
    triAction.apply(ivmodel);
    unsigned int count = 0;
    double tolerance = 0.00001;

    for (unsigned int i = 0; i < vec.size(); i+=3)
    {
        unsigned int j = 0;
        for (; j<obj->vertexes.size(); j+=3)
            if (((obj->vertexes[j]-tolerance)<=vec[i])&&((obj->vertexes[j]+tolerance)>=vec[i])
                    &&((obj->vertexes[j+1]-tolerance)<=vec[i+1])&&((obj->vertexes[j+1]+tolerance)>=vec[i+1])
                    &&((obj->vertexes[j+2]-tolerance)<=vec[i+2])&&((obj->vertexes[j+2]+tolerance)>=vec[i+2]))
                break;
        if (j<obj->vertexes.size())
            obj->indexes.push_back(j/3);
        else
        {
            obj->vertexes.push_back(vec[i]);
            obj->vertexes.push_back(vec[i+1]);
            obj->vertexes.push_back(vec[i+2]);
            obj->indexes.push_back(count++);
        }
    }
    searchColor(ivmodel, obj);
}

void KauthamDEEnvironment::getTrimesh(SoSeparator *ivmodel, odinObject* obj, double scale)
{

    SoScale * sca = new SoScale();
    sca->scaleFactor.setValue((float)scale,(float)scale,(float)scale);
    ivmodel->addChild(sca);

    vector<double> vec;
    SoCallbackAction triAction;
    triAction.addTriangleCallback(SoShape::getClassTypeId(), triang_CB, &vec);
    triAction.apply(ivmodel);

    unsigned int count = 0;
    double tolerance = 0.00001;

    for (unsigned int i = 0; i < vec.size(); i+=3)
    {
        unsigned int j = 0;
        for (; j<obj->vertexes.size(); j+=3)
            if (((obj->vertexes[j]-tolerance)<=vec[i])&&((obj->vertexes[j]+tolerance)>=vec[i])
                    &&((obj->vertexes[j+1]-tolerance)<=vec[i+1])&&((obj->vertexes[j+1]+tolerance)>=vec[i+1])
                    &&((obj->vertexes[j+2]-tolerance)<=vec[i+2])&&((obj->vertexes[j+2]+tolerance)>=vec[i+2]))
                break;
        if (j<obj->vertexes.size())
            obj->indexes.push_back(j/3);
        else
        {
            obj->vertexes.push_back(vec[i]);
            obj->vertexes.push_back(vec[i+1]);
            obj->vertexes.push_back(vec[i+2]);
            obj->indexes.push_back(count++);
        }
    }
    //searchColor(ivmodel, obj);

}

void KauthamDEEnvironment::triang_CB(void* data,
                                     SoCallbackAction* action,
                                     const SoPrimitiveVertex* vertex1,
                                     const SoPrimitiveVertex* vertex2,
                                     const SoPrimitiveVertex* vertex3)
{
    vector<double>* verts = (vector<double>*)data;
    SbVec3f points[] = { vertex1->getPoint(), vertex2->getPoint(), vertex3->getPoint() };
    const SbMatrix  mm = action->getModelMatrix();
    for (int i = 0; i < 3; i++)
    {
        mm.multVecMatrix(points[i], points[i]);
        for (int j = 0; j < 3; j++)
            verts->push_back(points[i][j]);
    }
}
void KauthamDEEnvironment::searchColor(SoSeparator* root, trimeshD* obj)
{
    SoSearchAction search;

    search.setFind(SoSearchAction::TYPE);
    search.setSearchingAll(true);
    search.setInterest(SoSearchAction::LAST);

    SoType ID;

    ID = SoVRMLMaterial::getClassTypeId();
    search.setType(ID);
    search.apply(root);

    const SoPath * path = search.getPath();
    if (path != NULL)
    {
        cout << "Path length of SoVRMLmaterial "  << path->getLength() << '\n';
        SoVRMLMaterial * mat = (SoVRMLMaterial *)path->getTail();
        const SbColor col = mat->diffuseColor.getValue();
        for (int i = 0; i < 3; i++)
        {
            obj->color.push_back(col[i]);
            cout << " Colors of somaterial " << col[i] << '\n';
        }
    }
    else
    {
        ID = SoMaterial::getClassTypeId();
        search.setType(ID);
        search.apply(root);
        const SoPath * path = search.getPath();
        if (path != NULL)
        {
            cout << "Path length of somaterial "  << path->getLength() << '\n';
            SoMaterial * mat = (SoMaterial *)path->getTail();
            const SbColor col = mat->diffuseColor[0];
            for (int i = 0; i < 3; i++)
                obj->color.push_back(col[i]);
        }
    }
    return;
}
void KauthamDEEnvironment::searchColor(SoSeparator* root, odinObject* obj)
{
    SoSearchAction search;

    search.setFind(SoSearchAction::TYPE);
    search.setSearchingAll(true);
    search.setInterest(SoSearchAction::LAST);

    SoType ID;

    ID = SoVRMLMaterial::getClassTypeId();
    search.setType(ID);
    search.apply(root);

    const SoPath * path = search.getPath();
    if (path != NULL)
    {
        cout << "Path length of SoVRMLmaterial "  << path->getLength() << '\n';
        SoVRMLMaterial * mat = (SoVRMLMaterial *)path->getTail();
        const SbColor col = mat->diffuseColor.getValue();
        for (int i = 0; i < 3; i++)
        {
            obj->color.push_back(col[i]);
            cout << " Colors of somaterial " << col[i] << '\n';
        }
    }
    else
    {
        ID = SoMaterial::getClassTypeId();
        search.setType(ID);
        search.apply(root);
        const SoPath * path = search.getPath();
        if (path != NULL)
        {
            cout << "Path length of somaterial "  << path->getLength() << '\n';
            SoMaterial * mat = (SoMaterial *)path->getTail();
            const SbColor col = mat->diffuseColor[0];
            for (int i = 0; i < 3; i++)
                obj->color.push_back(col[i]);
        }
    }
    return;
}

bool KauthamDEEnvironment::getMotion(KauthamDEEnvironment::KinematicChain* chain,
                                     Link *link,
                                     odinObject* obj,
                                     string robotName,
                                     vector< double >& rotAxis)
{
    Joint joint;
    string jointID;

    if (!link->getMovable())
    {
        obj->fixed = true;

        return true;
    }
    else
    {
        obj->fixed = false;
        joint.target2 = obj->name;
        string parent = link->getParent()->getName();
        odinObject daddy = chain->objects[robotName + parent];
        joint.target1 = robotName + parent;
        jointID = joint.target1 + "&" + obj->name;
        std::cout<<"Body1 and Body2 are "<< joint.target1<<"  "<<joint.target2<<std::endl;
        Motor motor;
        motor.targetJoint = jointID;
        motor.fmax.push_back(dInfinity);
        chain->motors[jointID] = motor;
        cout << "ROTATIONAL JOINT BETWEEN " << link->getRotational() << std::endl;

        if (link->getRotational())
        {
            joint.hiStop = *(link->getLimits(false));// * toRad;
            joint.loStop = *(link->getLimits(true));// * toRad;
            joint.value=link->getValue();

            joint.type = 1;
                        cout << "ROTATIONAL JOINT BETWEEN " << joint.target1 << " AND " << joint.target2 << '\n';
            for (int i = 0; i < 3; i++)
                joint.position.push_back(obj->position[i]);
            for (int j = 0; j < 3; j++)
                joint.position.push_back(rotAxis.at(j));
            //chain->joints[jointID] = joint;
            std::cout<< "Joint value Position and rotation axis :"<< joint.value<<" , "<<joint.position[0]<<" , "<<joint.position[1]<<" , "<<joint.position[2]<<" , "<<joint.position[3]<<" , "<<joint.position[4]<<" , "<<joint.position[5]<<std::endl;
        }
        else //linear
        {
            joint.hiStop = *(link->getLimits(false));
            joint.loStop = *(link->getLimits(true));
            joint.type = 2;
            // 			cout << "LINEAR JOINT BETWEEN " << joint.target1 << " AND " << joint.target2 << '\n';
            for (int i = 0; i < 3; i++)
                joint.position.push_back(link->getAxis().at(i));
        }
        chain->joints[jointID] = joint;

        chain->jointsOrdered.push_back(joint);
    }

    return true;
}

bool KauthamDEEnvironment::getTransformation(KauthamDEEnvironment::KinematicChain* chain,
                                             Link* link,
                                             odinObject* obj,
                                             string robotName,
                                             vector<double>& rotAxis)
{

    //information for URDF model
    //to get the current transformation of the link
     // link->getTransformation()
    //to get the uper and lower limits
    //link->getLimits();
    //to get the current value to get angle in radian denormalize the value
    //link->getValue();
    //to get the information of the joint true is rotational false if slider
    //link->getRotational();
    //Building D-H Matrix

    //float *ori = link->getElement()->getOrientation();
     //mt::Rotation rot =link->getTransformation()->getRotation();
    // float *ori = link->getTransformation()->getRotation()
    //float *pose=link->getElement()->getPosition();
    //mt::Point3 pos = link->getTransformation()->getTranslation();



    mt::Transform absoluteTransform = *(link->getTransformation());
    mt::Rotation rot = absoluteTransform.getRotation();
    mt::Matrix3x3 rotMatrix = rot.getMatrix();
    obj->orientation.push_back(rot[3]);
    obj->orientation.push_back(rot[0]);
    obj->orientation.push_back(rot[1]);
    obj->orientation.push_back(rot[2]);


    mt::Point3 pos = absoluteTransform.getTranslation();
    obj->position.push_back(pos[0]);
    obj->position.push_back(pos[1]);
    obj->position.push_back(pos[2]);

    //joint axis of rotation in world frame
    mt::Point3 Xrot = link->getAxis()*rotMatrix;
    rotAxis.push_back(Xrot[0]);
    rotAxis.push_back(Xrot[1]);
    rotAxis.push_back(Xrot[2]);

////    rotAxis.push_back(link->getAxis()[0]);
////    rotAxis.push_back(link->getAxis()[1]);
////    rotAxis.push_back(link->getAxis()[2]);

   // rotAxis.push_back(rotMatrix[0][2]);
   // rotAxis.push_back(rotMatrix[1][2]);
   // rotAxis.push_back(rotMatrix[2][2]);
    std::cout<<"******************************************************************************"<<std::endl;
std::cout<<" Body Name and  position  "<<link->getName()<<" : "<<obj->position[0]<<" , "<<obj->position[1]<<" , "<<obj->position[2]<<std::endl;
std::cout<<" Body orientation from Link "<< obj->orientation[0]<<" , "<< obj->orientation[1]<<" , "<< obj->orientation[2]<< " , "<< obj->orientation[3]<<std::endl;
std::cout<<" Axis of rotation "<<rotAxis[0]<<" , "<<rotAxis[1]<<" , "<<rotAxis[2]<<std::endl;

//    obj->a = a;
//    obj->d = d;
//    obj->theta = theta;
//    obj->alpha = alpha;

    return true;
}

vector<KthReal> KauthamDEEnvironment::baseGetPos(Robot* robot)
{
    vector<KthReal> basePos;
    vector<KthReal> tmp;

    RobConf* RobC = robot->getCurrentPos();
    //RobConf* RobC=robot->getHomePos();
    SE3Conf SE3 = RobC->getSE3();
    tmp=SE3.getPos();
    basePos.push_back(tmp[0]);
    basePos.push_back(tmp[1]);
    basePos.push_back(tmp[2]);
    tmp.clear();
    tmp=SE3.getOrient();
    basePos.push_back(tmp[3]);
    basePos.push_back(tmp[0]);
    basePos.push_back(tmp[1]);
    basePos.push_back(tmp[2]);

//    std::cout<<"Base Pose : "<<basePos[0]<<" , "<<basePos[1]<<" , "<<basePos[2]<<std::endl;
//    std::cout<<"Base rot : "<<basePos[3]<<" , "<<basePos[4]<<" , "<<basePos[5]<<" , "<<basePos[6]<<std::endl;
   return basePos;


}

void KauthamDEEnvironment::makeGeomPrimitive(const string name, const vector<double>& position,
                                             const vector<double>& orientation, const vector<double>& params)
{
    dGeomID geometry;
    dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
    switch (params.size())
    {
    case 1:
        geometry = dCreateSphere(_OpenDEspace, params[0]);
        break;
    case 2:
        geometry = dCreateCylinder(_OpenDEspace, params[0], params[1]);
        break;
    case 3:
        geometry = dCreateBox(_OpenDEspace, params[0], params[1], params[2]);
        break;
    default:
        return;
    }
    dGeomSetPosition(geometry, position[0], position[1], position[2]);
    dGeomSetQuaternion(geometry, q);
    return;
}

dBodyID KauthamDEEnvironment::makePrimitive(const vector<double>& position, const vector<double>& orientation, const double mass, const vector<double>& params, string name)
{

    dBodyID body = dBodyCreate(bodyworld);
    //dBodyID body = dBodyCreate(_world);
    dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
    dBodySetPosition(body, position[0], position[1], position[2]);
    std::cout<<"Primitive Pos  "<<position[0]<<" , "<<position[1]<<" , "<<position[2]<<std::endl;
    dBodySetQuaternion(body, q);
    dGeomID geometry;
    dMass buildingMass;
    std::vector<double> dim;
    switch (params.size())
    {
    case 1:

        geometry = dCreateSphere(_OpenDEspace, params[0]);
        dMassSetSphereTotal(&buildingMass, mass, params[0]);
        dBodySetMass(body, &buildingMass);
        dGeomSetBody(geometry, body);
        dim.push_back(params[0]);

        break;
    case 2:
        geometry = dCreateCylinder(_OpenDEspace, params[0], params[1]);
        dMassSetCylinderTotal(&buildingMass, mass, 2, params[0], params[1]);
        dBodySetMass(body, &buildingMass);
        dGeomSetBody(geometry, body);
        dim.push_back(params[0]);
        dim.push_back(params[1]);

//        dQFromAxisAndAngle(q,1,0,0,1.57);
//        dBodySetQuaternion(body,q);
//        dGeomSetQuaternion(geometry,q);
        break;
    case 3:
        geometry = dCreateBox(_OpenDEspace, params[0], params[1], params[2]);
        dMassSetBoxTotal(&buildingMass, mass, params[0], params[1], params[2]);
        dBodySetMass(body, &buildingMass);
        dGeomSetBody(geometry, body);
        dim.push_back(params[0]);
        dim.push_back(params[1]);
        dim.push_back(params[2]);
        break;
    default:
        return NULL;
    }

    bodyDim.insert(pair<dBodyID,std::vector<double> >(body,params));


    std::vector<double> dimR;
    std::vector<string> rigionDir;
    rigionDir.push_back("x");
    dimR.push_back(4);
    dimR.push_back(4);

    bodyInfo kb;
    kb.bodyName=name;
    kb.body=body;
    kb.Geom=geometry;
    kb.mass=mass;
    kb.dimBody=bodyDim[body];
    kb.dimRegion=dimR;
    kb.regionDir=rigionDir;
    string type=InstknowledgeInference(kb);
    geomname.insert(pair<dGeomID,std::string>(geometry,type));
    GeomID.push_back(geometry);

    return body;
}

void KauthamDEEnvironment::makePrimitiveForComposite(const vector< double >& position,
                                                     const vector< double >& orientation,
                                                     const double mass,
                                                     const vector< double > params,
                                                     const dBodyID coreBody)
{
    dGeomID geometry;
    dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
    dMass attachedMass;
    switch (params.size())
    {
    case 1:
        geometry = dCreateSphere(_OpenDEspace, params[0]);
        dMassSetSphereTotal(&attachedMass, mass, params[0]);
        break;
    case 2:
        geometry = dCreateCylinder(_OpenDEspace, params[0], params[1]);
        dMassSetCylinderTotal(&attachedMass, mass, 2, params[0], params[1]);
        break;
    case 3:
        geometry = dCreateBox(_OpenDEspace, params[0], params[1], params[2]);
        dMassSetBoxTotal(&attachedMass, mass, params[0], params[1], params[2]);
        break;
    default:
        return;
    }
    dGeomSetBody(geometry, coreBody);
    dGeomSetOffsetWorldPosition(geometry, position[0], position[1], position[2]);
    dGeomSetOffsetWorldQuaternion(geometry, q);
    const dReal* p = dGeomGetOffsetPosition(geometry);
    dMassTranslate(&attachedMass, p[0], p[1], p[2]);
    const dReal* rot = dGeomGetOffsetRotation(geometry);
    dMassRotate(&attachedMass, rot);
    dMass finalMass;
    dBodyGetMass(coreBody, &finalMass);
    dMassAdd(&finalMass, &attachedMass);
    dBodySetMass(coreBody, &finalMass);
}

void KauthamDEEnvironment::makeGeomTrimesh(const string name, const vector<double>& position, const vector<double>& orientation,
                                           const vector<double>& vertexes, const vector<unsigned int>& indexes)
{
    //Building trimesh data.
    float * vrtxs = new float[vertexes.size()];
    for (unsigned int i = 0; i < (vertexes.size()); i++)
        vrtxs[i] = ((float)vertexes[i]);

    dTriIndex * trindexes = new dTriIndex[indexes.size()];
    for (unsigned int j = 0; j < (indexes.size()); j++)
        trindexes[j] = indexes[j];

    dTriMeshDataID data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(data, vrtxs, 3*sizeof(float), (int)vertexes.size(), trindexes, indexes.size(), 3*sizeof(unsigned int));
    dGeomID geometry = dCreateTriMesh(_OpenDEspace, data, NULL, NULL, NULL);

    dGeomSetPosition(geometry, position[0], position[1], position[2]);
    dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
    dGeomSetQuaternion(geometry, q);
    return;
}

dBodyID KauthamDEEnvironment::makeTriMesh(vector<double> position, vector<double> orientation,
                                          vector<double> vertexes, vector<unsigned int> indexes, double mass, string name, std::vector<double> bodydim, std::vector<float> color)
{
    Tmesh MeSh;
    dMass buildingMass;
    //body = NULL;
    dBodyID body = dBodyCreate(bodyworld);
    //dBodyID body = dBodyCreate(_world);
    //Building trimesh data.
    float * vrtxs = new float[vertexes.size()];
    for (unsigned int i = 0; i < (vertexes.size()); i++)
       {
        vrtxs[i] = ((float)vertexes[i]);
        MeSh.vertices.push_back(((float)vertexes[i]));
        }
    dTriIndex * trindexes = new dTriIndex[indexes.size()];
    for (unsigned int j = 0; j < (indexes.size()); j++)
    {
        trindexes[j] = indexes[j];
        MeSh.indices.push_back(indexes[j]);
    }
    dTriMeshDataID data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(data, vrtxs, 3*sizeof(float), (int)vertexes.size(), trindexes, indexes.size(), 3*sizeof(unsigned int));
    //dGeomTrimeshDataBuildSingle(data,vrtxs,(int)vertexes.size(),trindexes,indexes.size());
    dGeomID geometry = dCreateTriMesh(_OpenDEspace, data, NULL, NULL, NULL);
    dGeomSetData(geometry, data);
    MeSh.indexSize=indexes.size();
    MeSh.meshD=data;
    MeSh.color=color;
    meshID.push_back(MeSh);
    dMassSetTrimeshTotal(&buildingMass, mass, geometry);
    std::cout<<"Mass is  "<<buildingMass.mass<<std::endl;
    dGeomSetPosition(geometry, -buildingMass.c[0], -buildingMass.c[1], -buildingMass.c[2]);
    dMassTranslate(&buildingMass, -buildingMass.c[0], -buildingMass.c[1], -buildingMass.c[2]);

    dBodySetMass(body, &buildingMass);
    dGeomSetBody(geometry, body);

    dBodySetPosition(body, position[0], position[1], position[2]);
    dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
    dBodySetQuaternion(body, q);
    dGeomSetPosition(geometry, position[0], position[1], position[2]);
    dGeomSetQuaternion(geometry, q);

    //std::cout<< "Mass of Body is" <<buildingMass.mass<<std::endl;
    bodyDim.insert(pair<dBodyID,std::vector<double> >(body,bodydim));

    std::vector<double> dimR;
    std::vector<string> rigionDir;
    rigionDir.push_back("y");
    rigionDir.push_back("-y");

    dimR.push_back(3);
    dimR.push_back(3);

    bodyInfo kb;
    kb.bodyName=name;
    kb.body=body;
    kb.Geom=geometry;
    kb.mass=mass;
    kb.dimBody=bodyDim[body];
    kb.dimRegion=dimR;
    kb.regionDir=rigionDir;
    string type=InstknowledgeInference(kb);
    geomname.insert(pair<dGeomID,std::string>(geometry,type));
    GeomID.push_back(geometry);


    //string typ=InferenceProcess(name,body,geometry);

    //geomname.insert(pair<dGeomID,std::string>(geometry,typ));
    return body;
}
/*dJointID KauthamDEEnvironment::makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params)
 {
     dJointID joint;
              switch (type)
              {
      //          Joint has to be attatched after it is created, otherwise strange things happen.
                  case ball:
                      joint = dJointCreateBall(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetBallAnchor(joint, params[0], params[1], params[2]);
                      break;
                  case hinge:
                      joint = dJointCreateHinge(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetHingeAnchor (joint, params[0], params[1], params[2]);
                      dJointSetHingeAxis (joint, params[3], params[4], params[5]);
                      break;
                  case slider:
                      joint = dJointCreateSlider(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetSliderAxis(joint, params[0], params[1], params[2]);
                      break;
                  case universal:
                      joint = dJointCreateUniversal(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetUniversalAnchor(joint, params[0], params[1], params[2]);
                      dJointSetUniversalAxis1(joint, params[3], params[4], params[5]);
                      dJointSetUniversalAxis2(joint, params[6], params[7], params[8]);
                      break;
                  case hinge2:
                      joint = dJointCreateHinge2(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetHinge2Anchor(joint, params[0], params[1], params[2]);
                      dJointSetHinge2Axis1(joint, params[3], params[4], params[5]);
                      dJointSetHinge2Axis2(joint, params[6], params[7], params[8]);
                      break;
                  case prismatic_rotoide:
                      joint = dJointCreatePR(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetPRAnchor(joint, params[0], params[1], params[2]);
                      dJointSetPRAxis1(joint, params[3], params[4], params[5]);
                      dJointSetPRAxis2(joint, params[6], params[7], params[8]);
                      break;
                  case prismatic_universal:
                      joint = dJointCreatePU(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetPUAxis1(joint, params[0], params[1], params[2]);
                      dJointSetPUAxis2(joint, params[3], params[4], params[5]);
                      dJointSetPUAxisP(joint, params[6], params[7], params[8]);
                      break;
                  case piston:
                      joint = dJointCreatePiston(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetPistonAnchor(joint, params[0], params[1], params[2]);
                      dJointSetPistonAxis(joint, params[3], params[4], params[5]);
                      break;
                  case plane:
                      joint = dJointCreatePlane2D(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      break;
                  case fixed:
                      joint = dJointCreateFixed(_OpenDEWorld, 0);
                      dJointAttach(joint, body1, body2);
                      break;
                  default:
                      joint = NULL;
              }
              return joint;
 }*/

dJointID KauthamDEEnvironment::makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params,vector<double>& maxForces,const double hiStop, const double LoStop,const double value)
{
    KthReal min_Threshold = 0.800;
    KthReal max_Threshold = 1.2;
    std::vector<int> rotation_Axis;
    rotation_Axis.resize(3);
    dJointID joint;

             switch (type)
             {
     //Joint has to be attatched after it is created, otherwise strange things happen.

                 case 1:
                     joint = dJointCreateHinge(bodyworld, 0);
                     dJointAttach(joint, body1, body2);
                    std::cout<<"ODE Joint Anchor is :"<<bodyDim[body2][2]/2<<std::endl;
                     dJointSetHingeAnchor (joint, params[0], params[1], params[2]);//-bodyDim[body2][2]/2));
                     dVector3 result;
                     dJointGetHingeAnchor(joint,result);
                     rotation_Axis[0]=((params[3]>min_Threshold && params[3]<max_Threshold)? 1:0);
                     rotation_Axis[1]=((params[4]>min_Threshold && params[4]<max_Threshold)? 1:0);
                     rotation_Axis[2]=((params[5]>min_Threshold && params[5]<max_Threshold)? 1:0);
                     //dJointSetHingeAxis (joint, rotation_Axis[0],rotation_Axis[1],rotation_Axis[2]);
                     dJointSetHingeAxis (joint, params[3],params[4],params[5]);
                     dJointSetHingeParam (joint, dParamHiStop, hiStop);
                     dJointSetHingeParam (joint, dParamLoStop, LoStop);

                     std::cout<<"Joint Anchor: "<<result[0]<<" , "<< result[1]<<" , "<<result[2]<<std::endl;
                     std::cout<<"*****Joint Axis of rotation : "<<params[3]<<" , "<< params[4]<<" , "<<params[5]<<std::endl;
                     std::cout<<"Joint lower and uper limits are : "<<LoStop<<" , "<< hiStop<<std::endl;


                     break;
                 case 2:
                     joint = dJointCreateSlider(bodyworld, 0);
                     dJointAttach(joint, body1, body2);
                     dJointSetSliderAxis(joint, params[0], params[1], params[2]);
                     break;

             }
      return joint;


}




dJointID KauthamDEEnvironment::addMotor2Joint(dJointID joint,  vector<double>& maxForces,const double value, const double hiStop, const double LoStop)
{
    dVector3 axis;
    dBodyID body1 = dJointGetBody(joint, 0);
    dBodyID body2 = dJointGetBody(joint, 1);
    unsigned int type;

    switch (dJointGetType(joint))
    {
        case dJointTypeHinge:
            dJointGetHingeAxis(joint, axis);

            type = 0;
            break;
        case dJointTypeSlider:
            dJointGetSliderAxis(joint, axis);
            type = 1;
            break;
        default:
            //return false;
           break;
    }

    vector<double> axisVec;
    axisVec.push_back(axis[0]);
    axisVec.push_back(axis[1]);
    axisVec.push_back(axis[2]);


    /*vector<double> magnitude;
    magnitude.push_back(0);*/

    //addMotor(findBodyName(body1), findBodyName(body2), type, axisVec, maxForces, set, value, id);
  //  makeMotor(body1,body2,type,axisVec,maxForces);
  return  makeMotor(body1,body2,type,axisVec,maxForces, hiStop,LoStop,value);


}
dJointID KauthamDEEnvironment::makeMotor(dBodyID body1,dBodyID body2,const unsigned int type,
                                     const vector< double >& axes,const vector< double >& fmax, const double hiStop, const double LoStop, const double value )
{

    dJointID motor;
         int numAxes = (int)(axes.size()/3);
         switch (type)
         {
             case 0:
             {

                 motor = dJointCreateAMotor(bodyworld, 0);
                 //motor = dJointCreateAMotor(_world, 0);
                 dJointAttach(motor, body1, body2);
                 dJointSetAMotorNumAxes(motor, numAxes);
                 std::cout<<"Num Axis  "<<numAxes<<std::endl;

              /*   Set the AMotor axes. The anum argument selects the axis to change (0,1 or 2).
               *   Each axis can have one of three "relative orientation" modes, selected by rel:
                     0: The axis is anchored to the global frame.
                     1: The axis is anchored to the first body.
                     2: The axis is anchored to the second body. */

                 dJointSetAMotorAxis(motor,/*anum*/ 0, /*rel*/ 1, axes[0], axes[1], axes[2]);
     //          dJointSetAMotorParam(motor, dParamFMax, dInfinity);
                 dJointSetAMotorParam(motor, dParamHiStop, hiStop);
                 dJointSetAMotorParam(motor, dParamLoStop, LoStop);
                 dJointSetAMotorParam(motor, dParamFMax, dInfinity);
     //          dReal angle= value*(hiStop-LoStop)+LoStop;
     //          dJointSetAMotorAngle(motor,0,angle);
     //          std::cout<<"normalized value and computed angles are  "<<value<<" , "<< angle<<std::endl;

                 if (numAxes > 1)
                 {
                     dJointSetAMotorAxis(motor, 1, 1, axes[3], axes[4], axes[5]);
                     dJointSetAMotorParam(motor, dParamFMax2, fmax[1]);
                     dJointSetAMotorParam(motor, dParamVel2, 0);
                     if (numAxes > 2)
                     {
                         dJointSetAMotorAxis(motor, 2, 1, axes[6], axes[7], axes[8]);
                         dJointSetAMotorParam(motor, dParamFMax3, fmax[2]);
                         dJointSetAMotorParam(motor, dParamVel3, 0);

                     }
                 }
             }
             break;
             case 1:
             {
                 motor = dJointCreateLMotor(bodyworld, 0);
                 dJointAttach(motor, body1, body2);
                 dJointSetLMotorNumAxes(motor, numAxes);
                 dJointSetLMotorAxis(motor, 0, 1, axes[0], axes[1], axes[2]);
                 dJointSetLMotorParam(motor, dParamFMax, fmax[0]);
                 dJointSetLMotorParam(motor, dParamVel, 0);
                 if (numAxes > 6)
                 {
                     cout << "Setting second axis\n";
                     dJointSetLMotorAxis(motor, 1, 1, axes[3], axes[4], axes[5]);
                     dJointSetLMotorParam(motor, dParamFMax, fmax[1]);
                     dJointSetLMotorParam(motor, dParamVel2, 0);
                     cout << "Second axis set\n";
                     if (numAxes > 11)
                     {
                         cout << "Setting third axis\n";
                         dJointSetLMotorAxis(motor, 2, 1, axes[6], axes[7], axes[8]);
                         dJointSetLMotorParam(motor, dParamFMax, fmax[2]);
                         dJointSetLMotorParam(motor, dParamVel3, 0);
                     }
                     cout << "Third axis set\n";
                 }
             }
             break;
             default:
                 motor = NULL;
         }



    _motor.push_back(motor);

return motor;
}

}

};

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
