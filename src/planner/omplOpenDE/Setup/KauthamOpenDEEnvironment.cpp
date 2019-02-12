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
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <boost/bind/mem_fn.hpp>
#include <kautham/planner/omplOpenDE/Setup/KauthamOpenDEEnvironment.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>

#include <kautham/util/kthutil/kauthamdefs.h>


using namespace std;

namespace Kautham {

namespace omplcplanner{

//! The KauthamDEEVironment constructor will creat a chainMap that will capture the kinamatic structure of the world
//! (robots and objects) along with their dynamic properties such as masses, joint limits bounds on velocities and
//! torques. Then this chainMap is used to create the ODE world. Moreover constructor also create the instantiated
//! knowledge instance to define the manipulation constraints for the objects.

KauthamDEEnvironment::KauthamDEEnvironment(WorkSpace *wkspace, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm,bool isKchain): oc::OpenDEEnvironment()
{

    (void) isKchain; //unused
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
        defaulttorque.push_back(1.5);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(1.5);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(1.5);//+(1.5*0.7*9.8*1));
        defaulttorque.push_back(1.5);//+(1.5*0.7*9.8*1));
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
//! The createWorld function creates odebody by using information obtained from the chainMap.
void KauthamDEEnvironment::createWorld(WorkSpace *wkspace)
{

    bodyworld=dWorldCreate();
    _OpenDEspace = dHashSpaceCreate(0);
    dWorldSetGravity(bodyworld, 0,0,-9.8);
    dWorldSetCFM (bodyworld, 1e-5);
    dWorldSetERP (bodyworld, 0.8);

    if(wkspace->getRobot(0)->getNumJoints()>1 && wkspace->getRobot(0)->getName()!="SimpleCar")
        trimesh=true;

    for (int i=0;i < (int(wkspace->getNumRobots())); i++)
    {
        for (unsigned int j=0;j< (wkspace->getRobot(i)->getNumLinks()); j++)
        {

            if(! trimesh)
            {
                dBodyID odebody;
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
                const vector<double> position= chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position;
                const vector<double> orientation=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation;
                const vector<double> vertexes=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes;
                const vector<unsigned int> indexes = chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].indexes;
                const vector<double> bodyDimension=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].bodydimension;
                const vector<float> color=chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].color;
                double mass= wkspace->getRobot(i)->getLink(j)->getOde().inertial.mass;
                std::string name = (wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName());
                std::cout<<"Name of Rob Part: " <<name<<std::endl;

                odebody = makeTriMesh(position,orientation,vertexes,indexes,mass,name,bodyDimension, color);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())),odebody));
                dBodySetGravityMode (odebody,0);
                bodies.push_back(odebody);
            }
        }

    }
    for (int i=0;i < (int(wkspace->getNumObstacles()));i++)
    {
        for (unsigned int j=0;j < (wkspace->getObstacle(i)->getNumLinks()); j++)
        {
            if(! trimesh)
            {

                dBodyID odebody;
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
                double mass=wkspace->getObstacle(i)->getLink(j)->getOde().inertial.mass;
                std::string name = wkspace->getObstacle(i)->getName();
                if(mass==0)
                    mass=1;
                odebody = makeTriMesh(position,orientation,vertexes,indexes,mass,name,bodyDimension,color);
                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())),odebody));
                std::cout<<"Obs body name"<<((wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName()))<<std::endl;
                bodies.push_back(odebody);
                getBody.insert(pair<string,dBodyID>(wkspace->getObstacle(i)->getName(),odebody));

            }
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

//! InstknowledgeInference function computes the manipulation regions and attached to the associated bodies along with the other
//! knowledge related information.
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
                                if(bodyinfo.bodyName == "box2"||bodyinfo.bodyName == "box3"||bodyinfo.bodyName == "box4"||bodyinfo.bodyName == "box5")
                                { //bodyinfo.bodyName == "box1"||
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
//! set the planning parameters such as collision space, step size, control steps range for the ODE world.
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
//!this function add joints and motors between ode bodies.
void KauthamDEEnvironment::setjointsandmotors2bodies(map<string, dBodyID> stateBodiesmap_,map<string, KinematicChain> chainMap, WorkSpace* wkspace)
{
    for (map<string,Joint>::iterator it = chainMap[wkspace->getRobot(0)->getName()].joints.begin(); it!=chainMap[wkspace->getRobot(0)->getName()].joints.end(); ++it)
    {
        dBodyID body1 = stateBodiesmap_[it->second.target1];
        dBodyID body2 = stateBodiesmap_[it->second.target2];
        std::cout<<"Target is "<<it->second.target1+it->second.target2<<std::endl;
        dJointID joint = makeJoint(body1,body2,it->second.type,it->second.position, chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].hiStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].loStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].value);
        dJointID motor = addMotor2Joint(joint,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax, it->second.value, chainMap[wkspace->getRobot(0)->getName()].joints[it->first].hiStop,chainMap[wkspace->getRobot(0)->getName()].joints[it->first].loStop);
        _Joint.push_back(joint);
        joint_.insert(pair<std::string,dJointID>(it->second.target1+"+"+it->second.target2,joint));
        motor_.insert(pair<std::string,dJointID>(it->second.target1+"+"+it->second.target2,motor));


    }

}
//!This function fills a vector of bodies depending on the order of the bodies i state body map.
vector<dBodyID> KauthamDEEnvironment::fillstatebodies(map<string,dBodyID> stateBodiesmap_,WorkSpace *wkspace)
{
    vector<dBodyID> bodies;
    if(wkspace->getNumRobots()<1)
    {
        for(map<string,dBodyID>::iterator it = stateBodiesmap_.begin(); it!=stateBodiesmap_.end(); ++it)
        {
            bodies.push_back(it->second);
        }
    }
    else
    {
        if(wkspace->getRobot(0)->getNumLinks()==1)
        {
            for (unsigned int i=0; i < wkspace->getNumRobots(); i++)
            {
                bodies.push_back(stateBodiesmap_[(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(0)->getName())]);
                std::cout<<(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(0)->getName())<<std::endl;
            }
            for (unsigned int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (unsigned int j=0; j < (wkspace->getObstacle(i)->getNumLinks()); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()]);
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
                }
            }
            for (unsigned int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (unsigned int j=0; j < wkspace->getObstacle(i)->getNumLinks(); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()]);
                }
            }
        }
    }
    return bodies;
}
//! This function reads the kautham workspace are build a kinamatic chain to create the ode world
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
//!This function generate the information to create the primitive shapes for ODE.
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
                //SoTranslation *trans = (SoTranslation*)node;
                //const SbVec3f pose = trans->translation.getValue();
            }
            else if (node->getTypeId() == SoRotation::getClassTypeId())
            {
                if(rotflag)
                {
                   // SoRotation *rot = (SoRotation*)node;
                    //const SbRotation &rotation = rot->rotation.getValue();
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
            }
            else
                std::cout<<" Type "<< node->getTypeId().getName()<<std::endl;
        }

    }
}
//!getTrimesh function generates the trimesh to build the ode bodies.
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
//!getTrimesh function generates the trimesh to build the ode bodies.
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
//!search the color of the trimesh material
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
//!search the color of the trimesh material
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
            cout << " Colors of somaterial " << col[0]<<" "<<col[1] << '\n';
        }
    }
    return;
}
//! getMotion function create the joint data for ODE such as upper and lower limits, attached bodies, rotation axis
//! and max force.
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
        joint.target1 = robotName + parent;
        jointID = joint.target1 + "&" + obj->name;
        // std::cout<<"Body1 and Body2 are "<< joint.target1<<"  "<<joint.target2<<std::endl;
        Motor motor;
        motor.targetJoint = jointID;
        motor.fmax.push_back(dInfinity);
        chain->motors[jointID] = motor;
        //cout << "ROTATIONAL JOINT BETWEEN " << link->getRotational() << std::endl;

        if (link->getRotational())
        {
            joint.hiStop = *(link->getLimits(false));// * toRad;
            joint.loStop = *(link->getLimits(true));// * toRad;
            joint.value=link->getValue();

            joint.type = 1;
            //  cout << "ROTATIONAL JOINT BETWEEN " << joint.target1 << " AND " << joint.target2 << '\n';
            for (int i = 0; i < 3; i++)
                joint.position.push_back(obj->position[i]);
            for (int j = 0; j < 3; j++)
                joint.position.push_back(rotAxis.at(j));
            //chain->joints[jointID] = joint;
            //std::cout<< "Joint value Position and rotation axis :"<< joint.value<<" , "<<joint.position[0]<<" , "<<joint.position[1]<<" , "<<joint.position[2]<<" , "<<joint.position[3]<<" , "<<joint.position[4]<<" , "<<joint.position[5]<<std::endl;
        }
        else //linear
        {
            joint.hiStop = *(link->getLimits(false));
            joint.loStop = *(link->getLimits(true));
            joint.type = 2;
            for (int i = 0; i < 3; i++)
                joint.position.push_back(link->getAxis().at(i));
        }
        chain->joints[jointID] = joint;

        chain->jointsOrdered.push_back(joint);
    }

    return true;
}
//!getTransformation function computes the transformation between the links
bool KauthamDEEnvironment::getTransformation(KauthamDEEnvironment::KinematicChain* chain,
                                             Link* link,
                                             odinObject* obj,
                                             string robotName,
                                             vector<double>& rotAxis)
{

    (void) robotName; //unused
    (void) chain; //unused
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
    mt::Point3 Xrot = rotMatrix*link->getAxis();
    rotAxis.push_back(Xrot[0]);
    rotAxis.push_back(Xrot[1]);
    rotAxis.push_back(Xrot[2]);

    return true;
}
//!baseGetPos function obtain the base pose of the robot and the objects for ODE world.
vector<KthReal> KauthamDEEnvironment::baseGetPos(Robot* robot)
{
    vector<KthReal> basePos;
    vector<KthReal> tmp;

    RobConf* RobC = robot->getCurrentPos();
    // RobConf* RobC=robot->getHomePos();
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
    return basePos;
}
//!makeGeomPrimitive function create the ODE geometries using primitive shapes such as sphere, box and cylinder.
void KauthamDEEnvironment::makeGeomPrimitive(const string name, const vector<double>& position,
                                             const vector<double>& orientation, const vector<double>& params)
{
    (void) name;//unused

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
//!makePrimitive this function creates ODE bodies using primitive shapes such as box, cylinder and sphere. Moreover it associate the
//! ode bodies with the ode geometries and fill the masses.
dBodyID KauthamDEEnvironment::makePrimitive(const vector<double>& position, const vector<double>& orientation, const double mass, const vector<double>& params, string name)
{

    dBodyID body = dBodyCreate(bodyworld);
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
//!makePrimitiveForCompositethis functions creates composite ode shapes using primitive shapes.
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
//!makeGeomTrimesh this function make ode geometries using triangular meshes.
void KauthamDEEnvironment::makeGeomTrimesh(const string name, const vector<double>& position, const vector<double>& orientation,
                                           const vector<double>& vertexes, const vector<unsigned int>& indexes)
{
    (void) name;//unused

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
//! makeTriMesh this function makes ode bosies using triangular meshes, associate them to the ode geometries and fill the masses.
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
    return body;
}
//!makeJoint this function makes ode joints and and set their properties.
dJointID KauthamDEEnvironment::makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params,vector<double>& maxForces,const double hiStop, const double LoStop,const double value)
{
    (void) maxForces; //unused
    (void) value; //unused

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
        dJointSetHingeAxis (joint, params[3],params[4],params[5]);
        dJointSetHingeParam (joint, dParamHiStop, hiStop);
        dJointSetHingeParam (joint, dParamLoStop, LoStop);

        break;
    case 2:
        joint = dJointCreateSlider(bodyworld, 0);
        dJointAttach(joint, body1, body2);
        dJointSetSliderAxis(joint, params[0], params[1], params[2]);
        break;

    }
    return joint;


}
//! addMotor2Joint this function add motors to the joints and set there parameters for ode.
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
    return  makeMotor(body1,body2,type,axisVec,maxForces, hiStop,LoStop,value);


}
//! makeMotor this function makes motors such as angular motor and linear motor for ode joints.
dJointID KauthamDEEnvironment::makeMotor(dBodyID body1,dBodyID body2,const unsigned int type,
                                         const vector< double >& axes,const vector< double >& fmax, const double hiStop, const double LoStop, const double value )
{

    (void) value;//unused

    dJointID motor;
    int numAxes = (int)(axes.size()/3);
    switch (type)
    {
    case 0:
    {
        motor = dJointCreateAMotor(bodyworld, 0);
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
            dJointSetLMotorAxis(motor, 1, 1, axes[3], axes[4], axes[5]);
            dJointSetLMotorParam(motor, dParamFMax, fmax[1]);
            dJointSetLMotorParam(motor, dParamVel2, 0);
            if (numAxes > 11)
            {
                dJointSetLMotorAxis(motor, 2, 1, axes[6], axes[7], axes[8]);
                dJointSetLMotorParam(motor, dParamFMax, fmax[2]);
                dJointSetLMotorParam(motor, dParamVel3, 0);
            }
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

}

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL
