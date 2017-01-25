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

#include <kautham/problem/workspace.h>
#include <problem/ivpqpelement.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplOpenDE/KauthamOpenDEEnvironment.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>


#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>


using namespace std;

#define SCALE 1


namespace Kautham {
//namespace libPlanner {

namespace omplcplanner{

    //Quan es crea l'Environment es crida el contructor. Es reserva una chain (KinematicChain) que un cop s'ompli per via de la crida dels metodes buildKinematicChain, es posaran dintre del chainMap mapejats per nom de cadena.
    //Els buildKinematicChain vindria a ser la manera com el Alfredo amb ODIN llegia els documents xml , aquí com Kautham ja llegeix la escena abans que res copiem aquesta informació del workspace sense haver de preocuparnos pels documents xml.
    //El constructor pass per tots els cossos de la escena (workspace) i va llegint la seva informacio ( posició , orientacio , vertexs, etc...)
    //Un cop s'ha omplert chainMap es crida el mètode worldCreate.
    //! Constructor

     // When you create the Environment calling the contructor. Reserves a chain (KinematicChain) that once filled by way of calling methods buildKinematicChain, will be within the chainMap mapped by name string.
     // The buildKinematicChain would be the way Alfredo with ODIN parsing XML documents, as here Kautham already read the scene above all this information copy of the workspace without preocuparnos for XML documents.
     // Constructor to pass all the bodies of the scene (workspace) and reading their information (position, orientation, vertices, etc ...)
     // Once filled chainMap worldCreate method is called.
     //! Constructor

KauthamDEEnvironment::KauthamDEEnvironment(WorkSpace *wkspace, KthReal maxspeed): oc::OpenDEEnvironment()
  {
      KinematicChain* chain(new KinematicChain);
      vector<KthReal> basePos;
      _NumLinksFirstRobot=wkspace->getRobot(0)->getNumLinks();
      _maxspeed=maxspeed;


  for (int i=0; i < wkspace->getNumRobots(); i++)
  {

      basePos = baseGetPos(wkspace->getRobot(i));
      buildKinematicChain(chain, wkspace->getRobot(i),SCALE,basePos);

      chainMap.insert(pair<string,KinematicChain>(chain->name,*chain));
      chain->objects.clear();
      chain->joints.clear();
      chain->jointsOrdered.clear();
      chain->motors.clear();

  }
  for (int i=0; i < wkspace->getNumObstacles(); i++)
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
/*  dVector3 pos;
  dBodyCopyPosition ( stateBodies_[0], pos);
  dVector3 pos1;
  dBodyCopyPosition ( stateBodies_[1], pos1);
  dVector3 pos2;
  dBodyCopyPosition ( stateBodies_[2], pos2);
  const dReal* l=dBodyGetPosition(stateBodies_[0]);
  /*int u;
  u=0;*/
  chainMap.clear();
  stateBodiesmap_.clear();

}

//A createWorld es va creant constantment un odebody que mitjançant la funció makeTriMesh o makePrimitive omple les seves estructures de dades a partir de la informació que extreu del mapa chainMap.
//Segons diu Alfredo en aquest cas mai usariem la funció makePrimitive però ho he deixat perquè es vegi que s'ha tingut en compte per si en algun moment es considera necessari utilitzar-ho.
//Un cop creat cada body es passa a stateBodiesMap de bodys amb el mateix nom que aquella estructura tenia en el chainMap.
//*******makeTriMesh no funciona del tot bé per això té una part comentada ja que no funciona correctament la manera de determinar la massa dels dBodys.
//Desde createWorld es crida els mètodes fillstatebodies---> només omple el vector StateBodies_ a partir dels cossos de stateBodiesmap_ fent que els primers objectes siguin els del robot a planificar.
//també es crida el setJointsandMotors2bodies del qual no estic 100% segur del seu funcionament.

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
   // dWorldSetGravity(bodyworld, 0,0,-9.8);
      dWorldSetCFM (bodyworld, 1e-5);
      dWorldSetERP (bodyworld, 0.8);
      dWorldSetQuickStepNumIterations (bodyworld,20);

      //ground = dCreatePlane (_OpenDEspace,0,0,1,0);
//_OpenDEspace =dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );
    for (int i=0;i < (int(wkspace->getNumRobots())); i++)
    {
        for (int j=0;j< (wkspace->getRobot(i)->getNumLinks()); j++)
        {
            //intent de deduir que un objecte es un Primitive i no es un trimesh segons la dimensio del seu vector de vertex, en odeserver::buildObject aquesta decisio es pren segons req.parameters que te pinta de ser equivalent als vertexs
            // attempt to infer that an object is a Primitive and not a trimesh depending on the size of your vector vertex in odeserver :: buildObject req.parameters as this decision is that you seems to be equivalent to the vertices
            if (chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes.size() < 0)
            {

                dBodyID odebody;
                odebody = makePrimitive(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes);

                stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())),odebody));

            }
            else
            {
                 dBodyID odebody;
                 odebody = makeTriMesh(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].indexes,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass);

                 stateBodiesmap_.insert(pair<string,dBodyID>(((wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())),odebody));
                 bodies.push_back(odebody);
            }
        }

    }
    for (int i=0;i < (int(wkspace->getNumObstacles()));i++)
    {
        for (int j=0;j < (wkspace->getObstacle(i)->getNumLinks()); j++)
        {
            //intent de deduir que un objecte es un Primitive i no es un trimesh segons la dimensio del seu vector de vertex
            if (chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes.size() <= 3)
            {
                dBodyID odebody;
                odebody = makePrimitive(chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].position,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].mass,chainMap[wkspace->getRobot(i)->getName()].objects[(wkspace->getRobot(i)->getName())+(wkspace->getRobot(i)->getLink(j)->getName())].vertexes);
                stateBodiesmap_.insert(pair<string,dBodyID>((wkspace->getRobot(i)->getName()),odebody));

            }
            else
            {
                 dBodyID odebody;

                           /*   if(i==0)
                                {
                                vector<double> t;
                                t.push_back(-90);
                                 t.push_back(-90);
                                  t.push_back(50);
                                    const vector<double> position= t;
                                    const vector<double> orientation=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation;
                                    const vector<double> vertexes=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes;
                                    const vector<unsigned int> indexes = chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes;
                                    //double mass=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass;
                                    double mass=0.01;

                                     // odebody = makeTriMesh(chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass);
                                      odebody = makeTriMesh(position,orientation,vertexes,indexes,mass);
                                      //dVector3 pos;
                                      //dBodyCopyPosition ( odebody, pos);
                                      bodies.push_back(odebody);
                                }
                                else*/
                                {

               const vector<double> position= chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position;
               const vector<double> orientation=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation;
               const vector<double> vertexes=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes;
               const vector<unsigned int> indexes = chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes;
               double mass=chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass;
               // odebody = makeTriMesh(chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].position,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].orientation,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].vertexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].indexes,chainMap[wkspace->getObstacle(i)->getName()].objects[(wkspace->getObstacle(i)->getName())+(wkspace->getObstacle(i)->getLink(j)->getName())].mass);
               odebody = makeTriMesh(position,orientation,vertexes,indexes,mass);
               //dVector3 pos;
               //dBodyCopyPosition ( odebody, pos);
                 bodies.push_back(odebody);
                }
                 //stateBodiesmap_.insert(pair<string,dBodyID>((wkspace->getRobot(i)->getName()),odebody));

                 // dBodyDestroy(odebody);
            }
        }

    }
   /* for (int i=0;i < (int(wkspace->getNumObstacles()));i++)
    {
        std::string k = "";

        k = static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str();

        if (chainMap[k].objects[k].vertexes.size() <=3)
        {
            dBodyID odebody;
            odebody = makePrimitive(chainMap[k].objects[k].position,chainMap[k].objects[k].orientation,chainMap[k].objects[k].mass,chainMap[k].objects[k].vertexes);

            stateBodiesmap_.insert(pair<string,dBodyID>(k,odebody));



        }
        else
        {
            dBodyID odebody;
            const vector<double> position = chainMap[k].objects[k].position;
            const vector<double> orientation = chainMap[k].objects[k].orientation;
            const vector<double> vertexes=chainMap[k].objects[k].vertexes;
            const vector<unsigned int> indexes=chainMap[k].objects[k].indexes;
            double mass=chainMap[k].objects[k].mass;
            odebody = makeTriMesh(position,orientation,vertexes,indexes,mass);

            dVector3 pos;
            dBodyCopyPosition ( odebody, pos);
            stateBodiesmap_.insert(pair<string,dBodyID>(k,odebody));
            bodies.push_back(odebody);

            //dBodyDestroy(odebody);

        }
    }*/

//setjointsandmotors2bodies(stateBodiesmap_,chainMap,wkspace);
//vector<dBodyID> tmp;

//stateBodies_=fillstatebodies(stateBodiesmap_,wkspace);
stateBodies_=bodies;

//tmp=bodies;//fillstatebodies(stateBodiesmap_,wkspace);

SetPlanningParameters();

    //tmp=stateBodies_;
    //dVector3 pos;
   // dBodyCopyPosition ( tmp[0], pos);
    //dVector3 pos1;
    //dBodyCopyPosition ( tmp[1], pos1);
    //dVector3 pos2;
    //dBodyCopyPosition ( tmp[2], pos2);


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

   stepSize_ = 0.05;
   maxContacts_ = 3;
   minControlSteps_ = 10;
   maxControlSteps_ = 500;
}


void KauthamDEEnvironment::setjointsandmotors2bodies(map<string, dBodyID> stateBodiesmap_,map<string, KinematicChain> chainMap, WorkSpace* wkspace)
{

    for (map<string,Joint>::iterator it = chainMap[wkspace->getRobot(0)->getName()].joints.begin(); it!=chainMap[wkspace->getRobot(0)->getName()].joints.end(); ++it)
    {
        dBodyID body1 = stateBodiesmap_[it->second.target1];
        dBodyID body2 = stateBodiesmap_[it->second.target2];
        dJointID joint = makeJoint(body1,body2,it->second.type,it->second.position);
        //makeMotor(body1,body2,it->second.type,it->second.position,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax);
        //dJointID motorjoint = makeMotor(body1,body2,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].type,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].pos);
        addMotor2Joint(joint,chainMap[wkspace->getRobot(0)->getName()].motors[it->first].fmax);
    }
    /*for(map<string,Motor>::iterator it = chainMap[wkspace->getRobot(0)->getName()].motors.begin(); it!=chainMap[wkspace->getRobot(0)->getName()].motors.end(); ++it)
    {
        //dBodyID body3 = stateBodiesmap_[chainMap[wkspace->getRobot(0)->getName()].joints[it->second.targetjoint].target1];
       // dBodyID body4 = chainMap[wkspace->getRobot(0)->getName()].joints[it->second.targetjoint].target2;
        makeMotor(stateBodiesmap_[chainMap[wkspace->getRobot(0)->getName()].joints[it->second.targetJoint].target1],stateBodiesmap_[chainMap[wkspace->getRobot(0)->getName()].joints[it->second.targetJoint].target2],0,chainMap[wkspace->getRobot(0)->getName()].joints[it->second.targetJoint].position,it->second.fmax);
    }*/

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
        }
    }
    else
    {
        //comprovem que no sigui 'taula'
        if(wkspace->getRobot(0)->getNumLinks()==1)
        {
            bodies.push_back(stateBodiesmap_[wkspace->getRobot(0)->getName()+ wkspace->getRobot(0)->getLink(0)->getName()]);

            for (int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (int j=0; j < (wkspace->getObstacle(i)->getNumLinks()); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getObstacle(i)->getLink(j)->getName()]);
                }
            }
            for (int i=0; (i < wkspace->getNumObstacles()); i++)
            {
                std::string k = "";

                k = static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str();
                bodies.push_back(stateBodiesmap_[k]);
            }

           for(map<string,dBodyID>::iterator it = stateBodiesmap_.begin(); it!=stateBodiesmap_.end(); ++it)
           {
               if(it->first == wkspace->getRobot(0)->getName())
               {
               }
               else
               {
                   //bodies.push_back(stateBodiesmap_[it->second]);
               }
           }

        }
        //si no és una taula será un robot format per varios links
        else
        {
                        //omplir els cossos corresponents als links
            for (int i=0; i < wkspace->getNumRobots(); i++)
            {
                for (int j=0; j < wkspace->getRobot(i)->getNumLinks(); j++ )
                {
                    bodies.push_back(stateBodiesmap_[(wkspace->getRobot(i)->getName()) + (wkspace->getRobot(i)->getLink(j)->getName())]);
                }
            }
            for (int i=0; i < wkspace->getNumObstacles(); i++)
            {
                for (int j=0; j < wkspace->getObstacle(i)->getNumLinks(); j++ )
                {
                    bodies.push_back(stateBodiesmap_[wkspace->getObstacle(i)->getName()+wkspace->getRobot(i)->getLink(j)->getName()]);
                }
            }
            for (int i=0; i < wkspace->getNumObstacles(); i++)
            {
                std::string k = "";

                k = static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str();
                bodies.push_back(stateBodiesmap_[k]);
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
    //string directory = robFileName.substr(0,robFileName.find_last_of("/")+1);
    //xml_document robotXml;
    //xml_parse_result parsed = robotXml.load_file(robFileName.c_str());
    //if(parsed)
    //{
    //string robotName = robotXml.child("Robot").attribute("name").value();
    //string robotName = robot->getName();
    chain->name = robot->getName();
        //string robotDHType = robotXml.child("Robot").attribute("DHType").value();
    chain->dhType = robot->getDHApproach();
        //xml_node linkNode = robotXml.child("Robot").child("Joints");

        //xml_node_iterator linkIterator = linkNode.begin();
        int i=0;
        if (!basePos.empty())
        {
            odinObject base;
            base.mass = 1;
            //base.name = robotName + ((*linkIterator).attribute("name").value());
            base.name = (chain->name) + (robot->getLink(0)->getName());
            //string ivFileName = directory + (*linkIterator).attribute("ivFile").value();
            getTrimesh(((IVPQPElement*)robot->getLink(i)->getElement())->getIvFromPQPModel(false),&base, scale);
            //getTrimesh(((IVElement*)robot->getLink(0)->getElement())->ivModel(false), &base, scale);

            for (int j = 0; j < 3; j++)
            {
                base.position.push_back(basePos[j]);
                base.orientation.push_back(basePos[j+3]);
            }
            base.orientation.push_back(basePos[6]);

            chain->objects[base.name] = base;
            i++;
        }
        for(; i < robot->getNumLinks(); ++i)
        {
            odinObject obj;
            obj.mass = 1;
            //obj.name = robotName + ((*linkIterator).attribute("name").value());
            obj.name = (chain->name) + (robot->getLink(i)->getName());
            //string ivFileName = directory + (*linkIterator).attribute("ivFile").value();
            getTrimesh(((IVPQPElement*)robot->getLink(i)->getElement())->getIvFromPQPModel(false),&obj, scale);
            //getTrimesh( ((IVElement*)robot->getLink(i)->getElement())->ivModel(false), &obj, scale);
            vector<double> rotAxis;
            getTransformation(chain, chain->dhType, robot->getLink(i), &obj, chain->name, rotAxis);
            getMotion(chain, robot->getLink(i), &obj, chain->name, rotAxis);
            chain->objects[obj.name] = obj;
        }
        return true;
    }


void KauthamDEEnvironment::getTrimesh(SoSeparator *ivmodel, odinObject* obj, double scale)
{
    //SoInput input;
    //SoSeparator *ivmodel;
    //ivmodel = new SoSeparator;
    //ivmodel->ref();
    SoScale * sca = new SoScale();
    sca->scaleFactor.setValue((float)scale,(float)scale,(float)scale);
    ivmodel->addChild(sca);

    //if(input.openFile(ivFileName.c_str()))
        //ivmodel->addChild(SoDB::readAll(&input));
    //else
        //return;

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

        Motor motor;
        motor.targetJoint = jointID;
        motor.fmax.push_back(INFINITY);
        chain->motors[jointID] = motor;

        if (link->getRotational())
        {
            joint.hiStop = *(link->getLimits(false)) * toRad;
            joint.loStop = *(link->getLimits(true)) * toRad;
            joint.type = 1;
// 			cout << "ROTATIONAL JOINT BETWEEN " << joint.target1 << " AND " << joint.target2 << '\n';
            for (int i = 0; i < 3; i++)
                joint.position.push_back(obj->position[i]);
            for (int j = 0; j < 3; j++)
                joint.position.push_back(rotAxis[j]);
            chain->joints[jointID] = joint;
        }
        else //linear
        {
            joint.hiStop = *(link->getLimits(false));
            joint.loStop = *(link->getLimits(true));
            joint.type = 2;
// 			cout << "LINEAR JOINT BETWEEN " << joint.target1 << " AND " << joint.target2 << '\n';
            for (int i = 0; i < 3; i++)
                joint.position.push_back(rotAxis[i]);
            chain->joints[jointID] = joint;
        }
        chain->jointsOrdered.push_back(joint);
    }
    return true;
}

bool KauthamDEEnvironment::getTransformation(KauthamDEEnvironment::KinematicChain* chain,
                                    string robotDHType,
                                    Link* link,
                                    odinObject* obj,
                                    string robotName,
                                    vector<double>& rotAxis)
{
    //Building D-H Matrix
    double theta = (link->getTheta()) * toRad;
    double d = (link->getD())*SCALE;
    double a = (link->getA())*SCALE;
    double alpha = (link->getAlpha()) * toRad;
    double dhMatrix[4][4];
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            if(i == j)
                dhMatrix[i][j]= 1;
            else
                dhMatrix[i][j]= 0;

    if(robotDHType == "Standard")
    {
        dhMatrix[0][0] = cos(theta);
        dhMatrix[0][1] = -cos(alpha)*sin(theta);
        dhMatrix[0][2] = sin(alpha)*sin(theta);
        dhMatrix[0][3] = a*cos(theta);


        dhMatrix[1][0] = sin(theta);
        dhMatrix[1][1] = cos(alpha)*cos(theta);
        dhMatrix[1][2] = -sin(alpha)*cos(theta);
        dhMatrix[1][3] = a*sin(theta);

        dhMatrix[2][1] = sin(alpha);
        dhMatrix[2][2] = cos(alpha);
        dhMatrix[2][3] = d;
    }
    else
    {
        dhMatrix[0][0] = cos(theta);
        dhMatrix[0][1] = -sin(theta);
        dhMatrix[0][3] = a;

        dhMatrix[1][0] = cos(alpha)*sin(theta);
        dhMatrix[1][1] = cos(alpha)*cos(theta);
        dhMatrix[1][2] = -sin(alpha);
        dhMatrix[1][3] = -d*sin(alpha);

        dhMatrix[2][0] = sin(alpha)*sin(theta);
        dhMatrix[2][1] = sin(alpha)*cos(theta);
        dhMatrix[2][2] = cos(alpha);
        dhMatrix[2][3] = d*cos(alpha);
    }

    mt::Rotation tempRot(mt::Matrix3x3(dhMatrix[0][0], dhMatrix[0][1], dhMatrix[0][2],
                                       dhMatrix[1][0], dhMatrix[1][1], dhMatrix[1][2],
                                       dhMatrix[2][0], dhMatrix[2][1], dhMatrix[2][2]));

    mt::Point3 tempTran(dhMatrix[0][3], dhMatrix[1][3], dhMatrix[2][3]);

    mt::Transform dhTemp(tempRot, tempTran);
    mt::Transform absoluteTransform = *(link->getTransformation());

    string parent = link->getParent()->getName();
    if (!parent.empty())
    {
        odinObject* daddy = &chain->objects[robotName + parent];
        mt::Point3 parentTranslation(daddy->position[0], daddy->position[1], daddy->position[2]);
        mt::Rotation parentRotation(daddy->orientation[1], daddy->orientation[2],
                                    daddy->orientation[3], daddy->orientation[0]);
        mt::Transform parentTransform(parentRotation,parentTranslation);
        absoluteTransform = parentTransform;
    }
    else
        cout << "link has no parent\n";

    /*xml_node preTNode = (*linkIterator).child("PreTrans");
    if( preTNode != NULL )
    {
        mt::Point3 tempPreTran(preTNode.attribute("X").as_double(),
                               preTNode.attribute("Y").as_double(),
                               preTNode.attribute("Z").as_double());
        mt::Rotation tempPreRot(mt::Unit3(preTNode.attribute("WX").as_double(),
                                          preTNode.attribute("WY").as_double(),
                                          preTNode.attribute("WZ").as_double()),
                                preTNode.attribute("TH").as_double() * toRad);
        mt::Transform preTransform(tempPreRot,tempPreTran);
        obj->preTransform = preTransform;
        absoluteTransform *= preTransform;
    }
    else
        obj->preTransform.setIdentity();*/

    //absoluteTransform *= dhTemp;

    tempTran = absoluteTransform.getTranslation();
    tempRot = absoluteTransform.getRotation();

    obj->orientation.push_back(tempRot[3]);
    for(int j=0; j<3; j++)
    {
        obj->position.push_back(tempTran[j]);
        obj->orientation.push_back(tempRot[j]);
    }

    mt::Rotation rot = absoluteTransform.getRotation();
    mt::Matrix3x3 rotMatrix = rot.getMatrix();
    rotAxis.push_back(rotMatrix[0][2]);
    rotAxis.push_back(rotMatrix[1][2]);
    rotAxis.push_back(rotMatrix[2][2]);
    obj->a = a;
    obj->d = d;
    obj->theta = theta;
    obj->alpha = alpha;
    return true;
}

vector<KthReal> KauthamDEEnvironment::baseGetPos(Robot* robot)
{
    vector<KthReal> basePos;
    vector<KthReal> tmp;
    //for(xml_node_iterator robotIterator = (*problemIterator).begin(); robotIterator != (*problemIterator).end(); ++robotIterator)
    //{
        //string attribute = (*robotIterator).name();
        //if("Home" == attribute)
        //{
            /*basePos.push_back((robot->getLink(0)->getElement()->getPosition()[0])*SCALE);
            basePos.push_back((robot->getLink(0)->getElement()->getPosition()[1])*SCALE);
            basePos.push_back((robot->getLink(0)->getElement()->getPosition()[2])*SCALE);
            mt::Rotation tmp (mt::Unit3((robot->getLink(0)->getElement()->getOrientation()[0]),(robot->getLink(0)->getElement()->getOrientation()[1]),(robot->getLink(0)->getElement()->getOrientation()[2])),(robot->getLink(0)->getElement()->getOrientation()[3])* toRad);
            basePos.push_back(tmp[3]);
            basePos.push_back(tmp[0]);
            basePos.push_back(tmp[1]);
            basePos.push_back(tmp[2]);*/
            RobConf* RobC = robot->getCurrentPos();
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

 dBodyID KauthamDEEnvironment::makePrimitive(const vector<double>& position, const vector<double>& orientation, const double mass, const vector<double>& params)
 {
     dBodyID body = dBodyCreate(bodyworld);
     //dBodyID body = dBodyCreate(_world);
     dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
     dBodySetPosition(body, position[0], position[1], position[2]);
     dBodySetQuaternion(body, q);
     dGeomID geometry;
     dMass buildingMass;
     switch (params.size())
     {
         case 1:
             geometry = dCreateSphere(_OpenDEspace, params[0]);
             dMassSetSphereTotal(&buildingMass, mass, params[0]);
             break;
         case 2:
             geometry = dCreateCylinder(_OpenDEspace, params[0], params[1]);
             dMassSetCylinderTotal(&buildingMass, mass, 2, params[0], params[1]);
             break;
         case 3:
             geometry = dCreateBox(_OpenDEspace, params[0], params[1], params[2]);
             dMassSetBoxTotal(&buildingMass, mass, params[0], params[1], params[2]);
             break;
         default:
             return NULL;
     }
     dBodySetMass(body, &buildingMass);
     dGeomSetBody(geometry, body);
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
     for (int i = 0; i < (vertexes.size()); i++)
         vrtxs[i] = ((float)vertexes[i]);

     dTriIndex * trindexes = new dTriIndex[indexes.size()];
     for (int j = 0; j < (indexes.size()); j++)
         trindexes[j] = indexes[j];

     dTriMeshDataID data = dGeomTriMeshDataCreate();
     dGeomTriMeshDataBuildSingle(data, vrtxs, 3*sizeof(float), (int)vertexes.size(), trindexes, indexes.size(), 3*sizeof(unsigned int));
     dGeomID geometry = dCreateTriMesh(_OpenDEspace, data, NULL, NULL, NULL);

     dGeomSetPosition(geometry, position[0], position[1], position[2]);
     dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
     dGeomSetQuaternion(geometry, q);
    return;
 }

 dBodyID KauthamDEEnvironment::makeTriMesh(const vector< double > position, const vector< double > orientation,
                                  const vector< double > vertexes, const vector< unsigned int > indexes, const double mass)
 {
     dMass buildingMass;

     //body = NULL;
     dBodyID body = dBodyCreate(bodyworld);
     //dBodyID body = dBodyCreate(_world);
     //Building trimesh data.
     float * vrtxs = new float[vertexes.size()];
     for (int i = 0; i < (vertexes.size()); i++)
        vrtxs[i] = ((float)vertexes[i]);

     dTriIndex * trindexes = new dTriIndex[indexes.size()];
     for (int j = 0; j < (indexes.size()); j++)
         trindexes[j] = indexes[j];

     dTriMeshDataID data = dGeomTriMeshDataCreate();
     dGeomTriMeshDataBuildSingle(data, vrtxs, 3*sizeof(float), (int)vertexes.size(), trindexes, indexes.size(), 3*sizeof(unsigned int));
     //dGeomTrimeshDataBuildSingle(data,vrtxs,(int)vertexes.size(),trindexes,indexes.size());


     dGeomID geometry = dCreateTriMesh(_OpenDEspace, data, NULL, NULL, NULL);
     dGeomSetData(geometry, data);
     dMassSetTrimeshTotal(&buildingMass, mass, geometry);
     dMassTranslate(&buildingMass, -buildingMass.c[0], -buildingMass.c[1], -buildingMass.c[2]);

     dBodySetMass(body, &buildingMass);
     dGeomSetBody(geometry, body);

    // New implementation of Trimesh geom. and building mass     // dGeomTriMeshDataBuildSimple(data, (dReal*) vrtxs,
     //(int)vertexes.size(), trindexes, indexes.size());
     // dGeomID geometry = dCreateTriMesh(_OpenDEspace, data, NULL, NULL, NULL);
     // dGeomSetData(geometry, data);
     // dMassSetTrimeshTotal(&buildingMass, mass, geometry);
     // dGeomSetPosition(geometry, -buildingMass.c[0], -buildingMass.c[1], -buildingMass.c[2]);
     // dMassTranslate(&buildingMass, -buildingMass.c[0], -buildingMass.c[1], -buildingMass.c[2]);
     // dGeomSetBody(geometry, body);
     // dBodySetMass(body, &buildingMass);



     dBodySetPosition(body, position[0], position[1], position[2]);
     dQuaternion q = {orientation[0], orientation[1], orientation[2], orientation[3]};
     dBodySetQuaternion(body, q);
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

 dJointID KauthamDEEnvironment::makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params)
 {
     dJointID joint;
              switch (type)
              {
      //          Joint has to be attatched after it is created, otherwise strange things happen.

                  case 1:
                      joint = dJointCreateHinge(world_, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetHingeAnchor (joint, params[0], params[1], params[2]);
                      dJointSetHingeAxis (joint, params[3], params[4], params[5]);
                      break;
                  case 2:
                      joint = dJointCreateSlider(world_, 0);
                      dJointAttach(joint, body1, body2);
                      dJointSetSliderAxis(joint, params[0], params[1], params[2]);
                      break;

              }
       return joint;
 }

/* dJointID KauthamDEEnvironment::makeMotor(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector< double >& axes,const vector< double >& fmax)
 {
     dJointID motor;
          int numAxes = (int)(axes.size()/3);
          switch (type)
          {
              case angular_motor:
              {
                  motor = dJointCreateAMotor(_OpenDEWorld, 0);
                  dJointAttach(motor, body1, body2);
                  dJointSetAMotorNumAxes(motor, numAxes);
                  //0 es el eje, 1 es que va a estar anclado al objeto1, 0 seria el global frame, 2 el objeto2
                  dJointSetAMotorAxis(motor, 0, 1, axes[0], axes[1], axes[2]);
      //          dJointSetAMotorParam(motor, dParamFMax, dInfinity);
                  dJointSetAMotorParam(motor, dParamFMax, fmax[0]);
                  dJointSetAMotorParam(motor, dParamVel, 0);
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
              case linear_motor:
              {Ascherbachstr. 4
                  motor = dJointCreateLMotor(_OpenDEWorld, 0);
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
          return motor;
 }*/

 void KauthamDEEnvironment::makeMotor(dBodyID body1,dBodyID body2,const unsigned int type, const vector< double >& axes,const vector< double >& fmax)
  {
      dJointID motor;
           int numAxes = (int)(axes.size()/3);
           switch (type)
           {
               case 0:
               {
                   motor = dJointCreateAMotor(world_, 0);
               //motor = dJointCreateAMotor(_world, 0);
                   dJointAttach(motor, body1, body2);
                   dJointSetAMotorNumAxes(motor, numAxes);
                   //0 es el eje, 1 es que va a estar anclado al objeto1, 0 seria el global frame, 2 el objeto2
                   dJointSetAMotorAxis(motor, 0, 1, axes[0], axes[1], axes[2]);
       //          dJointSetAMotorParam(motor, dParamFMax, dInfinity);
                   dJointSetAMotorParam(motor, dParamFMax, fmax[0]);
                   dJointSetAMotorParam(motor, dParamVel, 0);
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
                   motor = dJointCreateLMotor(world_, 0);
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

  }

 void KauthamDEEnvironment::addMotor2Joint(dJointID joint, vector<double>& maxForces)
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
          makeMotor(body1,body2,type,axisVec,maxForces);
          //jointForces[id] = JointForce(joints[id], magnitude);
         // return true;
 }

 /*void KauthamDEEnvironment::setAnchorage(dJointID motor, vector<unsigned int>& anchor)
 {
     dVector3 axis;
              switch (dJointGetType(motor))
              {
                  case dJointTypeAMotor:
                      for (int i = 0; i < 3; i++)
                      {
                          dJointGetAMotorAxis(motor, i, axis);
                          dJointSetAMotorAxis(motor, i, (int)anchor[i], axis[0], axis[1], axis[2]);
                      }
                      break;
                  case dJointTypeLMotor:
                      for (int j = 0; j < 3; j++)
                      {
                          dJointGetLMotorAxis(motor, j, axis);
                          dJointSetLMotorAxis(motor, j, (int)anchor[j], axis[0], axis[1], axis[2]);
                      }
                      break;
                  default:
                      ;
              }
 }*/

/*void KauthamDEEnvironment::jointSet(dJointID joint, unsigned int parameter, dReal value)
{
 switch (dJointGetType(joint))
          {
              case dJointTypeBall:
                  dJointSetBallParam(joint, parameter, value);
                  break;
              case dJointTypeHinge:
                  dJointSetHingeParam(joint, parameter, value);
                  break;
              case dJointTypeSlider:
                  dJointSetSliderParam(joint, parameter, value);
                  break;
              case dJointTypeUniversal:
                  dJointSetUniversalParam(joint, parameter, value);
                  break;
              case dJointTypeHinge2:
                  dJointSetHinge2Param(joint, parameter, value);
                  break;
              case dJointTypePR:
                  dJointSetPRParam(joint, parameter, value);
                  break;
              case dJointTypePU:
                  dJointSetPUParam(joint, parameter, value);
                  break;
              case dJointTypePiston:
                  dJointSetPistonParam(joint, parameter, value);
                  break;
              case dJointTypePlane2D:
                  dJointSetPlane2DAngleParam(joint, parameter, value);
  //              dJointSetPlane2DXParam(joint, parameter, value);
  //              dJointSetPlane2DYParam(joint, parameter, value);
                  break;
              case dJointTypeAMotor:
                  dJointSetAMotorParam(joint, parameter, value);
                  break;
              case dJointTypeLMotor:
                  dJointSetLMotorParam(joint, parameter, value);
              default:
                  ;
          }

}*/
}

};

#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

