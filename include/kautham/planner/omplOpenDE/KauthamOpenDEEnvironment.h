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
#if !defined(_KauthamOpenDEEnvironment_H)
#define _KauthamOpenDEEnvironment_H
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#define dDOUBLE
#include <ode/ode.h>
#include <ompl/extensions/opende/OpenDEEnvironment.h>
#include <ompl/extensions/opende/OpenDEControlSpace.h>
#include <ompl/extensions/opende/OpenDEStateSpace.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/extensions/opende/OpenDEStatePropagator.h>
#include <ompl/extensions/opende/OpenDEStateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <kautham/problem/link.h>
#include <kautham/problem/robot.h>
#include <kautham/problem/workspace.h>

#define _USE_MATH_DEFINES

#include <math.h>
//#include <pugixml.hpp>
#include <fstream>
#include <iostream>
#include <Inventor/fields/SoSFVec3f.h>
#include <Inventor/fields/SoSFRotation.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SbLinear.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <vector>
#include <cmath>
//#include <libmt/mt/mt.h>
#include <kautham/problem/link.h>
#include <kautham/problem/robot.h>
#include <kautham/problem/ivelement.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


using namespace std;

namespace Kautham
{
/** \addtogroup PhysicsBasedPlanners Physics-based Planners
 *  @{
 */
namespace omplcplanner
{

/***************************************************************************
 *  /* OpenDEEnvironment es una classe en que es recicla gran part del codi ODIn de l'Alfredo sobretot en
     la part en què es construeixen els bodys i les geometries de ODE. ODIN tenia la possibilitat de construir el món
a partir dels fitxers XML o a partir de la GUI de odin, nosaltres hem aprofitat la part del codi que crea cossos TRIMESH
des de XML. Com Kautham ja s'ha encarregat de llegir els fitxers hem fet petits canvis als codis perque treiessin la informació
del workspace que ell mateix ja l'ha extret previament dels fitxers XML.*
D'aquesta classe han de derivar classes per cada environment que volguem crear on s'han de definir els bounds del control i la manera com
OMPL i ODE han d'aplicar el control , (han d'aplicar forces als cossos , o parells als motors , o bé han de donar velocitats ?? necessitem cossos definits com a kinematics?? etc...
 ***************************************************************************/


/************************************************* **************************
 * / * OpenDEEnvironment is a class that recycles much of the code Odin Alfredo especially
     the part where you build and Bodys geometries ODE . ODIN was the possibility of building the world
from XML files or from the GUI Odin , we have taken the part of the code that creates bodies TRIMESH
from XML . As has already been commissioned Kautham read the files we made small changes to the code so removing information
the workspace that he has already previously extracted XML files . *
This class must derive classes for each environment we want to create where to set the Bounds of the control and how
Complete and ODE must apply the control ( have to apply forces to the bodies or even engines , or should give speeds ? Need bodies defined as Kinematics ? Etc ...
 ************************************************** *************************/
/*
 * ODE 
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Class KauthamDEEnvironment
    /////////////////////////////////////////////////////////////////////////////////////////////////*/
//! KauthamDEEnvironment class read the workspace and generate the dynamic environment.
//! this class is the parent of all the enviroments (like KauthamDERobotEnvironment, KauthamDETableEnvironment,..) in which ODE will apply the control.
    class KauthamDEEnvironment: public oc::OpenDEEnvironment
    {
        public:

        //Crida per cada Link dels robots i per cada cos dels obstacles la funció buildkinematicChain i omple un mapa de KinematicChain que
        //anomenem ChainMap. (cada chain te informacio del objecte de joints , etc... (veure documentació odin). Un cop ple ChainMap cridem a la creació del world amb createWorld.
        //cada cos dintre del chainMap es diu com nomrobot+nomlink


		// Called for each Link of robots and obstacles for every body function and fill buildkinematicChain map that KinematicChain
         // call ChainMap. (each chain has information object joints, etc ... (see documentation Odin.) Once full ChainMap called the creation of the world with createWorld.
         // every body within chainMap called as nomrobot + nomlink

        KauthamDEEnvironment(WorkSpace* wkspace, KthReal maxspeed);//!< constructor will build the KinamaticChainMap (each chain has information about objects, joints, etc)and creat the world.
        ~KauthamDEEnvironment(void);

        //aprofitant la informació del ChainMap va recorrent tots els cossos del workspace altre cop i va creant bodysode i omplint un mapa StateBodiesmap_ amb els bodys. Aquest mapa després s'anirà extreient
        // a stateBodies_ propi de ompl en un ordre en que els primers cossos a introduir siguin els del robot a planificar. La idea era intentar que fos capaç de distingir entre primitive i trimesh però només funciona amb trimesh
        //ja que ODIN només acceptava primitives si s'entraven via GUI.
        //També crida a setjointandmotors2bodies que és la part del codi en que més dubtes tinc ja que no domino ben bé si he de introduir motors o joints als enllaços.
        //De primeres he optat per introduïr motors(això es el que crec que fa el codi).


		 // making information ChainMap scoured the bodies of workspace again and bodysode creating and filling a map with StateBodiesmap_ Bodys. This map will then by drawing
         // StateBodies_ own to fill in the order they are introduced in the first stage of the robot plan. The idea was to try to be able to distinguish between primitive and trimesh but only works trimesh
         // since ODIN only accepted if entered via primitive GUI.
         // Call setjointandmotors2bodies is also part of the code that I doubt not dominate quite as if I introduce engines joints or links.
         // At first I chose to introduce engines (this is what I think makes the code).
        void createWorld(WorkSpace *wkspace); //!< This function will create the ODE world
        void destroyWorld(); //!< This function destory the world.

        virtual void SetPlanningParameters(); //!< setup the necessary planning parameter for OpenDE.



        //PROVIENEN DE ODIN

        //Totes aquests mètodes provenen d'ODIN i tenen com a objectiu passar d'una estructura de dades organitzada a la manera de Kautham
        //passar a tenir les estructuras tal com les demanen els cossos ODE dBody y les seves geometries. Per fer això s'utilitzan les estructures que
        // usava el Alfredo a ODIN i que en aquest cas anomeno odinobjects.
        //Les estructures son:
        //-odinObject: Conté la informació d'un cos individual , posició , orientació , forma , dimensions, etc... tenen un mom associat que correspon o bé al nom del robot+ el nom del link o bé
        // a l'index del obstacle al que representen dintre del vector d'obstacles de workspace.
        //-Joints:Conté informació del tipus de joint que és , de la seva posició , dels límits dels seus valors i els noms del objectes que vol unir.
        //-Motor:Nom , joint objectiu i força màxima
        //-KinematicChain:KinematicChain representa un robot amb tots els seus objectes, els joints i tota la seva informació.

        //Mapes creats:
        //-chainMap: Mapa de KinematicsChains
        //-stateBodiesmap_:Mapa de odebodys, es a dir conté la mateixa informació que chainMap però 'a la manera del ode'. Es a dir , ja conté dBodys.
        // A partir de l'estructura stateBodiesMap_ s'omple l'estructura de l'environment de OMPL StateBodies_ que es un vector amb els dBodys de la escena.
        //S'omplen de manera que els primers dobys del vector corresponguin al robot pel qual es planifica.

 // PROVIENEN Odin

        // All these methods come from ODIN and aim to pass a data structure organized in the way Kautham
        // move to have the structures as demanded by the ODE dBody bodies and their geometries . To do this the structures that are used,
        // Used the Alfredo Odin and in this case call odinobjects .
        // Structures are:
        // - odinObject : Contains information about a single body position , orientation , shape, size, etc. ... mom have a partner that corresponds to the name or the name of the robot + link or
        // The index of the obstacle within the vector representing obstacles workspace .
        // - Joints : Contains information about the type of joint that is their position , the limits of their values ​​and the names of the objects you want to join .
        // Engine - name, target and maximum strength joint
        // - KinematicChain : KinematicChain represents a robot with all its objects, joints and all your information .

        // Created maps :
        // - chainMap : Map KinematicsChains
        // - stateBodiesmap_ : Map odebodys , ie contains the same information but chainMap ' ode to the way ' . That is, it contains dBodys .
        // From the structure stateBodiesMap_ fills the structure of the environment that StateBodies_ fill a vector with dBodys the scene .
        // Fill up so early Doby vector corresponding to the robot which is planned .

        dBodyID makePrimitive(const vector< double >& position, const vector< double >& orientation,
                               const double mass, const vector< double >& params); //!< this function creates ODE Bodies (dBody).

        void makePrimitiveForComposite(const vector< double >& position, const vector< double >& orientation,
                                        const double mass, const vector< double > params, const dBodyID coreBody);

        dBodyID makeTriMesh(const vector<double> position,
                             const vector<double> orientation,
                           const vector<double> vertexes,
                           const vector<unsigned int> indexes,
                           const double mass); //!< This function make the ODE bodies (dBody) using triangular mesh.

         void makeGeomPrimitive(const string name, const vector<double>& position,
                                const vector<double>& orientation, const vector<double>& params);

         void makeGeomTrimesh(const string name, const vector<double>& position, const vector<double>& orientation,
                            const vector<double>& vertexes, const vector<unsigned int>& indexes); //!< This function create the geometries of ODE bodies using triangular mesh.



        static void triang_CB(void *data,
                              SoCallbackAction *action,
                              const SoPrimitiveVertex *vertex1,
                              const SoPrimitiveVertex *vertex2,
                              const SoPrimitiveVertex *vertex3);

        //! Contains information about body position , orientation , shape, size, etc.
        typedef struct
        {
            string name;
            vector<double> position;
            vector<double> orientation;
            mt::Transform preTransform;
            vector<double> vertexes;
            vector<unsigned int> indexes;
            double mass;
            vector<float> color;
            bool fixed;
            double a;
            double d;
            double theta;
            double alpha;
        } odinObject;
        //! Contains information about the type of joint that is their position , the limits of their values ​​and the names of the Bodies that we want to join.
        typedef struct
        {
            string target1;
            string target2;
            vector<double> position;
            unsigned int type;
            double loStop;
            double hiStop;
        } Joint;


        //! Contains information about the motor
        typedef struct
        {
            string name;
            string targetJoint;
            vector<double> fmax;
        } Motor;



        //la estructura que forma el chainMap que s'omple en el constructor de la classe,els objects contenen informació de la geometria en general (vertexs , indexs, etc...)
        //els joints tenen informació dels joints (bodys que junten, posició , tipus , etc...)


		// structure that forms the chainMap be filled in class constructor, the objects contain information about the overall geometry (vertices, indices, etc ...)
         // the joints are joints of information (together Bodys, position, type, etc ...)
        //! KinematicChain represents a robot with all its objects, joints and all information .
        typedef struct
        {
            string name;
            map<string, odinObject> objects;
            map<string, Joint> joints;
            vector<Joint> jointsOrdered;
            map<string, Motor> motors;
            bool internalCollision;
            string dhType;
        } KinematicChain;
        map<string, KinematicChain> chainMap; //!< map KinamaticChain
        virtual void setjointsandmotors2bodies(map<string,dBodyID> stateBodiesmap_, map<string, KinematicChain> chainMapchainMap, WorkSpace* ws);//!< this function set the joint and motor for in ODE bodies
        void getTrimesh(SoSeparator *ivmodel, odinObject* obj, double scale);//!< get the triangular mesh.

        vector<dBodyID> bodies; //!< Vector of ODE Bodies.
        dSpaceID _OpenDEspace;  //!< Define the ODE space.
        dWorldID bodyworld;     //!< Define the ODE world.
        map<string, dBodyID> stateBodiesmap_;//!< stateBodiesmap_  Map ODE bodies , it contains the same information as ODE bodies has but in the form of chainMap.

        int _NumLinksFirstRobot; //!< define number of links of robot.
        KthReal _maxspeed;       //!< Define max. speed of motor.

        unsigned int getNumLinksFirstRobot(){return _NumLinksFirstRobot;}; //!< returns number of links of first robot

        //la idea es omplir _StateBodies de ompl fent que els dels primers indexs siguin els cossos a planejar amb ells , es a dir el robot o la taula


        // fill _StateBodies the idea of making the fill Indexes are the first bodies to plan with them, ie the robot or table

        vector<dBodyID> fillstatebodies(map<string,dBodyID> stateBodiesmap_, WorkSpace *wkspace);//!< this function return the vector of ODE bodies in such a way that, robot will be the first body.


           private:

            constexpr static double toRad  = M_PI/180.;

            //! This function will build the kinamatic chain for Robot
            bool buildKinematicChain(KauthamDEEnvironment::KinematicChain* chain, Robot *robot, double scale, vector<KthReal>& basePos);

            bool getTransformation(KauthamDEEnvironment::KinematicChain* chain, string robotDHType, Link* link, odinObject* obj, string robotName, vector<double>& rotAxis);

            bool getMotion(KauthamDEEnvironment::KinematicChain* chain, Link* link, odinObject* obj, string robotName, vector<double>& rotAxis);

            void searchColor(SoSeparator* root, odinObject* obj);

            vector<KthReal> baseGetPos(Robot* robot);// this function will returns the position and orientation of robot.

            void makeMotor(dBodyID body1, dBodyID body2,const unsigned int type, const vector< double >& axes,const vector< double >& fmax);
            void addMotor2Joint(dJointID joint, vector<double>& maxForces);
            dJointID makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params);

            int sceneObjectNumber;


    };


}
 /** @}   end of Doxygen module "Planner */
}


#endif  //_KauthamOpenDEEnvironment_H
#endif //KAUTHAM_USE_ODE
#endif // KAUTHAM_USE_OMPL

