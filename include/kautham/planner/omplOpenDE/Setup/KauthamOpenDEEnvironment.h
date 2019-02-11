
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


//#if defined(KAUTHAM_USE_OMPL)
//#if defined(KAUTHAM_USE_ODE)
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
//#include <problem/obstacle.h>
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
#include <kautham/problem/ivpqpelement.h>
#include <kautham/problem/urdf.h>
#include <kautham/planner/omplOpenDE/InstantiatedKnowledge/Instantiatedknowledge.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


using namespace std;

namespace Kautham
{
/** \addtogroup Environment
 *  @{
 */
namespace omplcplanner
{
class ManipulationQuery;
class Manipulatorkinematics;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  // Class KauthamDEEnvironment
  /////////////////////////////////////////////////////////////////////////////////////////////////*/

//! KauthamDEEnvironment class read the workspace of kautham and generates the dynamic environment.
//! This class is the parent of all the enviroments (like CarEnviornment, twoDRobotEnvironment,..) in which ODE will apply the control.
class KauthamDEEnvironment: public oc::OpenDEEnvironment
{
public:

    // Called for each Link of robots and obstacles for every body function and fill buildkinematicChain map that KinematicChain
    // call ChainMap. (each chain has information object joints. Once full ChainMap called the creation of the world with createWorld.
    // every body within chainMap called as nomrobot + nomlink

    KauthamDEEnvironment(WorkSpace* wkspace, KthReal maxspeed, KthReal maxContacts, KthReal minControlsteps,KthReal maxControlsteps, KthReal erp, KthReal cfm, bool isKchain);//!< constructor will build the KinamaticChainMap (each chain has information about objects, joints, etc)and creat the world.
    ~KauthamDEEnvironment(void);
    // making information ChainMap scoured the bodies of workspace again and bodysode creating and filling a map with StateBodiesmap_ Bodys. This map will then by drawing
    // StateBodies_ own to fill in the order they are introduced in the first stage of the robot plan. The idea was to try to be able to distinguish between primitive and trimesh but only works trimesh
    // since ODIN only accepted if entered via primitive GUI.
    // Call setjointandmotors2bodies is also part of the code that I doubt not dominate quite as if I introduce engines joints or links.
    // At first I chose to introduce engines (this is what I think makes the code).
    void createWorld(WorkSpace *wkspace); //!< This function will create the ODE world
    void destroyWorld(); //!< This function destory the world.
    virtual void SetPlanningParameters(); //!< setup the necessary planning parameter for OpenDE.

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
                          const double mass, const vector< double >& params, string name); //!< this function creates ODE Bodies (dBody).

    void makePrimitiveForComposite(const vector< double >& position, const vector< double >& orientation,
                                   const double mass, const vector< double > params, const dBodyID coreBody);

    dBodyID makeTriMesh( vector<double> position,
                         vector<double> orientation,
                         vector<double> vertexes,
                         vector<unsigned int> indexes,
                         double mass,
                         std::string name,
                         std::vector<double> bodydim,
                         std::vector<float> color); //!< This function make the ODE bodies (dBody) using triangular mesh.

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
        std::vector<double> bodydimension;
        double mass;
        vector<float> color;
        bool fixed;
        bool mesh;
        double a;
        double d;
        double theta;
        double alpha;
    } odinObject;
    //! Contains information about the type of joint that is their position , the limits of their values ​​and the names of the Bodies that we want to join.
    typedef struct
    {
        vector<double> vertexes;
        vector<unsigned int> indexes;
        vector<float> color;

    }trimeshD;
    typedef struct
    {
        dTriMeshDataID meshD;
        std::vector<float> vertices;
        std::vector<dTriIndex> indices;
        vector<float> color;
        int indexSize;
    }Tmesh;
    std::vector<Tmesh> meshID;
    typedef struct
    {
        string target1;
        string target2;
        vector<double> position;
        unsigned int type;
        double loStop;
        double hiStop;
        double value;
    } Joint;

    //! Contains information about the motor
    typedef struct
    {
        string name;
        string targetJoint;
        vector<double> fmax;
    } Motor;

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

    //structure that contain the information about the body i.e it is robot body or obstacle body,
    //the geometry that is associated with that body and the name.
    typedef struct
    {
        string bodyName;
        dBodyID body;
        dGeomID Geom;
        dReal mass;
        std::vector<double> dimBody;
        std::vector<double> dimRegion;
        std::vector<string> regionDir;
    }bodyInfo;

    bodyInfo bI;
    std::vector<bodyInfo> bodyDEInfo;
    map<string, KinematicChain> chainMap; //!< map KinamaticChain
    virtual void setjointsandmotors2bodies(map<string,dBodyID> stateBodiesmap_, map<string, KinematicChain> chainMapchainMap, WorkSpace* ws);//!< this function set the joint and motor for in ODE bodies
    void getTrimesh(SoSeparator *ivmodel, odinObject* obj, double scale);//!< get the triangular mesh.
    void getTrimesh(SoSeparator *ivmodel, trimeshD* obj, double scale);
    vector<dBodyID> bodies; //!< Vector of ODE Bodies.
    dSpaceID _OpenDEspace;  //!< Define the ODE space.
    dWorldID bodyworld;     //!< Define the ODE world.
    map<string, dBodyID> stateBodiesmap_;//!< stateBodiesmap_  Map ODE bodies , it contains the same information as ODE bodies has but in the form of chainMap.
    int _NumLinksFirstRobot; //!< define number of links of robot.
    KthReal _maxspeed;       //!< Define max. speed of motor.
    KthReal _propagationStepSize; //!< Define the step size of the world.
    KthReal _maxContacts;//!< Define the number of contact to consider for ODE when two bodies are in contact.
    KthReal _minControlSteps;//!< Define the minimum number of time a control will be applied
    KthReal _maxControlSteps;//!< Define the max number of time a control will be applies.
    KthReal _erp;//!< Represents the value f error reduction parameter for ODE.
    KthReal _cfm;//!< Represents constraint force mixing for ODE.
    std::vector<dGeomID> GeomID; //!< vector of ODE GeomIDs in the world.

    unsigned int getNumLinksFirstRobot(){return _NumLinksFirstRobot;}; //!< returns number of links of first robot

    int tmpcounter;
    // fill _StateBodies the idea of making the fill Indexes are the first bodies to plan with them, ie the robot or table
    vector<dBodyID> fillstatebodies(map<string,dBodyID> stateBodiesmap_, WorkSpace *wkspace);//!< this function return the vector of ODE bodies in such a way that, robot will be the first body.

    dGeomID ground;
    std::vector<dJointID> _Joint;//!< ODE joints vector.
    std::vector<dJointID> _motor;//!< ODE motors vector.

    InstantiatedKnowledge *Instknowledge;//!< Instantiated knowledge instance.
    std::map<string,dBodyID> getBody; //!< map to obtaine the ODE bodies depending on their names.
    std::map<dGeomID,std::string> geomname; //!< map of ODE geometries and names.
    std::map<std::string,dJointID> joint_; //!< map of joints and connected bodies
    std::map<std::string,dJointID> motor_;//!< map of motors and connected bodies

    std::map<dBodyID, std::vector<double> > bodyDim;
    static constexpr double toRad  = M_PI/180.;
    dJointID cJoint[4];
    ManipulationQuery *manipulationQuery;
    Manipulatorkinematics *mkinematics;
    std::vector<double> linklength;
    dJointFeedback *feedback1;
    dJointFeedback *feedback2;
    dJointFeedback *feedback3;
    dJointFeedback *feedback4;

private:

    //static const double toRad  = M_PI/180.;
    bool trimesh;

    //! This function will build the kinamatic chain for Robot
    bool buildKinematicChain(KauthamDEEnvironment::KinematicChain* chain, Robot *robot, double scale, vector<KthReal>& basePos);
    //! This function will build the kinamatic chain for Obstracle
    // bool buildKinematicChain(KauthamDEEnvironment::KinematicChain* chain, Obstacle *obstacle, double scale, int k);
    bool getTransformation(KauthamDEEnvironment::KinematicChain* chain, Link* link, odinObject* obj, string robotName, vector<double>& rotAxis);
    bool getMotion(KauthamDEEnvironment::KinematicChain* chain, Link* link, odinObject* obj, string robotName, vector<double>& rotAxis);
    void searchColor(SoSeparator* root, odinObject* obj);
    void searchColor(SoSeparator* root, trimeshD* obj);
    vector<KthReal> baseGetPos(Robot* robot);// this function will returns the position and orientation of robot.
    dJointID  makeMotor(dBodyID body1, dBodyID body2,const unsigned int type, const vector< double >& axes,const vector< double >& fmax,const double hiStop, const double LoStop,const double value);
    dJointID addMotor2Joint(dJointID joint, vector<double>& maxForces,const double value, const double hiStop, const double LoStop);
    // dJointID makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params,vector<double>& maxForces);
    dJointID makeJoint(const dBodyID body1, const dBodyID body2,const unsigned int type, const vector<double>& params,vector<double>& maxForces, const double hiStop, const double LoStop,const double value);
    int sceneObjectNumber;
    string InstknowledgeInference(bodyInfo bodyinfo);
    int tmp;
    int tmp2;
    std::vector<string> bodyname;
    std::vector<string> bodytype;
    std::vector<string> mRgnDirection;
    std::vector<bool> collisionflag;
    string InferenceProcess(string rigidbodyName, dBodyID body, dGeomID geom);
    void getPrimitiveShapes(Link* link, odinObject* obj, bool rotflag);
};
typedef struct
{
    std::vector<double> q;

}Configuration;

class ManipulationQuery
{
    bool planningPhase;
    bool isKinematicsChain;
    std::string actionType;
    std::string direction;
    unsigned int targetbody;
    std::vector<double> force;
    std::vector<double> goalPose;
    //record the joint values for the configurations in the solution path.
    std::vector<Configuration> jointConf;
    std::vector<Configuration>  qdot;
    std::vector<Configuration>  Torque;
    Configuration conf;
public:
    ManipulationQuery()
    {
        actionType="move";
        planningPhase=true;
        isKinematicsChain=true;
    }
    inline std::string getActionType(){return actionType;}
    inline void setActionType(std::string action){ actionType=action;}
    inline unsigned int gettargeBody(){return targetbody;}
    inline void settargeBody(unsigned int index){ targetbody=index;}
    inline std::string getDirection(){return direction;}
    inline void setDirection(std::string direc){ direction=direc;}
    inline std::vector<double>  getforce(){return force;}
    inline void setforce(std::vector<double>  f){ force=f;}
    inline bool getPlanningPhase(){return planningPhase;}
    inline void setPlanningPhase(bool status){ planningPhase=status;}
    inline bool getIskinematicsChain(){return isKinematicsChain;}
    inline void setIskinematicsChain(bool status){isKinematicsChain=status;}
    inline void setconf(double ithVal){conf.q.push_back(ithVal);}
    inline Configuration getconf(){return conf;}
    void clearconf(void){conf.q.clear();}

    void addJointConfiguration(Configuration config){jointConf.push_back(config);}
    Configuration getJointConfigurationAt(unsigned int i){return jointConf.at(i);}

    void setjointVelocity(Configuration qd){qdot.push_back(qd);}
    Configuration getJointVelocity(unsigned int i){return qdot.at(i);}

    void AddjointTorque(Configuration t){Torque.push_back(t);}
    Configuration getJointTorque(unsigned int i){return Torque.at(i);}

    std::vector<Configuration> getJointConfiguraion(){return jointConf;}
    void setJointConfiguraion(std::vector<Configuration> conf){jointConf=conf;}
    std::vector<Configuration> getJointVelocities(){return qdot;}
    void setJointVelocities(std::vector<Configuration> qd){qdot=qd;}
    std::vector<Configuration> getJointTorque(){return Torque;}
    void setJointTorque(std::vector<Configuration> t){Torque=t;}


};
class Manipulatorkinematics
{
    double **Jacobian;
    double **TransposedJacobian;
    std::vector<double> link;
    std::vector<double> torqueLimit;
    std::vector<double> defaulTorqueLimit;

public:
    inline void setTorqueLimit(std::vector<double> tarqueB){torqueLimit=tarqueB;}
    inline void setdefaultTorqueLimit(std::vector<double> defaultTB){defaulTorqueLimit=defaultTB;}
    inline std::vector<double> getTorqueLimit(){return torqueLimit;}
    inline std::vector<double> getdefaultTorqueLimit(){return defaulTorqueLimit;}

    Manipulatorkinematics(const unsigned int Size, std::vector<double> linklength, std::vector<double> defaultTLimit)
    {
        Jacobian= new double*[Size];
        TransposedJacobian = new double*[Size];
        for(unsigned int i = 0; i < Size; ++i)
        {
            Jacobian[i] = new double[Size];
            TransposedJacobian[i] = new double[Size];

        }
        link=linklength;
        defaulTorqueLimit=defaultTLimit;
    }
    Manipulatorkinematics(std::vector<double> defaultTLimit)
    {
        defaulTorqueLimit=defaultTLimit;
    }
    std::vector<double> getEndEffectorVelocity(std::vector<double> q,std::vector<double> qdot)
    {
        std::vector<double> eeVelocity;
        eeVelocity.resize(2);
        Jacobian[0][0] =-link[0]*sin(q[0])-link[1]*sin(q[0]+q[1]);
        Jacobian[0][1] =-link[1]*sin(q[0]+q[1]);
        Jacobian[1][0] = link[0]*cos(q[0])+link[1]*cos(q[0]+q[1]);
        Jacobian[1][1] = link[1]*cos(q[0]+q[1]);

        eeVelocity[0]=Jacobian[0][0]*qdot[0]+Jacobian[0][1]*qdot[1];
        eeVelocity[1]=Jacobian[1][0]*qdot[0]+Jacobian[1][1]*qdot[1];
        return eeVelocity;
    }

    std::vector<double> getJointTorque(std::vector<double> q,std::vector<double> f)
    {
        std::vector<double> torque;
        torque.resize(2);
        TransposedJacobian[0][0] =-link[0]*sin(q[0])-link[1]*sin(q[0]+q[1]);
        TransposedJacobian[0][1] = link[0]*cos(q[0])+link[1]*cos(q[0]+q[1]);
        TransposedJacobian[1][0] =-link[1]*sin(q[0]+q[1]);
        TransposedJacobian[1][1] = link[1]*cos(q[0]+q[1]);

        torque[0]=TransposedJacobian[0][0]*f[0]+TransposedJacobian[0][1]*f[1];
        torque[1]=TransposedJacobian[1][0]*f[0]+TransposedJacobian[1][1]*f[1];
        return torque;
    }

};
}
/** @}   end of Doxygen module "Environment */
}


#endif  //_KauthamOpenDEEnvironment_H
//#endif //KAUTHAM_USE_ODE
//#endif // KAUTHAM_USE_OMPL


