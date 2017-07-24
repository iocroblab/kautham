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


#if !defined(_ROBOT_H)
#define _ROBOT_H


#include <list>

#include <kautham/problem/link.h>
#include <mt/transform.h>

#include <kautham/sampling/robconf.h>
#include <Inventor/VRMLnodes/SoVRMLExtrusion.h>

#include <kautham/util/libkin/inversekinematic.h>
#include <kautham/util/libkin/constrainedkinematic.h>


using namespace std;
	
//class InverseKinematic;

namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

//! Struct progress_struct allows to inform how the problem loading is going and to abort it safely
  struct progress_struct {
      pthread_mutex_t *mutex;
      int *linksLoaded;
      bool abort;
  };

//! Struct attObj defines the transformation between an object and the robot link to which it is attached.
  struct attObj{
      attObj() {obs=NULL; link=NULL;}
      ~attObj() {obs=NULL; link=NULL;}
      Robot*        obs;
      Link*         link;
      mt::Transform trans;
      bool toLink( string linkName ){
          if (link == NULL) {
              return false;
          } else {
              return link->getName() == linkName;
          }
      }
  };

//! Class robot implements a kinematic tree with a free-flying base
  class Robot {
  private:
      string            name; //!< A descriptive name of robot
      KthReal           scale;//!< This is the global scale for all the links that compound the robot. Normally it is equal to one.
      RobWeight*        _weights; //!< Weights that affect the distance computations.
      mt::Transform     _homeTrans; //!< This is the Home Reference frame at time zero (used to calculate the spatial limits).
      SoSeparator*      visModel; //!< Visualitzation model for the path.
      SoSeparator*      collModel; //!< Collision model for the path.
      unsigned          nTrunk; //!< Number of links for the trunk in case of TREE robot
      InverseKinematic* _ikine; //!< Defines the inverse kinematics of the robot, if available.
      ConstrainedKinematic* _constrainKin; //!< Defines the constrained kinematics of the robot, if it has one.
      std::vector<RobConf>   _proposedSolution; //!< Solution path to be drawn.
      SoMFVec3f*        _graphicalPath; //!< This corresponds to translational part of the link origin selected while following the path to be drawn.
      SoSeparator*      _pathSeparator; //!< This is the SoSeparator to visualize the solution path. It is attached to the robot model.
      int               _linkPathDrawn; //!< This is the number of the link whose path will be drawn
      list<attObj>      _attachedObject; //!< List of objects attached to the robot gripper.
      KthReal           _spatialLimits[3][2]; //!< Limits of motions of the base of the robot, in world coordinates.
      KthReal           _homeLimits[3][2]; //!< Limits of motions of the base of the robot, with respect to the robot reference frame.
      std::vector<Link*>     links; //!< Vector of the robot links, starting at the base and ending at the end effector.In case of Tree robots, each branch is inserted sequentially.
      bool              _autocoll; //!< Flag that indicates if the robot is auto-colliding
      bool              _hasChanged; //!< Flag that indicates if the robot has changed its configuration. To speed up some computations.
      KthReal           *offMatrix; //!< Offset vector. If copuling is generated with PCA it contains the baricenter coordinates, otherwise 0.5.
      KthReal           **mapMatrix; //!< Matrix to compute the robot configuration from the controls. If copuling is generated with PCA it contains the scaled eignevectors.
      APPROACH          Approach;//!< It identifies the robot description method (D-H Standard/D-H Modified/urdf).
      bool              se3Enabled; //!< This attribute is true if the robot has a mobile base.
      bool              armed;//!< Flag that shows if the Robot is complete or still is under construction.
      RobConf           _homeConf;     //!< This attribute is the Home configuration of the robot.
      RobConf           _currentConf;  //!< This attribute is the current configuration of the robot.
      bool              collisionable; //!< Set to true if the robot can have collision with other robots
      Robot*            robotAttachedTo; //!< In case of obstacle, it is the robot where the obstacle is attached to. If it is not attached to any robot, it is equal to NULL
      Link*             linkAttachedTo; //!< In case of obstacle, it is the link where the obstacle is attached to. If it is not attached to any link, it is equal to NULL


  public:

    Robot(string robFile, KthReal robScale, bool useBBOX = false, progress_struct *progress = NULL); //!<  Constructor

    ~Robot();

    inline bool isArmed() {return armed;} //!< Returns true if the Robot is correctly armed

    void setMapMatrix(KthReal **MapMatrix); //!< Sets the mapMatrix.

    void setOffMatrix(KthReal *OffMatrix); //!< Sets the offMatrix.

    inline KthReal** getMapMatrix() const {return mapMatrix;} //!< Returns the mapMatrix.

    inline KthReal* getOffMatrix() const {return offMatrix;} //!< Returns the offMatrix.

    inline string getName() const {return name;} //!< Returns the robot name.

    inline RobConf* getCurrentPos(){return &_currentConf;} //!< Returns the current RobConf used to represent the SE3 position and Rn configuration

    inline RobConf* getHomePos(){ return &_homeConf;} //!< Returns the Home robot configuration

    inline KthReal* getLimits(int member){return _spatialLimits[member];} //!< Returns the limits of the robot (needed for mobile bases).

    inline KthReal getScale() const {return scale;} //!< Returns the scale.

    inline unsigned getTrunk() const {return nTrunk;} //!< Returns the number of links that compose the trunk of the kinematic tree.

    inline APPROACH getDHApproach(){return Approach;} //!< Returns the typs of D-H parameters used

    inline unsigned int getNumJoints(){return ((unsigned int)links.size()) - 1;}//!< Returns the number of joints of the robot (nlinks-1).

    inline unsigned int getNumLinks(){return (unsigned int)links.size();} //!< Returns the number of links of the robot.

    inline bool isSE3Enabled() const {return se3Enabled;} //!< retruns wether the robot has a mobile base

    inline void setSE3(bool SE3Enabled) {se3Enabled = SE3Enabled;} //!< Sets wether the robot has a mobile base

    inline bool isCollisionable() const {return collisionable;} //!< returns wether the robot is collisionable

    inline void setCollisionable(bool collEnabled) {collisionable = collEnabled;} //!< Enables/disables collisions with other robots

    inline mt::Transform& getLastLinkTransform(){ return
                                    *(((Link*)links.at(links.size()-1))->getTransformation());}

    inline mt::Transform& getLinkTransform(unsigned int numLink){
                                    if(numLink>=links.size()) numLink = links.size()-1;
                                    return
                                        *(((Link*)links.at(numLink))->getTransformation());} //!< R

    inline mt::Transform& getHomeTransform(){return *(links[0]->getTransformation());} //!< Retruns the transform of the robot base wrt the world

    inline std::vector<RobConf>& getProposedSolution(){return _proposedSolution;} //!< Returns the Proposed Solution as a vector of RobConf; for visualization purposes.

    inline void setName(string nam){name = nam;} //!< Sets the robot name.

    inline void setDHApproach(APPROACH dhA){Approach = dhA;} //!< Sets the type of D-H parameters to be used

    inline void setLinkPathDrawn(int n){_linkPathDrawn = n;}

    inline list<attObj> *getAttachedObject() {return &_attachedObject;} //!< Returns the list of the attached objects

    //! Returns the values that weight translations vs. rotations in SE3 distance computations.
    KthReal* getWeightSE3();

    //!< Returns the values that weights the motions of each link in Rn distance computations.
    std::vector<KthReal>& getWeightRn();

    //! Test for autocollision
    bool autocollision(string *message = NULL);

    //! Add link to the robot
    bool addLink(string name, string ivFile, string collision_ivFile, KthReal linkScale,
                 KthReal theta, KthReal d, KthReal a, KthReal alpha,
                 bool rotational, bool movable, KthReal low, KthReal hi, KthReal w, string parentName,
                 KthReal preTrans[] = NULL, bool useBBOX = false);

    //! Add link to the robot
    bool addLink(string name, SoSeparator *visual_model, SoSeparator *collision_model, Unit3 axis,
                 bool rotational, bool movable, KthReal low, KthReal hi, KthReal w, string parentName,
                 KthReal preTrans[], ode_element ode, bool useBBOX = false);

    //! Returns the pointer to link number i
    Link* getLink(unsigned int i);

    //! Retunrs the pointer to the link named linkName
    Link* getLink(string name);

    //! Computes direct kinematics
    bool Kinematics(RobConf *robq);

    //! Computes direct kinematics
    bool Kinematics(RobConf& robq);

    //! Computes direct kinematics
    bool Kinematics(SE3Conf& q) ;

    //! Computes direct kinematics
    bool Kinematics(RnConf& q);

    //! Computes direct kinematics
    bool Kinematics(Conf *q);

    //! Computes inverse kinematics
    RobConf& InverseKinematics(std::vector<KthReal> &target);

    //! Computes inverse kinematics
    RobConf& InverseKinematics(std::vector<KthReal> &target, std::vector<KthReal> masterconf,
                               bool maintainSameWrist);

    //! Sets the inverse kinematics to be used
    bool setInverseKinematic(INVKINECLASS type);

    //! Sets parameters inverse kinematics
    bool setInverseKinematicParameter(string name, KthReal value);

    //! Returns the inverse kinematics used
    InverseKinematic* getIkine(){return _ikine;}

    //! Sets the constrained kinematics
    bool setConstrainedKinematic(CONSTRAINEDKINEMATICS type);

    //! Sets parameters constrained kinematics
    bool setConstrainedKinematicParameter(string name, KthReal value);

    //! Returns the constrained kinematics used
    ConstrainedKinematic* getCkine(){return _constrainKin;}

    //! Computes direct constrrained kinematics
    RobConf& ConstrainedKinematics(std::vector<KthReal> &target);

    //! Sets the home position of the robot
    void setHomePos(Conf* qh);

    //! Verifies collision with an obstacke or with another robot
    bool collisionCheck(Robot *obs, string *message = NULL, std::pair<int, int> *link_element = NULL);

    //! Verifies distance with an obstacle or with another robot
    KthReal distanceCheck(Robot *rob, bool min = true);

    //! Sets the values of _spatialLimits[member]
    bool setLimits(unsigned int member, KthReal min, KthReal max);

    //! Returns a pointer to the visualitzation model
    SoSeparator* getModel();

    //! Returns a pointer to the collision model
    SoSeparator* getCollisionModel();

    //! Returns a pointer to visualize the model used for collisions
    SoSeparator* getModelFromColl();

    //! Maps from couol values to configurations.
    bool control2Pose(std::vector<KthReal> &values);

    //! Maps from control values to parameters (normalized configurations).
    bool control2Parameters(std::vector<KthReal> &control, std::vector<KthReal> &parameters);

    //! Loads the robot configuration _currentConf from the normalized values of the configuration (parameters)
    void parameter2Pose(std::vector<KthReal> &values);

    //! Depending on the type specified, retruns the SE3 config or the Rn configuration from the normalized values of the configuration (parameters)
    Conf& parameter2Conf(std::vector<KthReal> &values, CONFIGTYPE type);

    //! Retunrs the weights of the robot used in the computation of the distances
    RobWeight* getRobWeight(){return _weights;}

    //! Returns the string with the names of the DOFs, separated by |
    string getDOFNames();

    //! Sets a solution path to be visualized (corresponding to the origin of a given selected link of the robot)
    bool setProposedSolution(std::vector<RobConf*>& path);

    //! Deletes the solution path to be visualized.
    bool cleanProposedSolution();

    //! Toggles on/off the vision of the solution path.
    bool setPathVisibility(bool visible);

    //! Attachs an object to a given link, usually the end effector.
    bool attachObject(Robot* obs, uint link);

    //! Moves the attached objects.
    void moveAttachedObj();

    //! Detaches the attached object.
    bool detachObject(Robot *obs);

    //! This method returns the maximum value of the D_H parameters.
    KthReal maxDHParameter();

    //! Return if this obstcale is attachable
    inline bool isAttachable() {return (!isAttached() && (links.size() == 1));}

    //! Return if this obstcale is attached
    inline bool isAttached() {return ((robotAttachedTo != NULL) && (linkAttachedTo != NULL));}

    //! Sets the robot and the link where the obstacle is attached to
    inline void setAttachedTo(Robot* robot, Link* link) {robotAttachedTo = robot; linkAttachedTo = link;}

    //! Sets the obstacle detached
    inline void setDetached() {robotAttachedTo = NULL; linkAttachedTo = NULL;}

    //! Returns the link where the obstacle is attached to
    inline Robot* getRobotAttachedTo() {return robotAttachedTo;}

    //! Returns the link where the obstacle is attached to
    inline Link* getLinkAttachedTo() {return linkAttachedTo;}

  private:
    //! sets the Robot from a *.dh file
    bool setFromDhFile(string robFile, bool useBBOX, progress_struct *progress = NULL);

    //! sets the Robot from a *.urdf file
    bool setFromUrdfFile(string robFile, bool useBBOX, progress_struct *progress = NULL);

    //! sets the Robot from a 3D model file
    bool setFromModelFile(string robFile, bool useBBOX, progress_struct *progress = NULL);

    //! This method updates the absolute position and orientation of each link in the robot.
    void updateRobot();

    //! Recomputes the limits in the home frame
    void recalculateHomeLimits();

    //! Denormalizes the SE3 unit representation and returns the positon and the rotation (quaternion) in a single vector.
    std::vector<KthReal>  deNormalizeSE3(std::vector<KthReal> &values);

    //! Returns the diagonal of the cube defined by the home-limits of the robot
    float diagLimits();
  };

  /** @}   end of Doxygen module "Problem" */
}

#endif  //_ROBOT_H

