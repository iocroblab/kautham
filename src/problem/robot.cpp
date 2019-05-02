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

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <string>

//to solve local convertions problems
#include <locale.h>

#include <Inventor/VRMLnodes/SoVRMLExtrusion.h>

#include <mt/point3.h>
#include <mt/rotation.h>

#include <kautham/problem/urdf.h>
#include <kautham/problem/assimpImport.h>
#include <kautham/problem/robot.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/util/libkin/inversekinematic.h>
#include <kautham/util/libkin/ivkintx90.h>
#include <kautham/util/libkin/ivkintxhand.h>
#include <kautham/util/libkin/ivkinhand.h>
#include <kautham/util/libkin/ivkin2drr.h>
#include <kautham/util/libkin/ivkinUR5.h>
#include <kautham/util/libkin/ivkinyumi.h>
#include <kautham/util/libkin/ivkinkukalwr.h>
#include <kautham/util/libkin/constrainedkinematic.h>
#include <kautham/problem/ivpqpelement.h>
#include <kautham/util/kthutil/kauthamexception.h>


using namespace std;
using namespace Kautham;
using namespace pugi;

namespace Kautham {


    /*! \class Robot
*  Robots are kinematic trees composed of a trunk and branches and with a mobile base
 *     They are defined as a vector fo links and described in the *.dh or *.urdf files.
 *     A robot can have a fixed base if no controls are attached to the corresponding dof.
 *     A robot can be a free-flying object if it has a single link and has controls attached to the corresponding dof.
 *     The dof of the kinematic tree can be coupled by setting the controls that define the coupling.
 *     A dof is fixed if no control is attached to it.
*/


    /*! Constructor for a robot.
  *  \param robFile is an xml file with extension *.dh or *.urdf
  *  \param robScale is a global scale for all the links
  */
    Robot::Robot(string robFile, KthReal robScale, bool useBBOX, progress_struct *progress) {
        //set initial values
        robotAttachedTo = NULL;
        linkAttachedTo = NULL;
        collisionable = true;
        _linkPathDrawn = -1;
        scale = robScale;
        armed = false;
        se3Enabled = false;
        _autocoll = false;
        _hasChanged = false;
        //Inialization of SE3 configurations.
        _homeConf.setSE3();
        _currentConf.setSE3();
        _ikine = NULL;
        _constrainKin = NULL;
        _weights = NULL;
        visModel = NULL;
        collModel = NULL;
        _graphicalPath = NULL;
        mapMatrix = NULL;
        offMatrix = NULL;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                _homeLimits[i][j] = 0.;
                _spatialLimits[i][j] = 0.;
            }
        }

        //open the file
        fstream fin;
        fin.open(robFile.c_str(),ios::in);
        if (fin.is_open()){// The file already exists.
            fin.close();

            string extension = robFile.substr(robFile.find_last_of(".")+1);

            if (extension == "dh") {
                armed = setFromDhFile(robFile,useBBOX,progress);
            } else if (extension == "urdf") {
                armed = setFromUrdfFile(robFile,useBBOX,progress);
            } else {
                armed = setFromModelFile(robFile,useBBOX,progress);
            }

            if (armed) {
                //Initialize the limits to 0.
                for(int i=0; i<3; i++)
                    _spatialLimits[i][0] = _spatialLimits[i][1] = (KthReal) 0.0;

                //Set nTrunk
                if (links.size() > 0) {
                    Link *link = links.at(0);//starting from the base
                    nTrunk = 1;
                    bool trunk_end = false;
                    while (!trunk_end && nTrunk < links.size()) {
                        if (link->numChilds() > 1) {
                            //trunk's end was found
                            trunk_end = true;
                        } else {
                            //load next link
                            link = link->getChild(0);
                            nTrunk++;
                        }
                    }
                }
            }
        } else {// File does not exists
            fin.close();
            cout << "Robot file: " << robFile << "doesn't exist. Please confirm it." << endl;
            string message = "Robot file " +robFile + " couldn't be found";
            throw KthExcp(message);
        }
    }


    Robot::~Robot() {
        delete _weights;
        delete _ikine;
        delete _constrainKin;
        for (vector<Link*>::iterator it = links.begin(); it != links.end(); ++it) {
            delete *it;
        }
        delete offMatrix;
        delete mapMatrix;
    }


    bool getLinePosition(string filename, unsigned int offset, unsigned int &line, unsigned int &position) {
        ifstream file(filename.c_str());
        if (!file.is_open()) return false;

        unsigned int counter = 0;
        line = 0;
        position = 0;
        string str;
        while(counter < offset) {
            getline(file,str);
            if (counter + str.length() > offset) {
                position = offset - counter;
            } else {
                line++;
            }
            counter += str.length();
        }

        file.close();

        return true;
    }


    bool Robot::setFromDhFile(string robFile, bool useBBOX, progress_struct *progress) {
        //setting numeric parameter to avoid convertions problems
        char *old = setlocale(LC_NUMERIC, "C");

        string dir = robFile.substr(0,robFile.find_last_of("/")+1);
        string tmpString = "";

        // Opening the file with the new pugiXML library.
        xml_document doc;

        //Parse the rob file
        xml_parse_result result = doc.load_file(robFile.c_str());
        if (result) {
            //Robot Name
            name = doc.child("Robot").attribute("name").as_string();

            //Type of convention used to define the robot, in case of Chain or Tree
            if (tmpString != "Freeflying") {
                tmpString = doc.child("Robot").attribute("DHType").as_string();
                if (tmpString == "Standard") {
                    Approach = DHSTANDARD;
                } else {
                    Approach = DHMODIFIED;
                }
            }


            //Links of the robot
            int numLinks = doc.child("Robot").child("Joints").attribute("size").as_int();

            // Initialization of the RnConf part of the RobConf for each
            // special poses, the initial, the goal, the Home and the current poses.
            _homeConf.setRn( numLinks - 1 );
            _currentConf.setRn( numLinks - 1 );

            _weights = new RobWeight( numLinks - 1 );

            xml_node linkNode = doc.child("Robot").child("Joints");
            KthReal* preTransP = NULL;
            KthReal preTrans[7] = {0., 0., 0., 0., 0., 0., 0.};
            int i = 0;
            KthReal limMin = 0.;
            KthReal limMax = 0.;

            //Loop for all the links
            for (xml_node_iterator it = linkNode.begin(); it != linkNode.end(); ++it) {
                if (progress != NULL) {
                    pthread_mutex_lock(progress->mutex);
                    if (progress->abort) {
                        pthread_mutex_unlock(progress->mutex);
                        return false;
                    }
                }

                try {
                    //Sets the Pretransfomation needed when DH parameters are not able to secify the transform between links
                    //used for instance in the definition of the transofrms between tha palm and the finger bases in the SAH hand
                    xml_node preTNode = (*it).child("PreTrans");
                    if (preTNode != NULL) {
                        preTransP = preTrans;
                        preTrans[0] = (KthReal)preTNode.attribute("X").as_double();
                        preTrans[1] = (KthReal)preTNode.attribute("Y").as_double();
                        preTrans[2] = (KthReal)preTNode.attribute("Z").as_double();
                        preTrans[3] = (KthReal)preTNode.attribute("WX").as_double();
                        preTrans[4] = (KthReal)preTNode.attribute("WY").as_double();
                        preTrans[5] = (KthReal)preTNode.attribute("WZ").as_double();
                        preTrans[6] = (KthReal)preTNode.attribute("TH").as_double();
                    }

                    //Sets the limits of the joint
                    limMin = (KthReal)(*it).child("Limits").attribute("Low").as_double();
                    limMax = (KthReal)(*it).child("Limits").attribute("Hi").as_double();

                    KthReal linkScale = 1.;
                    if ((*it).attribute("scale")) linkScale = (*it).attribute("scale").as_double();

                    //Create the link
                    if ((*it).attribute("collision_ivFile")) {
                        addLink((*it).attribute("name").as_string(), dir + (*it).attribute("ivFile").as_string(),
                                dir + (*it).attribute("collision_ivFile").as_string(), linkScale,
                                (KthReal)(*it).child("DHPars").attribute("theta").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("d").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("a").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("alpha").as_double(),
                                (*it).child("Description").attribute("rotational").as_bool(),
                                (*it).child("Description").attribute("movable").as_bool(),
                                limMin, limMax,
                                (KthReal)(*it).child("Weight").attribute("weight").as_double(),
                                (*it).child("Parent").attribute("name").as_string(), preTransP, useBBOX);
                    } else {
                        addLink((*it).attribute("name").as_string(), dir + (*it).attribute("ivFile").as_string(),
                                dir + (*it).attribute("ivFile").as_string(), linkScale,
                                (KthReal)(*it).child("DHPars").attribute("theta").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("d").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("a").as_double(),
                                (KthReal)(*it).child("DHPars").attribute("alpha").as_double(),
                                (*it).child("Description").attribute("rotational").as_bool(),
                                (*it).child("Description").attribute("movable").as_bool(),
                                limMin, limMax,
                                (KthReal)(*it).child("Weight").attribute("weight").as_double(),
                                (*it).child("Parent").attribute("name").as_string(), preTransP, useBBOX);
                    }

                    if (!((Link*)links.at(i))->isArmed()) {
                        //restoring environtment values
                        setlocale(LC_NUMERIC,old);
                        return false;
                    }

                    //Add the weight. Defaults to 1.0
                    if (i > 0) { //First link is ommited because it is the base.
                        if ((*it).child("Weight")) {
                            _weights->setRnWeigh(i-1,(KthReal)(*it).child("Weight").attribute("weight").as_double());
                        } else {
                            _weights->setRnWeigh(i-1,(KthReal)1.0);
                            links[i]->setWeight(1.0); //defaulted to 1, if not added it is put to 0 in the creator!
                        }
                    }

                    i++;

                    preTransP = NULL ;//initialize for the next link

                    if (progress != NULL) {
                        ++(*(progress->linksLoaded));
                    }
                } catch (const KthExcp& excp) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw excp;
                    return false;
                } catch (const exception& excp) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw excp;
                    return false;
                } catch (...) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw;
                    return false;
                }

                if (progress != NULL) {
                    pthread_mutex_unlock(progress->mutex);
                }
            }

            //restoring environtment values
            setlocale(LC_NUMERIC,old);
            return true;
        } else {// the result of the file parser is bad
            cout << "Robot file: " << robFile << " can not be read." << endl;

            //restoring environtment values
            setlocale(LC_NUMERIC,old);

            unsigned int line, position;
            getLinePosition(robFile,result.offset,line,position);
            string message = "Robot file " + robFile + " couldn't be parsed";
            stringstream details;
            details << "Error: " << result.description() << endl <<
                       "Last successfully parsed character: " << result.offset
                    << "(line: " <<  line << ", position: " << position << ")";
            throw KthExcp(message,details.str());
            return false;
        }
    }


    bool Robot::setFromUrdfFile(string robFile, bool useBBOX, progress_struct *progress) {
        //setting numeric parameter to avoid convertions problems
        char *old = setlocale(LC_NUMERIC, "C");

        string dir = robFile.substr(0,robFile.find_last_of("/")+1);

        // Opening the file with the new pugiXML library.
        xml_document doc;

        //Parse the urdf file
        xml_parse_result result = doc.load_file(robFile.c_str());
        if (result) {
            //if file could be loaded
            urdf_robot robot;
            xml_node tmpNode = doc.child("robot"); //node containing robot information

            robot.fill(&tmpNode,dir); //fill robot information

            //Robot Name
            name = robot.name;

            Approach = URDF;

            //Links of the robot
            int numLinks = robot.num_links;

            // Initialization of the RnConf part of the RobConf for each
            // special poses, the initial, the goal, the Home and the current poses.
            _homeConf.setRn(numLinks - 1);
            _currentConf.setRn(numLinks - 1);

            _weights = new RobWeight(numLinks - 1);

            KthReal* preTransP;
            KthReal preTrans[7]= {0., 0., 0., 0., 0., 0., 0.};
            KthReal limMin, limMax;
            ode_element ode;

            //Loop for all the links
            for (int i = 0; i < numLinks; i++) {
                if (progress != NULL) {
                    pthread_mutex_lock(progress->mutex);
                    if (progress->abort) {
                        pthread_mutex_unlock(progress->mutex);
                        return false;
                    }
                }

                try {
                    preTransP = NULL;
                    limMin = 0;
                    limMax = 0.;

                    //Sets the Pretransfomation from parent's frame to joint's frame
                    if (i > 0) {
                        preTransP = preTrans;
                        preTrans[0] = (KthReal)robot.link[i].origin.xyz[0];//x translation
                        preTrans[1] = (KthReal)robot.link[i].origin.xyz[1];//y translation
                        preTrans[2] = (KthReal)robot.link[i].origin.xyz[2];//z translation
                        preTrans[3] = (KthReal)robot.link[i].origin.r;//roll rotation
                        preTrans[4] = (KthReal)robot.link[i].origin.p;//yaw rotation
                        preTrans[5] = (KthReal)robot.link[i].origin.y;//pitch rotation
                        preTrans[6] = 0.;//unused parameter
                    }

                    //Sets the limits of the joint
                    if (robot.link[i].type == "continuous") {
                        robot.link[i].type = "revolute";
                        robot.link[i].limit.lower = -2*M_PI;
                        robot.link[i].limit.upper = +2*M_PI;
                    }
                    limMin = (KthReal)robot.link[i].limit.lower;
                    limMax = (KthReal)robot.link[i].limit.upper;

                    //Set ode parameters
                    ode.dynamics.damping = robot.link[i].dynamics.damping;
                    ode.dynamics.friction = robot.link[i].dynamics.friction;
                    ode.inertial.inertia.ixx = robot.link[i].inertial.inertia.ixx;
                    ode.inertial.inertia.ixy = robot.link[i].inertial.inertia.ixy;
                    ode.inertial.inertia.ixz = robot.link[i].inertial.inertia.ixz;
                    ode.inertial.inertia.iyy = robot.link[i].inertial.inertia.iyy;
                    ode.inertial.inertia.iyz = robot.link[i].inertial.inertia.iyz;
                    ode.inertial.inertia.izz = robot.link[i].inertial.inertia.izz;
                    ode.inertial.inertia.matrix = robot.link[i].inertial.inertia.matrix;
                    ode.inertial.mass = robot.link[i].inertial.mass;
                    ode.inertial.origin.xyz = robot.link[i].inertial.origin.xyz;
                    ode.inertial.origin.r = robot.link[i].inertial.origin.r;
                    ode.inertial.origin.p = robot.link[i].inertial.origin.p;
                    ode.inertial.origin.y = robot.link[i].inertial.origin.y;
                    ode.inertial.origin.transform = robot.link[i].inertial.origin.transform;
                    ode.contact_coefficients.mu = robot.link[i].contact_coefficients.mu;
                    ode.contact_coefficients.kp = robot.link[i].contact_coefficients.kp;
                    ode.contact_coefficients.kd = robot.link[i].contact_coefficients.kd;
                    ode.limit.effort = robot.link[i].limit.effort;
                    ode.limit.velocity = robot.link[i].limit.velocity;
                    ode.limit.lower = robot.link[i].limit.lower;
                    ode.limit.upper = robot.link[i].limit.upper;

                    //Create the link
                    addLink(robot.link[i].name,robot.link[i].visual.model,
                            robot.link[i].collision.model,
                            robot.link[i].axis,robot.link[i].type == "revolute",
                            robot.link[i].type != "fixed",limMin,
                            limMax,robot.link[i].weight,robot.link[i].parent,preTransP,ode,useBBOX);

                    if (!((Link*)links.at(i))->isArmed()) {
                        //restoring environtment values
                        setlocale(LC_NUMERIC,old);
                        return false;
                    }

                    //Add the weight. Defaults to 1.0
                    if (i > 0) { //First link is ommited because it is the base.
                        _weights->setRnWeigh(i-1,(KthReal)robot.link[i].weight);
                    }

                    if (progress != NULL) {
                        ++(*(progress->linksLoaded));
                    }
                } catch (const KthExcp& excp) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw excp;
                    return false;
                } catch (const exception& excp) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw excp;
                    return false;
                } catch (...) {
                    if (progress != NULL) {
                        pthread_mutex_unlock(progress->mutex);
                    }

                    throw;
                    return false;
                }

                if (progress != NULL) {
                    pthread_mutex_unlock(progress->mutex);
                }
            }

            //restoring environtment values
            setlocale(LC_NUMERIC,old);
            return true;
        } else {// the result of the file parser is bad
            cout << "Robot file: " << robFile << " can not be read." << endl;

            //restoring environtment values
            setlocale(LC_NUMERIC,old);

            unsigned int line, position;
            getLinePosition(robFile,result.offset,line,position);
            string message = "Robot file " + robFile + " couldn't be parsed";
            stringstream details;
            details << "Error: " << result.description() << endl <<
                       "Last successfully parsed character: " << result.offset
                    << "(line: " <<  line << ", position: " << position << ")";
            return false;
        }
    }


    bool Robot::setFromModelFile(string robFile, bool useBBOX, progress_struct *progress) {
        char *old = setlocale(LC_NUMERIC, "C");

        //Robot Name
        int pos = robFile.find_last_of("/")+1;
        name = robFile.substr(pos,robFile.find_last_of(".")-pos);

        // Initialization of the RnConf part of the RobConf for each
        // special poses, the initial, the goal, the Home and the current poses.
        _homeConf.setRn(0);
        _currentConf.setRn(0);

        _weights = new RobWeight(0);

        if (progress != NULL) {
            pthread_mutex_lock(progress->mutex);
            if (progress->abort) {
                pthread_mutex_unlock(progress->mutex);
                return false;
            }
        }

        try {
            addLink("Base", robFile, robFile, 1, 0, 0, 0, 0, false, true, 0, 0, 1, "", NULL, useBBOX);

            if (progress != NULL) {
                ++(*(progress->linksLoaded));
            }
        } catch (const KthExcp& excp) {
            if (progress != NULL) {
                pthread_mutex_unlock(progress->mutex);
            }

            throw excp;
            return false;
        } catch (const exception& excp) {
            if (progress != NULL) {
                pthread_mutex_unlock(progress->mutex);
            }

            throw excp;
            return false;
        } catch (...) {
            if (progress != NULL) {
                pthread_mutex_unlock(progress->mutex);
            }

            throw;
            return false;
        }
        if (progress != NULL) {
            pthread_mutex_unlock(progress->mutex);
        }

        //restoring environtment values
        setlocale(LC_NUMERIC,old);

        return (((Link*)links.at(0))->isArmed());
    }



    /*!
 * Computes the transform between an object and a link that will be used to
 * update the object pose when the link is moved.
 * \param obs is a pointer to the obstacle to be attached.
 * \param linkName is the name of the link to which the object is to be attached.
*/
    bool Robot::attachObject(Robot* obs, uint link){
        try{
            if (obs == NULL) return false;
            if (!obs->isAttachable()) {
                std::cout<<"Robot::attachObject failed: obs not attachable"<<std::endl;
                return false;
            }
            if (link >= links.size()) {
                std::cout<<"Robot::attachObject failed: link out of limits"<<std::endl;
                return false;
            }
            if (collisionCheck(obs)) {
                std::cout<<"Robot::attachObject failed: in collision"<<std::endl;
                return false;
            }
            attObj newObj;
            newObj.obs = obs;
            newObj.link = links.at(link);
            mt::Transform tmpO;
            KthReal* pos = obs->getLink(0)->getElement()->getPosition();
            tmpO.setTranslation( mt::Point3(pos[0], pos[1], pos[2] ) );
            pos = obs->getLink(0)->getElement()->getOrientation();
            tmpO.setRotation( mt::Rotation( pos[0], pos[1], pos[2], pos[3] ) );
            newObj.trans = newObj.link->getTransformation()->inverse() * tmpO;
            _attachedObject.push_back( newObj );
            obs->setAttachedTo(this,newObj.link);
            return true;
        }catch(...){
            return false;
        }
    }

    /*!
   * The returned value gives a rough idea of the dimension of the links.
   */
    KthReal Robot::maxDHParameter(){
        KthReal maxim=0.;
        for(size_t i = 0; i < links.size(); ++i){
            maxim = getLink(i)->getA() > maxim ? getLink(i)->getA(): maxim;
            maxim = getLink(i)->getD() > maxim ? getLink(i)->getD(): maxim;
        }
        return maxim;
    }


    void Robot::setMapMatrix(KthReal **MapMatrix) {
        if (mapMatrix != NULL) delete mapMatrix;
        mapMatrix = MapMatrix;
    }

    void Robot::setOffMatrix(KthReal *OffMatrix) {
        if (offMatrix != NULL) delete offMatrix;
        offMatrix = OffMatrix;
    }

    /*! Processes the _attachedObj list to calculate the new position and
  * orientation based on the position and orientation of the robot link where the object is attached
  * and the mt::Transform calculated on the attached instant.
  */
    void Robot::moveAttachedObj(){
        KthReal pos[3]={0.};
        KthReal ori[4]={0.};
        list<attObj>::iterator it = _attachedObject.begin();
        for( it = _attachedObject.begin(); it != _attachedObject.end(); ++it){
            mt::Transform tmp = *((*it).link->getTransformation());
            tmp *= (*it).trans;
            pos[0] = tmp.getTranslation().at(0);
            pos[1] = tmp.getTranslation().at(1);
            pos[2] = tmp.getTranslation().at(2);

            ori[0] = tmp.getRotation().at(0);
            ori[1] = tmp.getRotation().at(1);
            ori[2] = tmp.getRotation().at(2);
            ori[3] = tmp.getRotation().at(3);

            SE3Conf newconf;
            std::vector<float> newpos;
            newpos.push_back(pos[0]);
            newpos.push_back(pos[1]);
            newpos.push_back(pos[2]);
            std::vector<float> newori;
            newori.push_back(ori[0]);
            newori.push_back(ori[1]);
            newori.push_back(ori[2]);
            newori.push_back(ori[3]);

            newconf.setPos(newpos);
            newconf.setOrient(newori);

            (*it).obs->setHomePos(&newconf);
        }

    }

    /*!
  *
  */
    bool Robot::detachObject(Robot* obs) {
        bool found = false;
        list<attObj>::iterator it = _attachedObject.begin();
        for( it = _attachedObject.begin(); it != _attachedObject.end(); ++it){
            if((*it).obs == obs ){
                _attachedObject.erase(it);
                obs->setDetached();
                found = true;
                break;
            }
        }
        return found;
    }

    /*!
  *
  */
    Link* Robot::getLink(string name ){
        for (size_t i = 0; i < links.size(); ++i) {
            if (links.at(i)->getName() == name) return links.at(i);
        }
        return NULL;
    }


    /*!
  *
  */
    string Robot::getDOFNames(){
        string tmp = "X|Y|Z|X1|X2|X3";
        for (unsigned int i = 1; i< links.size(); i++){
            tmp.append("|");
            tmp.append(links[i]->getName());
        }
        return tmp;
    }


    /*!
  * Allows to change the values of the robot base limits.
  * These values are in the world frame.
  *   \param member is an integer between 0 and 3 corresponding to three coordinates
  *    of position
  */
    bool Robot::setLimits(unsigned int member, KthReal min, KthReal max){
        if (member < 3 && min <= max) {
            _spatialLimits[member][0] = min;
            _spatialLimits[member][1] = max;

            //Recalculate the limits in home frame.
            recalculateHomeLimits();

            return true;
        }
        return false;
    }

    /*!
  *
  */
    void Robot::recalculateHomeLimits(){
        mt::Transform homeInv = _homeTrans.inverse();
        mt::Point3 Pmin_W(_spatialLimits[0][0], _spatialLimits[1][0], _spatialLimits[2][0]);
        mt::Point3 Pmax_W(_spatialLimits[0][1], _spatialLimits[1][1], _spatialLimits[2][1]);
        mt::Point3 Pmin_H(homeInv*Pmin_W);
        mt::Point3 Pmax_H(homeInv*Pmax_W);

        for (unsigned int i = 0; i < 3; ++i) {
            if (Pmin_H[i] == Pmax_H[i]) {
                _homeLimits[i][0] = 0.;
                _homeLimits[i][1] = 0.;
            } else {
                _homeLimits[i][0] = Pmin_H[i];
                _homeLimits[i][1] = Pmax_H[i];
            }
        }
    }


    /*!
  *
  */
    bool Robot::setConstrainedKinematic(CONSTRAINEDKINEMATICS type){
        switch(type){
            case Kautham::UNCONSTRAINED:
                _constrainKin = NULL;
                break;

            default:
                cout << "The Constrained Kinematic model has not be configured properly.\n" <<
                        "See the ConsBronchoscopyKin of the Robot class to call the constructor. " << endl;
                _constrainKin = NULL;
            return false;
        }
        return true;
    }

    /*!
  *
  */
    bool Robot::setConstrainedKinematicParameter(string name, KthReal value){
        if( _constrainKin != NULL)
            return _constrainKin->setParameter(name, value);
        return false;
    }

    /*!
  *
  */
    RobConf& Robot::ConstrainedKinematics(vector<KthReal> &target){
        if(_constrainKin != NULL){
            //First i try to connect to the remote object
            //if this pointer is not null, the object has been instantiate correctly.
            _constrainKin->setTarget(target);
            if(_constrainKin->solve()){
                return _constrainKin->getRobConf();
            }
        }
        return _currentConf;
    }

    /*!
  *
  */
    bool Robot::setInverseKinematic(INVKINECLASS type){
        switch(type){
            case Kautham::RR2D:
                _ikine = new IvKin2DRR(this);
            break;
            case Kautham::TX90:
                _ikine = new IvKinTx90(this);
            break;
            case Kautham::HAND:
                _ikine = new IvKinHand(this);
            break;
            case Kautham::TX90HAND:
                _ikine = new IvKinTxHand(this);
            break;
            case Kautham::UR5:
                _ikine = new IvKinUR5(this);
            break;
            case Kautham::YUMI_RIGHT:
                _ikine = new IvKinYumi(this,0);
            break;
            case Kautham::YUMI_LEFT:
                _ikine = new IvKinYumi(this,1);
            break;
            case Kautham::KUKA_LWR:
                _ikine = new IvKinKukaLWR(this);
            break;
            case Kautham::NOINVKIN:
                _ikine = NULL;
            break;
            case Kautham::UNIMPLEMENTED:
                cout << "The new Inverse Kinematic model have not been configured properly.\n" <<
                        "See the setInverseKinematic of the Robot class to call the constructor. " << endl;
                _ikine = NULL;
            return false;
        }
        return true;
    }

    /*!
  *
  */
    bool Robot::setInverseKinematicParameter(string name, KthReal value){
        if( _ikine != NULL)
            return _ikine->setParameter(name, value);
        return false;
    }

    /*!
  *
  */
    bool Robot::Kinematics(RobConf *robq) {
        bool se = Kinematics(robq->getSE3());
        if(se && Kinematics(robq->getRn()))
            return true;
        else
            return false;
    }

    /*!
  *
  */
    bool Robot::Kinematics(RobConf& robq) {
        bool se = Kinematics(robq.getSE3());
        if(se && Kinematics(robq.getRn()))
            return true;
        else
            return false;
    }

    /*!
  *
  */
    bool Robot::Kinematics(SE3Conf& q) {
        vector<KthReal>& coor = q.getCoordinates();

        links[0]->getTransformation()->setTranslation(Point3(coor[0],coor[1], coor[2]));
        links[0]->getTransformation()->setRotation(Rotation(coor[3], coor[4], coor[5], coor[6]));
        links[0]->forceChange(NULL);

        _currentConf.setSE3(q);
        updateRobot();
        return true;
    }

    /*!
  *
  */
    bool Robot::Kinematics(RnConf& q) {

        /*
   if(q.getDim() == getNumJoints()){
     for(int i = 0; i < q.getDim(); i++)
       links[i+1]->setValue(q.getCoordinate(i));

     _currentConf.setRn(q);

     updateRobot();
     return true;
   }else
     return false;
     */

        if(q.getDim() > getNumJoints()) return false;

        unsigned n = getNumJoints();
        if(q.getDim() < n){
            RnConf q2(n);
            unsigned i;
            vector<KthReal> coords(n);
            for(i = 0; i < q.getDim(); i++) coords[i]=q.getCoordinate(i);
            for(; i < n; i++) coords[i]=0.0;
            q2.setCoordinates(coords);

            for(unsigned i = 0; i < q2.getDim(); i++)
                links[i+1]->setValue(q2.getCoordinate(i));

            _currentConf.setRn(q2);
        }
        else{ //q.getDim() == getNumJoints()
            for(unsigned i = 0; i < q.getDim(); i++)
                links[i+1]->setValue(q.getCoordinate(i));

            _currentConf.setRn(q);
        }

        updateRobot();
        return true;
    }



    /*!
  * This method uses the Configuration q to set up the position, orientation,
  * and articular values if the robot has one. If the configuration q is SE2 or SE3
  * the robot change the position and/or orientation either the configuration is
  * Rn the robot change the articular values.
  */
    bool Robot::Kinematics(Conf *q) {
        vector<KthReal>& coor = q->getCoordinates();

        //cout << "q.gettype "<<q->getType()<<endl;
        switch(q->getType()){
            case SE3:
                links[0]->getTransformation()->setTranslation(Point3(coor[0],coor[1], coor[2]));
                links[0]->getTransformation()->setRotation(Rotation(coor[3], coor[4], coor[5], coor[6]));
                links[0]->forceChange(NULL);
                _currentConf.setSE3(q->getCoordinates());
                //cout<<"current conf ("<< _currentConf.getSE3()->getCoordinate(0)<<", "<< _currentConf.getSE3()->getCoordinate(1)<<
                //", "<< _currentConf.getSE3()->getCoordinate(2)<<")"<<endl;
            break;
            case SE2:
                links[0]->getTransformation()->setTranslation(Point3(coor[0],coor[1], (KthReal)0.0));
                links[0]->getTransformation()->setRotation(Rotation((KthReal)0.0, (KthReal)0.0,
                                                                    (KthReal)1.0, coor[2]));
                links[0]->forceChange(NULL);
                _currentConf.setSE3(q->getCoordinates());
            break;
            case Rn:
                if(q->getDim() == getNumJoints()){
                    for(unsigned i=0; i<q->getDim(); i++)
                        links[i+1]->setValue(q->getCoordinate(i));

                    _currentConf.setRn(q->getCoordinates());
                }
                //cout<<"current conf Link[0] = "<< _currentConf.getRn().getCoordinate(0)<<endl;
            break;
            default:
            return false;
        }
        updateRobot();
        return true;
    }

    /** This member method is the interface to calling the inverse kinematic
 *   object associated. It returns the RobConf<SE3Conf,RnConf> configuration
 *   that describe the pose of the robot completely.
 *
 * Call to inverseKinematics given the target defined as:
 * a) the tcp transform
 * b) a tcp transform and configuration parameters e.g. for the TX90: (l/r,ep/en,wp/wn)
 */
    RobConf& Robot::InverseKinematics(vector<KthReal> &target){
        if(_ikine != NULL){
            //First i try to conect to the remote object
            //if this pointer is not null, the object has been instantiate correctly.
            _ikine->setTarget(target);
            if(_ikine->solve()){
                return _ikine->getRobConf();
            }
        }
        throw InvKinEx(0);
    }

    /*!
  * Call to inverseKinematics given:
  * a) the target defined as the tcp transform
  * b) a robot configuration used as a reference to copy its same configuration
  * parameters e.g. for the TX90: (l/r,ep/en,wp/wn)
  */
    RobConf& Robot::InverseKinematics(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
        if(_ikine != NULL){
            //First i try to conect to the remote object
            //if this pointer is not null, the object has been instantiate correctly.
            _ikine->setTarget(target, masterconf, maintainSameWrist);
            if(_ikine->solve()){
                return _ikine->getRobConf();
            }
        }
        throw InvKinEx(0);
    }

    /*!
  *
  */
    bool Robot::autocollision(string *message) {
        //parameter t is used to only test the autocollision of the trunk part of a TREE robot
        //it is set to 0 in robot.h

        if(_hasChanged ){
            _autocoll = false;

            for (unsigned int i = 0; i< links.size(); i++){
                for (unsigned int j = i+2; j < links.size(); j++){
                    //collision-cheeck between a link and its children is skipped
                    if (links[j]->getParent() != links[i]) {
                        if(links[i]->getElement()->collideTo(links[j]->getElement())){
                            stringstream sstr;
                            sstr <<"Collision between links " << i << " (" << links[i]->getName()
                                << ") and " << j << " (" << links[j]->getName() << ")" << endl;
                            if (message != NULL) *message = sstr.str();
                            _autocoll = true;
                            return _autocoll;

                        }
                    }
                }
            }
        }

        return _autocoll;
    }


    /*!
  * This method verifies if this robot collides with the robot rob passed as a parameter.
  * The method returns true when the two robots collide, otherwise returns false.
  */
    bool Robot::collisionCheck(Robot *rob, string *message, std::pair<int, int> *link_element) {
        //if( _autocoll || rob->autocollision() ) return true;

        if(!collisionable || !rob->isCollisionable()) return false;

        for(unsigned i=0; i < links.size(); i++){
            for( unsigned j = 0; j < rob->getNumLinks(); j++ ){
                if( links[i]->getElement()->collideTo(rob->getLink(j)->getElement()) ) {
                    if (message != NULL) {

                        if(link_element !=NULL){
                            link_element->first = i;
                            link_element->second = j;
                        }

                        stringstream sstr;
                        sstr << "Collision between links " << i << " (" << links[i]->getName()
                             << ") and " << j << " (" << rob->getLink(j)->getName() << ")" << endl;
                        *message = sstr.str();
                    }
                    return true;
                }
            }
        }
        return false;
    }


    /*!
  * This methods returns the distance between the robot and the obstacle or robot passed as parameter.
  * The distance returned is between the endEfector or the most distal link, but
  * if parameter min is true, the distance is the minimum distance between all the links
  */
    KthReal Robot::distanceCheck(Robot *rob, bool min ){
        KthReal minDist;
        KthReal tempDist;

        if(autocollision()) {
            return 0.;
        } else {
            // First the most distal link
            minDist = links[links.size()-1]->getElement()->getDistanceTo(rob->getLink(0)->getElement());
        }
        if( min ){
            for(int i = links.size(); i > 0 ; --i) {
                for(int j = rob->getNumLinks(); j > 0; --j){
                    tempDist = links[i-1]->getElement()->getDistanceTo(rob->getLink(j-1)->getElement());
                    if(minDist > tempDist) {
                        minDist = tempDist;
                    }
                }
            }
        }

        return minDist;
    }

    /*!
  *
  */
    void Robot::setHomePos(Conf* qh){
        if( qh != NULL ){
            switch(qh->getType()){
                case SE3:
                case SE2:
                {
                    _homeConf.setSE3(qh->getCoordinates());
                    mt::Point3 tempTran(qh->getCoordinate(0),qh->getCoordinate(1),qh->getCoordinate(2));
                    mt::Rotation tempRot(qh->getCoordinate(3),qh->getCoordinate(4),
                                         qh->getCoordinate(5),qh->getCoordinate(6));

                    _homeTrans.setRotation(tempRot);
                    _homeTrans.setTranslation(tempTran);

                    recalculateHomeLimits();
                }
                break;
                case Rn:
                    if(qh->getDim() == links.size() - 1){
                        _homeConf.setRn(qh->getCoordinates());
                    }
            }
            Kinematics(qh);

        }
    }


    //! This method builds the link model and all their data structures in order
    //! to keep the coherence in the robot assembly. It doesn't use any intermediate
    //! structure to adquire the information to do the job.
    bool Robot::addLink(string name, string ivFile, string collision_ivFile, KthReal linkScale, KthReal theta,
                        KthReal d,KthReal a, float alpha, bool rotational,
                        bool movable, KthReal low, KthReal hi, float w, string parentName, float preTrans[], bool useBBOX){
        Link* temp = new Link(ivFile, collision_ivFile, scale*linkScale, Approach, useBBOX);
        temp->setName(name);
        temp->setMovable(movable && (low != hi));
        temp->setRotational(rotational);
        temp->setDHPars(theta, scale*d, scale*a, alpha);
        if (rotational) {
            temp->setLimits(low, hi);
        } else {
            temp->setLimits(scale*low, scale*hi);
        }
        temp->setWeight(w);
        if(preTrans != NULL)
            temp->setPreTransform(scale*preTrans[0],scale*preTrans[1],scale*preTrans[2], preTrans[3],
                    preTrans[4], preTrans[5], preTrans[6]);
        if(links.size() > 0 ){
            // There are finding the link by the name
            for(int i = links.size()-1; i >= 0 ; i--)
                if(parentName == links[i]->getName()){
                    temp->setParent(links[i]);
                    break;
                }
        }

        temp->setArmed();
        temp->setValue(0.0); //This is the home position
        links.push_back(temp);
        return true;
    }

    //! This method builds the link model and all their data structures in order
    //! to keep the coherence in the robot assembly. It doesn't use any intermediate
    //! structure to adquire the information to do the job.
    bool Robot::addLink(string name, SoSeparator *visual_model, SoSeparator *collision_model, Unit3 axis, bool rotational, bool movable,
                        KthReal low, KthReal hi, KthReal w, string parentName, KthReal preTrans[], ode_element ode, bool useBBOX){

        if (visual_model == NULL) {
            throw KthExcp("Link " + name + " from robot " + this->name + " has no visual model");
        }

        Link* temp = new Link(visual_model, collision_model, scale, Approach, useBBOX);
        temp->setName(name);
        temp->setMovable(movable && (low != hi));
        temp->setRotational(rotational);
        temp->setAxis(axis);
        temp->setDHPars(0., 0., 0., 0.);//defaults to zero
        if (rotational) {
            temp->setLimits(low, hi);
        } else {
            temp->setLimits(this->scale*low, this->scale*hi);
        }
        temp->setWeight(w);
        temp->setOde(ode);
        if(preTrans != NULL)
            temp->setPreTransform(this->scale*preTrans[0],this->scale*preTrans[1],this->scale*preTrans[2], preTrans[3],
                    preTrans[4], preTrans[5], 0.);
        if(links.size() > 0 ){
            // There are finding the link by the name
            for(int i = links.size()-1; i >= 0 ; i--)
                if(parentName == links[i]->getName()){
                    temp->setParent(links[i]);
                    break;
                }
        }

        temp->setArmed();
        temp->setValue(0.0); //This is the home position
        links.push_back(temp);
        return true;
    }


    //! This function return a pointer to the Link demanded.
    /*! If you pass a Link identification not valid it will return a null pointer.*/
    Link* Robot::getLink(unsigned int i) {
        if(i<links.size())
            return links[i];
        else
            return NULL;
    }

    /*!
  *
  */
    SoSeparator* Robot::getModel(){
        if (visModel == NULL) {
            SoSeparator *robot = new SoSeparator();
            robot->ref();
            for (unsigned int i = 0; i < links.size(); ++i) {
                robot->addChild(links[i]->getModel(true));
            }

            // Now the three dimensional proposed path for the last link is added
            _pathSeparator = new SoSeparator();
            _pathSeparator->ref();
            _pathSeparator->setName("Path");
            SoMaterial*  tmpMat = new SoMaterial();
            tmpMat->diffuseColor.setValue( 0., 0., 1.);
            _pathSeparator->addChild( tmpMat );
            SoVRMLExtrusion* tmpVRML = new SoVRMLExtrusion();
            tmpVRML->solid.setValue(true);
            float diag = diagLimits()/100.;
            //diag = diag < 2. ? 2. : diag;
            tmpVRML->scale.setValue( diag,diag );
            float vertex[13][2];
            vertex[0][0] = 0.1000;    vertex[0][1] = 0.;
            vertex[1][0] = 0.0866;    vertex[1][1] = 0.0500;
            vertex[2][0] = 0.0500;    vertex[2][1] = 0.0866;
            vertex[3][0] = 0.0000;    vertex[3][1] = 0.1000;
            vertex[4][0] = -0.0500;   vertex[4][1] = 0.0866;
            vertex[5][0] = -0.0866;   vertex[5][1] = 0.0500;
            vertex[6][0] = -0.1000;   vertex[6][1] = 0.0000;
            vertex[7][0] = -0.0866;   vertex[7][1] = -0.0500;
            vertex[8][0] = -0.0500;   vertex[8][1] = -0.0866;
            vertex[9][0] = -0.0000;   vertex[9][1] = -0.1000;
            vertex[10][0] = 0.0500;   vertex[10][1] = -0.0866;
            vertex[11][0] = 0.0866;   vertex[11][1] = -0.0500;
            vertex[12][0] = 0.1000;   vertex[12][1] = 0.;
            tmpVRML->crossSection.setValues(0,13,vertex);
            _graphicalPath = new SoMFVec3f();
            tmpVRML->spine.connectFrom(_graphicalPath);
            _pathSeparator->addChild(tmpVRML);

            //robot->addChild(_pathSeparator);
            visModel = robot;
        }
        return visModel;
    }


    /*!
  *
  */
    SoSeparator* Robot::getCollisionModel(){
        if (collModel == NULL) {
            SoSeparator *robot = new SoSeparator();
            robot->ref();
            for (unsigned int i = 0; i < links.size(); ++i) {
                robot->addChild(links[i]->getCollisionModel(true));
            }

            collModel = robot;
        }
        return collModel;
    }


    /*!
  *
  */
    KthReal* Robot::getWeightSE3(){
        if( _weights != NULL )
            return _weights->getSE3Weight();
        else
            throw runtime_error("Weights have not been initialized");
    }

    /*!
  *
  */
    vector<KthReal>& Robot::getWeightRn(){
        if( _weights != NULL )
            return _weights->getRnWeights();
        else
            throw runtime_error("Weights have not been initialized");
    }

    /*!
  *
  */
    float Robot::diagLimits(){
        float tmp;
        float dia = 0.;
        for (unsigned int i = 0; i < 3; ++i) {
            tmp = _homeLimits[i][1]-_homeLimits[i][0];
            dia += tmp*tmp;
        }
        return sqrt(dia);
    }

    SoSeparator* Robot::getModelFromColl(){
        SoSeparator* root = new SoSeparator;
        for (unsigned int i = 0; i < links.size(); ++i) {
            root->addChild(links[i]->getModelFromColl());
        }
        return root;
    }

    /*!
  * parameters = mapMaptrix*controls + offMatrix
  * the parameters define the robot configuration and their values lies in the range 0..1
  * \returns false if any of the parameters falls out of range (i.e. <0 or >1), and true otherwise
  */
    bool Robot::control2Parameters(vector<KthReal> &control, vector<KthReal> &parameters){
        bool retvalue = true;
        parameters = std::vector<KthReal>(6 + _currentConf.getRn().getDim(),0.0);

        if(se3Enabled){
            //EUROC
            //std::cout<<"HERE\n";
            for(int i =0; i < 6; i++){
                for(unsigned int j= 0; j < control.size(); j++)
                    parameters[i] += mapMatrix[i][j] * (control[j]-0.5) ;
                parameters[i] += offMatrix[i];
                if(parameters[i]<0.0 || parameters[i]>1.0) retvalue = false;
            }
        }
        if( _currentConf.getRn().getDim() != 0 ){
            for(unsigned i =0; i < _currentConf.getRn().getDim(); i++){
                for(unsigned j= 0; j < control.size(); j++) {
                    parameters[i+6] += mapMatrix[i+6][j] * (control[j]-0.5);
                    //cout << mapMatrix[i+6][j] << "\t" ;
                }
                //cout << offMatrix[i+6];
                parameters[i+6] += offMatrix[i+6];

                if(parameters[i+6]<0.0 || parameters[i+6]>1.0) retvalue = false;
                //cout << endl;
            }
        }
        return retvalue;
    }


    /*!
  * Computes the pose of the robot (currentconf) from a vector of controls.
  * \param values is the set of controls (their value lies in the range 0..1)
  * \returns true if the controls make the robot be places in a configuration within bounds
  * and false otherwise (in this case it is located at the border by the parameter2pose function)
  */
    bool Robot::control2Pose(vector<KthReal> &values){
        bool retvalue;
        vector<KthReal> vecTmp;
        _hasChanged = true;
        retvalue = control2Parameters(values,vecTmp);
        parameter2Pose(vecTmp);
        return retvalue;
    }

    /*!
  *
  */
    vector<KthReal> Robot::deNormalizeSE3(vector<KthReal> &values){
        vector<KthReal> coords(6);
        SE3Conf tmp;

        coords[0] = values[0]*(_homeLimits[0][1]-_homeLimits[0][0]) + _homeLimits[0][0];
        coords[1] = values[1]*(_homeLimits[1][1]-_homeLimits[1][0]) + _homeLimits[1][0];
        coords[2] = values[2]*(_homeLimits[2][1]-_homeLimits[2][0]) + _homeLimits[2][0];
        coords[3] = values[3];
        coords[4] = values[4];
        coords[5] = values[5];

        tmp.setCoordinates(coords); // It is the actual position but in the Home Frame

        mt::Point3 tempTran(tmp.getCoordinate(0),tmp.getCoordinate(1),tmp.getCoordinate(2));
        mt::Rotation tempRot(tmp.getCoordinate(3),tmp.getCoordinate(4),
                             tmp.getCoordinate(5),tmp.getCoordinate(6));

        mt::Transform in_home, in_world;
        in_home.setRotation(tempRot);
        in_home.setTranslation(tempTran);

        in_world = _homeTrans * in_home;      // Obtaining it in the world frame.
        tempTran = in_world.getTranslation();
        tempRot  = in_world.getRotation();

        coords.resize(7); // Resizing is needed to use quaternions
        coords[0] = tempTran[0];
        coords[1] = tempTran[1];
        coords[2] = tempTran[2];
        coords[3] = tempRot[0];
        coords[4] = tempRot[1];
        coords[5] = tempRot[2];
        coords[6] = tempRot[3];

        return coords;
    }

    //! This method maps between [0,1] parameter values to human readable values.
    //! The values vector must have the controls size.
    Conf& Robot::parameter2Conf(vector<KthReal> &values, CONFIGTYPE type){
        vector<KthReal> coords;
        switch(type){
            case SE2:
            break;
            case SE3:
                //  First, denormalize the SE3 configuration from 6 values in the Home frame.
                coords = deNormalizeSE3(values);
                _currentConf.setSE3(coords);
            return _currentConf.getSE3();
            break;
            case Rn:
                if(_currentConf.getRn().getDim() == values.size()-6){
                    coords.resize(_currentConf.getRn().getDim());
                    for(unsigned int i = 1; i< links.size(); i++)
                        coords[i-1] = ((Link*)links[i])->parameter2Value(values[i+5]);

                    _currentConf.setRn(coords);
                    return _currentConf.getRn();
                }
            break;
        }
        throw exception();
    }


    /*!
  * Denormalizes the configuration (the parameters are values in the range 0..1), updates the values in the links
  * and stores the _currentConf variable with these denormalized values
  */
    void Robot::parameter2Pose(vector<KthReal> &values){
        if( armed /*&& numControls == values.size() */){
            _hasChanged = true;
            if(se3Enabled){
                // SE3 Denormalization in the wold frame
                vector<KthReal> coords = deNormalizeSE3(values);
                _currentConf.setSE3(coords);
                links[0]->getTransformation()->setTranslation(Point3(coords.at(0),coords.at(1), coords.at(2)));
                links[0]->getTransformation()->setRotation(Rotation(coords.at(3), coords.at(4),
                                                                    coords.at(5), coords.at(6)));
                links[0]->forceChange(NULL);

                //EUROC
                //std::cout<<"parameter2Pose - link0: "<<
                //           coords.at(0)<<" "<<coords.at(1)<<" "<<coords.at(2)<<std::endl;

            }
            if(_currentConf.getRn().getDim() != 0){
                // Rn denormalization
                vector<KthReal> coords(_currentConf.getRn().getDim());
                for(unsigned i =0; i < _currentConf.getRn().getDim(); i++){
                    links[i+1]->setParameter(values[6+i]);
                    coords[i] = ((Link*)links[i+1])->getValue();
                    //coords[i] = ((Link*)links[i+1])->parameter2Value(values[i+6]);
                }

                _currentConf.setRn(coords);
            }
            updateRobot();
        }
    }

    /*!
  *
  */
    void Robot::updateRobot(){
        for(unsigned int i =0; i < links.size(); i++){
            if(links[i]->changed()){
                links[i]->calculatePnO();
                _hasChanged = true;
            }
        }
        //EUROC
        //std::cout<<"UpdateRobot - attachedObjectSize = "<<_attachedObject.size()<<std::endl;
        if(_attachedObject.size() != 0 )
            moveAttachedObj();
    }

    /*!
  *
  */
    bool Robot::setProposedSolution(vector<RobConf*>& path){
        try{
            unsigned int i = 0;
            _proposedSolution.clear();
            for( i = 0; i < path.size(); i++)
                _proposedSolution.push_back(*(path.at(i)));

            // Updating the graphical path if needed
            if( _graphicalPath != NULL )
                _graphicalPath->deleteValues(0);

            //if no need to draw path then return
            if(_linkPathDrawn<0) return true;

            //else fill graphicalPath
            SbVec3f* temp = new SbVec3f[path.size()];

            vector<RobConf>::iterator it;
            mt::Point3 pos;
            mt::Transform trans;
            i = 0;
            for(it = _proposedSolution.begin(); it != _proposedSolution.end(); ++it){
                Kinematics(*it);
                trans = getLinkTransform(_linkPathDrawn); //draws the path of link number linkPathDrawn
                //trans = getLastLinkTransform();
                pos = trans.getTranslation();
                temp[i].setValue( pos.at(0), pos.at(1), pos.at(2));
                ++i;
            }
            if( _graphicalPath != NULL )
                _graphicalPath->setValues(0, path.size(), temp);
            delete[] temp;
            return true;
        }catch(...){
            return false;
        }
    }

    /*!
  *
  */
    bool Robot::cleanProposedSolution(){
        try{
            _proposedSolution.clear();
            if( _graphicalPath != NULL )
                _graphicalPath->deleteValues(0);
            return true;
        }catch(...){
            return false;
        }
    }

    /*!
  *
  */
    bool Robot::setPathVisibility(bool visible){
        bool response=false;
        SoNode *sepgrid = NULL;
        try{
            if( visible ){
                visModel->addChild(_pathSeparator);
                response = true;
            }else{
                sepgrid = visModel->getByName("Path");
                if( sepgrid != NULL ){
                    visModel->removeChild(sepgrid);
                }
                response = false;
            }
        }catch(...){ }

        return response;
    }

}


