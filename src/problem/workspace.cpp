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

 
#include "workspace.h"
#include <vector>
#include <typeinfo>

using namespace std;

namespace Kautham {
    WorkSpace::WorkSpace() {
        obstacles.clear();
        robots.clear();
        distVec.clear();
        _robConfigMap.clear();
        _obsConfigMap.clear();
        _robWeight.clear();
        numRobControls = 0;
        numObsControls = 0;
    }


    unsigned int WorkSpace::_countWorldCollCheck = 0;


    void WorkSpace::resetCollCheckCounter() {
        _countWorldCollCheck = 0;
    }


    unsigned int WorkSpace::getCollCheckCounter() {
        return _countWorldCollCheck;
    }


    void WorkSpace::increaseCollCheckCounter() {
        _countWorldCollCheck++;
    }


    void WorkSpace::addDistanceMapFile(string distanceFile) {
        distanceMapFile = distanceFile;
    }


    void WorkSpace::addDimensionsFile(string dFile) {
        dimensionsFile = dFile;
    }


    void WorkSpace::addDirCase(string dirc) {
        dirCase = dirc;
    }


    vector<KthReal>* WorkSpace::distanceCheck(Sample* sample) {
        vector<KthReal> tmpVec;
        tmpVec.clear();
        for(unsigned int j=0; j < getNumRobControls(); j++ )
            tmpVec.push_back(sample->getCoords()[j]);

        distVec.clear();
        for(unsigned int i=0; i< robots.size(); i++){
            robots[i]->control2Pose(tmpVec);
            for(unsigned int m = 0; m < obstacles.size(); m++){
                distVec.push_back(robots[i]->distanceCheck(obstacles[m]));
            }
        }
        return &distVec;
    }


    /*!
    * Moves the robots to the configuration specified by the sample
    *  loads the flag withinbounds of the sample. If outofbounds, one of the
    *  robots ends at the border of one or more of its limits.
    */
    void WorkSpace::moveRobotsTo(Sample* sample) {
        bool withinbounds=true;
        vector<KthReal> tmpVec;
        tmpVec.clear();
        for(unsigned int j=0; j < getNumRobControls(); j++ )
            tmpVec.push_back(sample->getCoords()[j]);

        for(unsigned int i=0; i< robots.size(); i++){
            if(sample->getMappedConf().size()==0){
                withinbounds &= robots[i]->control2Pose(tmpVec);
            }
            else{
                robots[i]->Kinematics(sample->getMappedConf().at(i));
            }
        }
        _lastRobSampleMovedTo = sample;

        //set _sample::_config if it was not set
        if(sample->getMappedConf().size()==0) sample->setMappedConf(_robConfigMap);

        sample->setwithinbounds(withinbounds);
    }


    /*!
   * Moves the obstacles to the configuration specified by the sample
   *  loads the flag withinbounds of the sample. If outofbounds, one of the
   *  obstacles ends at the border of one or more of its limits.
   */
    void WorkSpace::moveObstaclesTo(Sample *sample) {

        bool withinbounds=true;
        vector<KthReal> tmpVec;
        tmpVec.clear();
        for(unsigned int j=0; j < getNumObsControls(); j++ )
            tmpVec.push_back(sample->getCoords()[j]);

        for(unsigned int i=0; i< obstacles.size(); i++){
            if (!obstacles.at(i)->isAttached()) {
                if(sample->getMappedConf().size()==0){
                    withinbounds &= obstacles[i]->control2Pose(tmpVec);
                }
                else{
                    obstacles[i]->Kinematics(sample->getMappedConf().at(i));
                }
            }
        }
        _lastObsSampleMovedTo = sample;

        //set _sample::_config if it was not set
        if(sample->getMappedConf().size()==0) sample->setMappedConf(_obsConfigMap);

        sample->setwithinbounds(withinbounds);
    }


    void WorkSpace::setInitObsSample(Sample* initsample) {
        moveObstaclesTo(initsample);
        _initObsSample = initsample;
    }


    bool WorkSpace::collisionCheck(Sample* sample, string *message) {
        stringstream sstr;
        increaseCollCheckCounter();

        vector<KthReal> tmpVec;
        bool collision = false;
        tmpVec.clear();
        for (int j=0; j < getNumRobControls(); j++) {
            tmpVec.push_back(sample->getCoords()[j]);
        }

        if (sample->getMappedConf().size() == 0) {
            for (uint i=0; i< robots.size(); i++) {
                robots[i]->control2Pose(tmpVec);
                //first is testing if the robots collide with the environment (obstacles)
                for (uint m = 0; m < obstacles.size(); m++) {
                    string str;
                    if (robots[i]->collisionCheck(obstacles[m],&str)) {
                        collision = true;
                        sstr << "Robot " << i << " (" << robots[i]->getName()
                             << ") is in collision with obstacle " << m << " ("
                             << obstacles[m]->getName() << ")" << endl;
                        sstr << str;
                        break;
                    }
                    if (collision) break;
                }
                if (collision) break;

                //second is testing if a robot collides with another one present in the workspace
                //this validation is done with the robots validated previously
                if (i > 0) {
                    for (int k = i-1; k == 0; k--) {
                        string str;
                        if (robots[i]->collisionCheck(robots[k],&str)) {
                            collision = true;
                            sstr << "Robot " << i << " (" << robots[i]->getName()
                                 << ") is in collision with robot " << k << " ("
                                 << robots[k]->getName() << ")" << endl;
                            sstr << str;
                            break;
                        }
                    }
                    if (collision) break;
                }
                if (collision) break;

                //third is testing if a robot autocollides
                string str;
                if (robots[i]->autocollision(0,&str)) {
                    collision = true;
                    sstr << "Robot " << i << " (" << robots[i]->getName()
                         << ") is in autocollision" << endl;
                    sstr << str;
                    break;
                }
                if (collision) break;

                //fourth is testing if an attached object (1-link-obstacle)
                //collides with the environment (other obstacles)
                list<attObj> *attachedObject = ((Robot*)robots[i])->getAttachedObject();
                for(list<attObj>::iterator it = attachedObject->begin();
                    it != attachedObject->end(); ++it) {
                    for (uint m = 0; m < obstacles.size(); m++) {
                        if (it->obs != obstacles.at(m)) {
                            string str;
                            if (it->obs->collisionCheck(obstacles.at(i),&str)) {
                                collision = true;
                                sstr << "Attached object " << it->obs->getName()
                                     << " is in collision with obstacle " << m << " ("
                                     << obstacles[m]->getName() << ")" << endl;
                                sstr << str;
                                break;
                            }
                            if (collision) break;
                        }
                        if (collision) break;
                    }
                    if (collision) break;
                }
                if (collision) break;
            }
        } else {
            for (uint i=0; i< robots.size(); i++) {
                robots[i]->Kinematics(sample->getMappedConf().at(i));

                //first is testing if the robot collides with the environment (obstacles)
                for (uint m = 0; m < obstacles.size(); m++) {
                    string str;
                    if (robots[i]->collisionCheck(obstacles[m],&str)) {
                        collision = true;
                        sstr << "Robot " << i << " (" << robots[i]->getName()
                             << ") is in collision with obstacle " << m << " ("
                             << obstacles[m]->getName() << ")" << endl;
                        sstr << str;
                        break;
                    }
                    if (collision) break;
                }
                if (collision) break;

                //second is testing if a robot collides with another one present in the workspace
                //this validation is done with the robots validated previously
                if (i > 0) {
                    for (int k = i-1; k >= 0; k--) {
                        string str;
                        if (robots[i]->collisionCheck(robots[k],&str)) {
                            collision = true;
                            sstr << "Robot " << i << " (" << robots[i]->getName()
                                 << ") is in collision with robot " << k << " ("
                                 << robots[k]->getName() << ")" << endl;
                            sstr << str;
                            break;
                        }
                    }
                    if (collision) break;
                }
                if (collision) break;

                //third is testing if a robot autocollides
                string str;
                if (robots[i]->autocollision(0,&str)) {
                    collision = true;
                    sstr << "Robot " << i << " (" << robots[i]->getName()
                         << ") is in autocollision" << endl;
                    sstr << str;
                    break;
                }
                if (collision) break;

                //fourth is testing if an attached object (1-link-obstacle)
                //collides with the environment (other obstacles)
                list<attObj> *attachedObject = ((Robot*)robots[i])->getAttachedObject();
                for(list<attObj>::iterator it = attachedObject->begin();
                    it != attachedObject->end(); ++it) {
                    for (uint m = 0; m < obstacles.size(); m++) {
                        if (it->obs != obstacles.at(m)) {
                            string str;
                            if (it->obs->collisionCheck(obstacles.at(i),&str)) {
                                collision = true;
                                sstr << "Attached object " << it->obs->getName()
                                     << " is in collision with obstacle " << m << " ("
                                     << obstacles[m]->getName() << ")" << endl;
                                sstr << str;
                                break;
                            }
                            if (collision) break;
                        }
                        if (collision) break;
                    }
                    if (collision) break;
                }
                if (collision) break;
            }
        }

        // Here will be putted the configuration mapping
        sample->setMappedConf(_robConfigMap);

        if (collision) sample->setcolor(-1);
        else sample->setcolor(1);
        if (message != NULL) *message = sstr.str();
        return collision;
    }


    //! This method returns the distances between two samples smp1 and
    //! smp2 passed as arguments. If the SPACETYPE is CONFIGSPACE, first
    //! the samples are inspected looking for the RobConf associated.
    //! If the sample do not has one, the workspace is asked for the
    //! respective Mapping and then the distance is calculated.
    //! Be careful with samples non-free or without collision checking
    //! because they do not have mapping.
    //! If the SPACETYPE is SAMPLEDSPACE the distance is calculated with
    //! the coordinates directly.
    KthReal WorkSpace::distanceBetweenSamples(Sample& smp1, Sample& smp2,
                                              Kautham::SPACETYPE spc) {
        switch(spc){
        case SAMPLEDSPACE:
            return smp1.getDistance(&smp2, spc);

        case CONFIGSPACE:
            if( smp1.getMappedConf().size() == 0){
                this->moveRobotsTo(&smp1);
                smp1.setMappedConf(getRobConfigMapping());
            }
            if( smp2.getMappedConf().size() == 0){
                this->moveRobotsTo(&smp2);
                smp2.setMappedConf(getRobConfigMapping());
            }
            return smp1.getDistance(&smp2, _robWeight, spc);

        default:
            return (KthReal)-1.0;
        }
    }


    KthReal WorkSpace::distanceCheck(Conf* conf, unsigned int robot) {
        KthReal resp = (KthReal)1e10;
        KthReal temp = (KthReal)0.0;
        robots[robot]->Kinematics(conf);
        for(unsigned int i = 0; i < obstacles.size(); i++){
            temp = robots[robot]->distanceCheck(obstacles[i]);
            if(resp > temp)resp = temp;
        }
        if(!resp){      // Now I test the robots collision
            if(robots.size() > 1 )
                for( size_t i=0; i < robots.size(); i++){
                    if( i == robot ) continue;
                    if( robots[robot]->distanceCheck( robots[i] ) ){
                        resp = true;
                        break;
                    }
                }
        }
        return resp;
    }


    bool WorkSpace::collisionCheck(Conf* conf, unsigned int robot) {
        bool resp = false;
        robots[robot]->Kinematics(conf);
        for(unsigned int i = 0; i < obstacles.size(); i++){
            resp = robots[robot]->collisionCheck(obstacles[i]);
            if(resp)break;
        }
        if(!resp){      // Now I test the robots collision
            if(robots.size() > 1 )
                for( size_t i=0; i < robots.size(); i++){
                    if( i == robot ) continue;
                    if( robots[robot]->collisionCheck( robots[i] ) ){
                        resp = true;
                        break;
                    }
                }
        }
        return resp;
    }


    void WorkSpace::addRobot(Robot* robot) {
        robots.push_back(robot);
        _robConfigMap.clear();
        _robWeight.clear();
        for(unsigned int i = 0; i < robots.size(); i++){
            _robConfigMap.push_back(((Robot *)robots.at(i))->getCurrentPos());
            _robWeight.push_back(((Robot *)robots.at(i))->getRobWeight());
        }
    }


    void WorkSpace::addObstacle(Robot *obs) {
        obstacles.push_back(obs);
        _obsConfigMap.clear();
    }


    void WorkSpace::removeRobot(int index) {
        if ((index >= 0) && (index < robots.size())) {
            robots.erase(robots.begin()+index);
            _robConfigMap.clear();
            _robWeight.clear();
            for(unsigned int i = 0; i < robots.size(); i++){
                _robConfigMap.push_back(((Robot *)robots.at(i))->getCurrentPos());
                _robWeight.push_back(((Robot *)robots.at(i))->getRobWeight());
            }
        }
    }


    void WorkSpace::removeObstacle(int index) {
        obstacles.erase(obstacles.begin()+index);
        _obsConfigMap.clear();
    }


    bool WorkSpace::inheritSolution(vector<Sample*>& path) {
        vector< vector<RobConf*> > tmpRobPath;

        for(unsigned int i = 0; i < robots.size(); i++)
            tmpRobPath.push_back(*(new vector<RobConf*>)) ;

        vector<Sample*>::iterator it;
        for(it = path.begin(); it != path.end(); ++it){
            if((*it)->getMappedConf().size() == 0 )
                collisionCheck((*it));

            vector<RobConf>& tmpMapp = (*it)->getMappedConf();

            for(unsigned int i = 0; i < robots.size(); i++)
                tmpRobPath[i].push_back(&(tmpMapp.at(i)) );

        }

        for(unsigned int i = 0; i < robots.size(); i++)
            ((Robot *)robots.at(i))->setProposedSolution(tmpRobPath[i]);

        for(unsigned int i = 0; i < robots.size(); i++)
            tmpRobPath.at(i).clear();

        tmpRobPath.clear();
        return true;
    }


    void WorkSpace::eraseSolution() {
        vector<Robot*>::iterator it;
        for(it = robots.begin(); it != robots.end(); ++it){
            (*it)->cleanProposedSolution();
        }
    }


    void WorkSpace::setPathVisibility(bool vis) {
        for(size_t i = 0; i < robots.size(); i++ )
            ((Robot *)robots.at(i))->setPathVisibility( vis );
    }


    bool WorkSpace::attachObstacle2RobotLink(uint robot, uint link, uint obs) {
        if (robot < 0 || robot >= robots.size() ||
                link < 0 || link >= robots.at(robot)->getNumLinks() ||
                obs < 0 || obs >= obstacles.size()) return false;
        return (robots.at(robot)->attachObject(obstacles.at(obs),link));
    }


    bool WorkSpace::detachObstacle(uint obs) {
        if (obs < 0 || obs >= obstacles.size()) return false;
        if (!obstacles.at(obs)->isAttached()) return false;
        return (obstacles.at(obs)->getRobotAttachedTo()->detachObject(obstacles.at(obs)));
    }
}
