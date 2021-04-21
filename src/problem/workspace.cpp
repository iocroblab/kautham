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


#include <kautham/problem/workspace.h>
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
        _initObsSample = NULL;
    }


    unsigned int WorkSpace::_countWorldCollCheck = 0;

    /*!
    * Stores the initial poses of objects
    */
    void WorkSpace::storeInitialObjectPoses(){
        //
        _obstaclePoses.clear();
        _obstaclePoses.resize(getNumObstacles());
        uint i=0;
        for (std::pair<std::string, Robot*> element : obstacles )
        {
            RobConf *c = element.second->getHomePos();
            _obstaclePoses[i] = new RobConf;
            _obstaclePoses[i]->setSE3(c->getSE3());
            _obstaclePoses[i]->setRn(c->getRn());
        }
    }

    /*!
    * Stores current object SE3 poses as new initial poses
    */
    void WorkSpace::storeNewInitialObjectPoses(){
        //
        _obstaclePoses.clear();
        _obstaclePoses.resize(getNumObstacles());
        map<string, Robot*>::iterator mapit;
        uint i=0;
        for (std::pair<std::string, Robot*> element : obstacles )
        {
            RobConf *c = element.second->getHomePos();
            _obstaclePoses[i] = new RobConf;
            vector<KthReal> coords(7);
            coords[0] = element.second->getLink(0)->getElement()->getPosition()[0];
            coords[1] = element.second->getLink(0)->getElement()->getPosition()[1];
            coords[2] = element.second->getLink(0)->getElement()->getPosition()[2];
            coords[3] = element.second->getLink(0)->getElement()->getOrientation()[0];
            coords[4] = element.second->getLink(0)->getElement()->getOrientation()[1];
            coords[5] = element.second->getLink(0)->getElement()->getOrientation()[2];
            coords[6] = element.second->getLink(0)->getElement()->getOrientation()[3];
            // Here is needed to convert from axis-angle to
            // quaternion internal represtantation.
            SE3Conf::fromAxisToQuaternion(coords);
            _obstaclePoses[i]->setSE3(coords);
            _obstaclePoses[i]->setRn(c->getRn());
            i++;
        }
    }

    /*!
    * Retores the initial poses of objects
    */
    bool WorkSpace::restoreInitialObjectPoses(){
        if( _obstaclePoses.size()==getNumObstacles())
        {
            uint i=0;
            for (std::pair<std::string, Robot*> element : obstacles )
            {
                element.second->Kinematics( _obstaclePoses[i]);
                if(element.second->isAttached())
                {
                    element.second->getRobotAttachedTo()->detachObject(element.second);
                    element.second->setDetached();
                }
                i++;
            }
            return true;
        }
        else
        {
            cout<<"ERROR in restoring Initial Object Poses. Sizes mismatch"<<endl;
            return false;
        }
    }

    void WorkSpace::resetCollCheckCounter() {
        _countWorldCollCheck = 0;
    }


    unsigned int WorkSpace::getCollCheckCounter() {
        return _countWorldCollCheck;
    }


    void WorkSpace::increaseCollCheckCounter() {
        _countWorldCollCheck++;
    }




    vector<KthReal>* WorkSpace::distanceCheck(Sample* sample) {
        vector<KthReal> tmpVec;
        tmpVec.clear();

        if (sample->getMappedConf().size() == 0) {
            for(unsigned int j=0; j < getNumRobControls(); j++ )
                tmpVec.push_back(sample->getCoords()[j]);

            for(unsigned int i=0; i< robots.size(); i++)
                robots[i]->control2Pose(tmpVec);
        } else {
            for(unsigned int i=0; i< robots.size(); i++)
                robots[i]->Kinematics(sample->getMappedConf().at(i));
        }

        distVec.clear();
        for(unsigned int i=0; i< robots.size(); i++){
            for (std::pair<std::string, Robot*> element : obstacles )
            {
                distVec.push_back(robots[i]->distanceCheck(element.second));
            }
        }
        return &distVec;
    }


    double WorkSpace::cumDistanceCheck(Sample *sample) {
        if (sample->getMappedConf().size() == 0) {
            vector<KthReal> tmpVec;
            for (unsigned int i = 0; i < getNumRobControls(); ++i)
                tmpVec.push_back(sample->getCoords()[i]);

            for (unsigned int i = 0; i < robots.size(); ++i)
                robots[i]->control2Pose(tmpVec);
        } else {
            for (unsigned int i = 0; i < robots.size(); ++i)
                robots[i]->Kinematics(sample->getMappedConf().at(i));
        }

        double cumDist = 0, dist;
        for (unsigned int i = 0; i < robots.size(); ++i) {
            Robot *robot = robots[i];

            for (unsigned int k = 0; k < robot->getNumLinks(); ++k) {
                Link *link = robot->getLink(k);

                for (unsigned int l = k+1; l < robot->getNumLinks(); ++l) {
                    Link *otherLink = robot->getLink(l);

                    if (link->getParent() == otherLink || otherLink->getParent() == link) continue;

                    dist = link->getElement()->getDistanceTo(otherLink->getElement());
                    if (dist < 0) cumDist += -dist;
                }

                for (unsigned int j = i+1; j < robots.size(); ++j) {
                    Robot *otherRobot = robots[j];

                    for (unsigned int l = 0; l < otherRobot->getNumLinks(); ++l) {
                        Link *otherLink = otherRobot->getLink(l);

                        dist = link->getElement()->getDistanceTo(otherLink->getElement());
                        if (dist < 0) cumDist += -dist;
                    }
                }
                for (std::pair<std::string, Robot*> element : obstacles ){
                    for (unsigned int l = 0; l < element.second->getNumLinks(); ++l) {
                        Link *otherLink = element.second->getLink(l);

                        dist = link->getElement()->getDistanceTo(otherLink->getElement());
                        if (dist < 0) cumDist += -dist;
                    }
                }
            }
        }
        return cumDist;
    }



    /*!
    * Moves the robots to the configuration specified by the sample
    *  loads the flag withinbounds of the sample. If outofbounds, one of the
    *  robots ends at the border of one or more of its limits.
    */
    void WorkSpace::moveRobotsTo(Sample *sample) {
        bool withinbounds = true;
        vector<KthReal> tmpVec;
        tmpVec.clear();
        for (unsigned int i = 0; i < getNumRobControls(); ++i) {
            tmpVec.push_back(sample->getCoords().at(i));
        }

        for (unsigned int i = 0; i < robots.size(); ++i) {
            if (sample->getMappedConf().size() == 0) {
                withinbounds &= robots.at(i)->control2Pose(tmpVec);
            } else {
                robots.at(i)->Kinematics(sample->getMappedConf().at(i));
            }
        }
        _lastRobSampleMovedTo = sample;

        //set _sample::_config if it was not set
        if (sample->getMappedConf().size() == 0) sample->setMappedConf(_robConfigMap);

        sample->setwithinbounds(withinbounds);
    }


    /*!
   * Moves the obstacles to the configuration specified by the sample
   *  loads the flag withinbounds of the sample. If outofbounds, one of the
   *  obstacles ends at the border of one or more of its limits.
   */
    void WorkSpace::moveObstaclesTo(Sample *sample) {
        if (!sample) return;

        bool withinbounds=true;
        vector<KthReal> tmpVec;
        tmpVec.clear();
        for(unsigned int j=0; j < getNumObsControls(); j++ ){
            tmpVec.push_back(sample->getCoords()[j]);
            printf("%f ",tmpVec[j]);
        }
        printf("\n");
        printf("JAN Trying to move obstacles now\n");

        int i=0;
        for (std::pair<std::string, Robot*> element : obstacles ){
            if (!element.second->isAttached()) {
                if(sample->getMappedConf().size()==0){
                    withinbounds &= element.second->control2Pose(tmpVec);
                }
                else{
                    element.second->Kinematics(sample->getMappedConf().at(i));
                }
            }
            i++;
        }
        _lastObsSampleMovedTo = sample;

        //set _sample::_config if it was not set
        if(sample->getMappedConf().size()==0) {
            //printf("setting Mapped conf x=%f y=%d\n", _obsConfigMap[0]->getRn().getCoordinate(0),_obsConfigMap[0]->getRn().getCoordinate(0));
            sample->setMappedConf(_obsConfigMap);
        }
        sample->setwithinbounds(withinbounds);
    }


    void WorkSpace::setInitObsSample(Sample* initsample) {
        moveObstaclesTo(initsample);
        _initObsSample = initsample;
    }


    bool WorkSpace::collisionCheck(Sample* sample, string *message, std::pair< std::pair<int, string> , std::pair<int,int> > *colliding_elements) {
        stringstream sstr;
        increaseCollCheckCounter();

        std::pair<int, string> robot_Obst; //It will storaged the index of the robot and obstacles in collision
        std::pair<int, int> robot_links; //It will storaged the index of the robot's link/obstacle's element in collision

        vector<KthReal> tmpVec;
        bool collision = false;
        tmpVec.clear();
        for (unsigned j=0; j < getNumRobControls(); j++) {
            tmpVec.push_back(sample->getCoords()[j]);
        }

        if (sample->getMappedConf().size() == 0) {
            for (uint i=0; i< robots.size(); i++) {
                robots[i]->control2Pose(tmpVec);
                //first is testing if the robots collide with the environment (obstacles)

                uint m=0;
                for (std::pair<std::string, Robot*> element : obstacles ){
                    string str;
                    if (robots[i]->collisionCheck(element.second,&str,&robot_links)) {
                        collision = true;

                        robot_Obst.first = i;
                        robot_Obst.second = element.first;

                        sstr << "Robot " << i << " (" << robots[i]->getName()
                             << ") is in collision with obstacle " << m << " ("
                             << element.first << ")" << endl;
                        sstr << str;
                        break;
                    }
                    if (collision) break;
                    m++;
                }
                if (collision) break;

                //second is testing if a robot collides with another one present in the workspace
                //this validation is done with the robots validated previously
                if (i > 0) {
                    for (int k = i-1; k == 0; k--) {
                        string str;
                        if (robots[i]->collisionCheck(robots[k],&str,&robot_links)) {
                            collision = true;

                            robot_Obst.first = i;
                            robot_Obst.second = robots[k]->getName();

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
                if (robots[i]->autocollision(&str)) {
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
                    uint m=0;
                    for (std::pair<std::string, Robot*> element : obstacles ){
                        if (it->obs != element.second) {
                            string str;
                            if (it->obs->collisionCheck(element.second,&str, &robot_links)) {
                                collision = true;
                                robot_Obst.first = i;
                                robot_Obst.second = element.first;

                                sstr << "Attached object " << it->obs->getName()
                                     << " is in collision with obstacle " << m << " ("
                                     << element.first << ")"<<endl;
                                //<< " Att. ob. pos(" << it->obs->getHomePos()->getSE3().getPos()[0] <<", "
                                //<< ", "<< it->obs->getHomePos()->getSE3().getPos()[1] <<", "
                                //<< ", "<< it->obs->getHomePos()->getSE3().getPos()[2] <<")"<<endl;
                                sstr << str;
                                break;
                            }
                            if (collision) break;
                        }
                        if (collision) break;
                        m++;
                    }
                    if (collision) break;
                }
                if (collision) break;
            }
        } else {
            for (uint i=0; i< robots.size(); i++) {
                robots[i]->Kinematics(sample->getMappedConf().at(i));

                //first is testing if the robot collides with the environment (obstacles)
                map<string, Robot*>::iterator mapit;
                uint m;
                for(m=0,mapit = obstacles.begin(); mapit != obstacles.end(); mapit++,m++){
                    string str;
                    if (robots[i]->collisionCheck(mapit->second,&str,&robot_links)) {
                        collision = true;

                        robot_Obst.first = i;
                        robot_Obst.second = mapit->first;

                        sstr << "Robot " << i << " (" << robots[i]->getName()
                             << ") is in collision with obstacle " << m << " ("
                             << mapit->first << ")" << endl;
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
                        if (robots[i]->collisionCheck(robots[k],&str, &robot_links)) {
                            collision = true;

                            robot_Obst.first = i;
                            robot_Obst.second = robots[k]->getName();

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
                if (robots[i]->autocollision(&str)) {
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
                    map<string, Robot*>::iterator mapit;
                    uint m;
                    for(m=0,mapit = obstacles.begin(); mapit != obstacles.end(); mapit++,m++){
                        if (it->obs != mapit->second) {
                            string str;
                            if (it->obs->collisionCheck(mapit->second,&str, &robot_links)) {
                                collision = true;
                                robot_Obst.first = i;
                                robot_Obst.second = mapit->first;

                                sstr << "Attached object " << it->obs->getName()
                                     << " is in collision with obstacle " << m << " ("
                                     << mapit->first << ")" << endl;
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

        //here will be storaged the elements that are in collision
        if(colliding_elements != NULL){
            colliding_elements->first = robot_Obst;
            colliding_elements->second = robot_links;
        }

        // Here will be putted the configuration mapping
        sample->setMappedConf(_robConfigMap);

        if (collision) sample->setcolor(-1);
        else {
            sample->setcolor(1);
            sstr << "Collision free";
        }
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
        map<string, Robot*>::iterator mapit;
        for(mapit = obstacles.begin(); mapit != obstacles.end(); mapit++){
            temp = robots[robot]->distanceCheck(mapit->second);
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
        map<string, Robot*>::iterator mapit;
        for(mapit = obstacles.begin(); mapit != obstacles.end(); mapit++){
            resp = robots[robot]->collisionCheck(mapit->second);
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
        obstacles.insert(std::pair<string,Robot*>(obs->getName(), obs));
        _obsConfigMap.clear();
    //JAN-JAN

        for (std::pair<std::string, Robot*> element : obstacles ){
            _obsConfigMap.push_back(element.second->getCurrentPos());
        }

    //JAN-JAN
    }


    void WorkSpace::removeRobot(unsigned index) {
        if (index < robots.size()) {
            robots.erase(robots.begin()+index);
            _robConfigMap.clear();
            _robWeight.clear();
            for(unsigned int i = 0; i < robots.size(); i++){
                _robConfigMap.push_back(((Robot *)robots.at(i))->getCurrentPos());
                _robWeight.push_back(((Robot *)robots.at(i))->getRobWeight());
            }
        }
    }



    void WorkSpace::removeObstacle(string obsname) {
        map<string,Robot*>::iterator mapit = obstacles.find(obsname);
        if(mapit!=obstacles.end()){
            obstacles.erase(mapit);
            _obsConfigMap.clear();
        }
        //JAN-JAN

        for (std::pair<std::string, Robot*> element : obstacles ){
            _obsConfigMap.push_back(element.second->getCurrentPos());
        }

        //JAN-JAN
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


    bool WorkSpace::attachObstacle2RobotLink(uint robot, uint link, string obsname) {
       map<string,Robot*>::iterator mapit = obstacles.find(obsname);
       if(mapit!=obstacles.end()){
            if (robot >= robots.size() ||
                    link >= robots.at(robot)->getNumLinks())
                return false;
            return (robots.at(robot)->attachObject(mapit->second,link));
       }
       return false;
    }


    bool WorkSpace::detachObstacle(string obsname) {
        map<string,Robot*>::iterator mapit = obstacles.find(obsname);
        if(mapit!=obstacles.end()){
            if (!mapit->second->isAttached()) return false;
            return (mapit->second->getRobotAttachedTo()->detachObject(mapit->second));
        }
        return false;
    }


    Robot *WorkSpace::getRobot(string name) {
        for (size_t i = 0; i < robots.size(); ++i) {
            if (robots.at(i)->getName() == name) return robots.at(i);
        }
        return NULL;
    }
}
