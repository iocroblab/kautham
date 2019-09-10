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

/* Author: Alexander Perez, Jan Rosell */

#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <kautham/planner/planner.h>
#include <pugixml.hpp>
#include <iostream>
#include <fstream>

using namespace pugi;

namespace Kautham{

  const KthReal RAD2GRAD=180.0/M_PI;
  Planner::Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws){
      _guiName = _idName = "";
      _spType = stype;
      _init.push_back(init);
      _goal.push_back(goal);
      _samples = samples;
      _wkSpace = ws;
      _path.clear();
      _parameters.clear();
      _solved = false;
      _hasCameraInformation = false;
      _sceneCspace = NULL;
      _scenePath = NULL;

  }

  bool Planner::solveAndInherit(){
    _solved = false;
    if(trySolve()){
      moveAlongPath(0);
      _wkSpace->inheritSolution(_simulationPath);
      return true;
    }
    return false;
  }
  
  void Planner::clearSimulationPath(){
    for(unsigned int i = 0; i < _simulationPath.size(); i++)
      delete _simulationPath[i];
    _simulationPath.clear();
    _wkSpace->eraseSolution();
  }

  void Planner::exportSimulationPath(){
    if( isSolved() ){
      ofstream  _theFile("Simulation_path_Rn.txt", ios::trunc | ios::out );

      if( getSimulationPath()->size() == 0 )
        moveAlongPath(0);

      for(size_t i = 0; i < getSimulationPath()->size(); i++){
        _wkSpace->moveRobotsTo(getSimulationPath()->at(i));
				std::vector<KthReal>& joints = _wkSpace->getRobot(0)->getCurrentPos()->getRn().getCoordinates();
        KthReal value=0;
        for(size_t j=0; j< joints.size(); j++){
          value = joints.at(j);
          _theFile << value << " ";
        }
        _theFile << std::endl;
      }

      _theFile.close();
    }else{
      std::cout << "The problem is not solved yet" << std::endl;
    }
  }

  
  void Planner::moveAlongPath(unsigned int step){
    if(_solved){
        //here simulationPath coincides with path
        //this function is overloaded by PRM planners that do an interpolation between paths nodes to obtain a more finner simulationPath
      if(_simulationPath.size() == 0 ){ // Then calculate the simulation path based on stepsize
            for(unsigned i=0; i<_path.size();i++)
            {
                Sample *s=new Sample(*_path.at(i));
                _simulationPath.push_back(s);
             }
      }
      if( _simulationPath.size() >= 2 ){
        step = step % _simulationPath.size();
        _wkSpace->moveRobotsTo(_simulationPath[step]);
      }else
        std::cout << "The problem is wrong solved. The solution path has less than two elements." << std::endl;
    }
  }

void Planner::loadSampleFromLine(Sample *Robsmp,std::stringstream  &RlineStream)
{
  //Load the configurations of the robots written in each line
  //loop for all the robots
  vector<RobConf> Robrc; //mapped configuration of the set of robots
  for (uint j = 0; j < _wkSpace->getNumRobots(); ++j) 
  {
    //std::cout << " robot "<< j << std::endl;
    RobConf *Robrcj = new RobConf; //Configuration for robot j
    vector<KthReal> se3coords; //SE3 part
    se3coords.resize(7);
    std::vector<KthReal> Rn; //Rn part
    //Mapped configuration: load the SE3 part, if ther is any
    if (_wkSpace->getRobot(j)->isSE3Enabled()) 
    {
      //std::cout << " se3 part ";
      //read the next 7 values
      for(int k=0; k<7; k++)
        RlineStream >> se3coords[k];
      //for(int k=0; k<7; k++)
      //  std::cout << " "<< se3coords[k];
      //std::cout << std::endl;
    }
    else{
      //If the robot does not have mobile SE3 dofs then the SE3 configuration of the init sample is maintained
      Robrcj->setSE3(initSamp()->getMappedConf()[0].getSE3());
    }
    SE3Conf se3;
    se3.setCoordinates(se3coords);
    Robrcj->setSE3(se3);
                      
    //Mapped configuration: load the Rn part, if ther is any
    if (_wkSpace->getRobot(j)->getNumJoints() > 0) 
    {
      //std::cout << " rn part: ";
      Rn.resize(_wkSpace->getRobot(j)->getNumJoints());
      //read the next n values
      for(uint k=0; k<_wkSpace->getRobot(j)->getNumJoints(); k++)
        RlineStream >> Rn[k];

      Robrcj->setRn(Rn);
      //for(int k=0; k<_wkSpace->getRobot(j)->getNumJoints(); k++)
      //  std::cout << " "<< Rn[k];
      //std::cout << std::endl;
    }
    else {
      //If the robot does not have mobile Rn dofs then the Rn is set to dim=0
      Robrcj->setRn(0);
    }

    //load the configuration of robot j to the configuration vector
    Robrc.push_back(*Robrcj);
  }

  //Set the mapped configuration of the sample
  //(its control values are kept to zero)               
  Robsmp->setMappedConf(Robrc);    
}                   

void Planner::moveAlongPathLoad(unsigned int step, std::ifstream &path)
{
    std::string action; //action being processed, either transit or transfer
    int attachedObj; //index of object being attached
    int robIndex; //index of robot to where the object is attached
    int linkIndex; //index of robot link to where the object is attached
    std::vector<Sample*> _path2;
    _path2.clear();
    std::string Rline;

    unsigned int line_number = 1; //to loop along the file
    unsigned int line_number2 = 1; //to loop along the configurations while processing an action

    //if(stop) std::cout<<"stop = true"<<std::endl; else std::cout<<"stop = false"<<std::endl;
    //if(stopAll) std::cout<<"stopAll = true"<<std::endl; else std::cout<<"stopAll = false"<<std::endl;

    while(  std::getline(path, Rline) && !stopAll )
    {
        stop = false;

        std::cout << " Rline requested is: " << requested_line_number << " "<< "line number "<<line_number<< Rline << std::endl;

        //a new action to be analyzed
        //line_numnber is the pointer to the line being analyzed (it is the counter of the main while loop)
        if (line_number != requested_line_number){
          line_number++;
          break;
        }
        else
        {
            std::vector <std::string> streamCheck; //the line is splited in a vector of strings
            streamCheck.clear();
            std::string check;

            //load the requested line to be analyzed into the vector of strings
            std::stringstream  RlineStreamCheck(Rline);

            while(RlineStreamCheck >> check)
            {
                streamCheck.push_back(check);
            }

            //analyze the first sting streamCheck[0]
            if(streamCheck[0] == "transit")
            {
                clearSimulationPath();
                //if(stopAll)
                //    break;

                //The action to be analyzed is a transit action
                action = streamCheck[0];

                //keep reading the next lines (until an "end" is found) 
                //these lines contain the configuration of the path to be transited and are stored in _path2 vector
                //line_number2 is the counter of the configurations of the path (it is the counter of the secondary while loop)
                while( std::getline(path, Rline) && !stop)
                {
                    std::cout << " Rline number is : " << line_number2 << std::endl;
                    //the while loop breakes when a "end" line is encountered
                    //then the next line to be analyzed (variable requested_line_number) is updated
                    if(Rline == "end")
                    { 
                        stop = true;
                        //stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " end - Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    //Read the line of coordinates
                    std::stringstream  RlineStream(Rline);
                    //Create the sample
                    Sample *Robsmp; //sample
                    Robsmp=new Sample(_wkSpace->getNumRobControls());
                    //fill the sample
                    loadSampleFromLine(Robsmp,RlineStream);
                    //add the sample to the path
                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }

            else if(streamCheck[0] == "transfer")
            {
                clearSimulationPath();
                //if(stopAll)
                //    break;

                //The action to be analyzed is a transfer action
                //it has three parameters: robot index, link index and object index.
                //they are stored in the streamCheck vector
                action = streamCheck[0];
                robIndex = atoi(streamCheck[1].c_str());
                linkIndex = atoi(streamCheck[2].c_str());
                attachedObj = atoi(streamCheck[3].c_str());

                //The object is attached to the link of the robot
                _wkSpace->attachObstacle2RobotLink(robIndex,linkIndex,attachedObj);

                //keep reading the next lines (until an "end" is found) 
                //these lines contain the configuration of the path to be transferred and are stored in _path2 vector
                //line_number2 is the counter of the configurations of the path (it is the counter of the secondary while loop)
                while( std::getline(path, Rline) && !stop)
                {
                    //std::cout << " Rline number is : " << line_number2 << std::endl;

                    if(Rline == "end")
                    { 
                        stop = true;
                        //stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " end - Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    //Read the line of coordinates
                    std::stringstream  RlineStream(Rline);
                    //Create the sample
                    Sample *Robsmp; //sample
                    Robsmp=new Sample(_wkSpace->getNumRobControls());
                    loadSampleFromLine(Robsmp,RlineStream);

                    //Load the configuiration to the path
                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }
            else if(streamCheck[0] == "endtask")
            {
                clearSimulationPath();
                std::cout << " ENDTASK - Plan successfully executed !"<<std::endl;
                requested_line_number = 1;
                stopAll = true;
            }
            else //TO BE REVISED
            {
                clearSimulationPath();
                std::cout << " Plan successfully executed !"<<std::endl;
                requested_line_number = 1;
                stopAll = true;
            }
        }
    }

    //loads the simulated path variable
    for(unsigned i=0; i<_path2.size();i++)
    {
        Sample *s=new Sample(*_path2.at(i));
        _simulationPath.push_back(s);
    }

    //moves the robot to the step of the simulated path specified by the step input parameter
    step = step % _simulationPath.size();
    _wkSpace->moveRobotsTo(_simulationPath[step]);

    std::cout<<step<<" SIMULATION FROM PATH "<<_simulationPath.size() <<" "<<action <<endl;

    //if(stop) std::cout<<"stop = true"<<std::endl; else std::cout<<"stop = false"<<std::endl;
    //if(stopAll) std::cout<<"stopAll = true"<<std::endl; else std::cout<<"stopAll = false"<<std::endl;

    //once the simulated path has reached its end, it should be cleared to proceed to the following
    if(step>=_simulationPath.size()-getSpeedFactor())
    {
        //if the action was a transfer, the object is detached when finished
        if(action=="transfer")
            _wkSpace->detachObstacle(attachedObj);

        stop=false;
        stopAll=false;
        //stopC=1;
    }
}

}

