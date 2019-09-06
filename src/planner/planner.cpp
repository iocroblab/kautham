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


void Planner::moveAlongPathLoad(unsigned int step, std::string address)
{
    bool stopAll = false; //to stop main while loop
    bool stop = false; //to stop nested while loops
    std::string action; //action being processed, either transit or transfer
    int attachedObj; //index of object being attached
    int robIndex; //index of robot to where the object is attached
    int linkIndex; //index of robot link to where the object is attached
    unsigned int requested_line_number = 1;
    std::ifstream path;
    std::vector<Sample*> _path2;
    _path2.clear();
    std::string Rline;
    float value;

    unsigned int line_number = 1; //to loop along the file
    unsigned int line_number2 = 1; //to loop along the configurations while processing an action


    path.open (address);
    if (path.is_open())
    {
      std::cout<< " File successfully opened ! "<<std::endl;
    }
    else
    {
      std::cout << "Error opening file : " <<address<<std::endl;
      exit(1);
    }

    while(  std::getline(path, Rline) && !stopAll )
    {
        clearSimulationPath();

        std::cout << " Rline requested is : " << requested_line_number << " "<< Rline<< std::endl;

        if (line_number == requested_line_number)
        {
            std::vector <std::string> streamCheck;
            streamCheck.clear();
            std::string check;
            std::stringstream  RlineStreamCheck(Rline);

            while(RlineStreamCheck >> check)
            {
                streamCheck.push_back(check);
            }

            if(streamCheck[0] == "transit")
            {
                if(stopAll)
                    break;

                action = streamCheck[0];

                while( std::getline(path, Rline) && !stop)
                {
                    //std::cout << " Rline number is : " << line_number2 << std::endl;

                    if(Rline == "end")
                    { 
                        stop = true;
                        stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    Sample *Robsmp;
                    vector<RobConf> Robrc;
                    std::vector<float>   Rn;

                    Robsmp=new Sample(_wkSpace->getNumRobControls());

                    RobConf *Robrcj = new RobConf;

                    std::stringstream  RlineStream(Rline);


                    while(RlineStream >> value)
                    {
                        Rn.push_back(value);
                    }
                    Robrcj->setSE3(initSamp()->getMappedConf()[0].getSE3());
                    Robrcj->setRn(Rn);
                    Robrc.push_back(*Robrcj);
                    Robsmp->setMappedConf(Robrc);

                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }

            else if(streamCheck[0] == "transfer")
            {
                if(stopAll)
                    break;

                action = streamCheck[0];
                robIndex = atoi(streamCheck[1].c_str());
                linkIndex = atoi(streamCheck[2].c_str());
                attachedObj = atoi(streamCheck[3].c_str());

                _wkSpace->attachObstacle2RobotLink(robIndex,linkIndex,attachedObj);

                while( std::getline(path, Rline) && !stop)
                {
                    //std::cout << " Rline number is : " << line_number2 << std::endl;

                    if(Rline == "end")
                    { 
                        stop = true;
                        stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    Sample *Robsmp;
                    vector<RobConf> Robrc;
                    std::vector<float>   Rn;

                    Robsmp=new Sample(_wkSpace->getNumRobControls());

                    RobConf *Robrcj = new RobConf;

                    std::stringstream  RlineStream(Rline);


                    while(RlineStream >> value)
                    {
                        Rn.push_back(value);
                    }
                    Robrcj->setSE3(initSamp()->getMappedConf()[0].getSE3());
                    Robrcj->setRn(Rn);
                    Robrc.push_back(*Robrcj);
                    Robsmp->setMappedConf(Robrc);

                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }
            else
            {
                clearSimulationPath();
                std::cout << " Plan successfully executed !"<<std::endl;
                requested_line_number = 1;
            }
        }
        line_number++;
    }


    for(unsigned i=0; i<_path2.size();i++)
    {
        Sample *s=new Sample(*_path2.at(i));
        _simulationPath.push_back(s);
    }
    step = step % _simulationPath.size();
    _wkSpace->moveRobotsTo(_simulationPath[step]);

    std::cout<<step<<" SIMULATION FROM PATH "<<_simulationPath.size() <<" "<<action <<endl;

    if(step==_simulationPath.size()-1)
    {
        if(action=="transfer")
            _wkSpace->detachObstacle(attachedObj);

        stop=false;
        stopAll=false;
        stopC=1;
    }
}

}

