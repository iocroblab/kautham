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

}

