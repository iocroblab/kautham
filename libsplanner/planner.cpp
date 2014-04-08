/*************************************************************************\
  Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITAT POLITECNICA DE CATALUNYA BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITAT POLITECNICA DE CATALUNYA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITAT
  POLITECNICA DE CATALUNYA  HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
  SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

\***************************************************************************/

/* Author: Alexander Perez, Jan Rosell */
 

#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "planner.h"
#include <libpugixml/pugixml.hpp>
#include <iostream>
#include <fstream>

using namespace pugi;

namespace Kautham{

  const KthReal RAD2GRAD=180.0/M_PI;
  Planner::Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws){
    _guiName = _idName = "";
		_spType = stype;		
    _init = init; 
    _goal = goal; 
    _samples = samples; 
    _wkSpace = ws; 
    _path.clear();
    _parameters.clear();
    _solved = false;
    _hasCameraInformation = false;
	_sceneCspace=NULL;
	}

  Planner::~Planner(){

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
          value = joints.at(j)* RAD2GRAD;
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
            int d =  _path[0]->getDim() ;
            for(int i=0; i<_path.size();i++)
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

