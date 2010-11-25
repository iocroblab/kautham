/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingenierí­a "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
 
 

#include "kthquery.h"
#include <string>
#include <sstream>

namespace libPlanner{
  KthQuery::KthQuery(){
    KthQuery(-1,-1);
  }

  KthQuery::KthQuery(int init, int goal){
    _smoothTime = _totalTime = 0.;
    _init = init;
    _goal = goal;
    _path.clear();
    _generatedSamples = 0;
    _connectedSamples = 0;
    _generatedEdges = 0;
    _solved = false;
  }

  string KthQuery::printInit(){
    stringstream ss;
    ss << _init ;
    return ss.str();
  }

  string KthQuery::printGoal(){
    stringstream ss;
    ss << _goal ;
    return ss.str();
  }

  string KthQuery::printSolved(){
    if(_solved)
      return "True";
    else
      return "False";
  }

  string KthQuery::printGeneratedEdges(){
    stringstream ss;
    ss << _generatedEdges ;
    return ss.str();
  }

  string KthQuery::printConnectedSamples(){
    stringstream ss;
    ss << _connectedSamples ;
    return ss.str();
  }

  string KthQuery::printGeneratedSamples(){
    stringstream ss;
    ss << _generatedSamples ;
    return ss.str();
  }

  string KthQuery::printPath(){
    stringstream ss;
    for(int i=0; i<_path.size(); i++)
      ss << _path[i] << " ";
    string path = ss.str();
    return path.substr(0, path.length() - 1);
  }

  string KthQuery::printTotalTime(){
    stringstream ss;
    ss << _totalTime ;
    return ss.str();
  }

  string KthQuery::printSmoothTime(){
    stringstream ss;
    ss << _smoothTime ;
    return ss.str();
  }

  bool KthQuery::sameInitGoal(int init, int goal) const{
    if( _init == init && _goal == goal )
      return true;
    return false;
  }

  bool KthQuery::sameInitGoal(const KthQuery &other) const{
    if( _init == other._init && _goal==other._goal )
      return true;
    return false;
  }

  bool KthQuery::operator==(const KthQuery &other) const{
    if( _init == other._init && _goal==other._goal 
        && _path.size() == other._path.size()){
      for(int i = 0; i < _path.size(); i++)
        if(_path[i] != other._path[i])
          return false;
      return true;
    }
    return false;
  }
}


