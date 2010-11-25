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
 
 

#ifndef KTHQUERY_H
#define KTHQUERY_H

#include <libutil/kauthamdefs.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

namespace libPlanner{
//  class Planner();
  class KthQuery{
  public:
      KthQuery();
      KthQuery(int init, int goal);

      //! Returns a vector with index of the samples whom comform the path solution.
      inline vector<int>&     getPath(){return _path;}
      inline void             setPath(vector<int>& path){_path.resize(path.size());
        copy(path.begin(),path.end(),_path.begin());}
      inline KthReal          getTotalTime(){return _totalTime;}
      inline void             setTotalTime(KthReal time){_totalTime = time;}
      inline KthReal          getSmoothTime(){return _smoothTime;}
      inline void             setSmoothTime(KthReal time){_smoothTime = time;}
      inline int              getGeneratedSamples(){return _generatedSamples;}
      inline int              getConnectedSamples(){return _connectedSamples;}
      inline int              getGeneratedEdges(){return _generatedEdges;}
      inline void             solved(bool solved){_solved = solved;}
      inline bool             solved(){return _solved;}
      inline int              getInit(){return _init;}
      inline int              getGoal(){return _goal;}
      inline bool             setInitGoal(int init, int goal){_init = init; _goal = goal;}
      inline void             setSampleStats(int generated, int connected, int edges){_generatedSamples = generated;
                                            _connectedSamples = connected; _generatedEdges = edges;}

      string printInit();
      string printGoal();
      string printSolved();
      string printGeneratedEdges();
      string printConnectedSamples();
      string printGeneratedSamples();
      string printPath();
      string printTotalTime();
      string printSmoothTime();

      bool sameInitGoal(const KthQuery &other) const;
      bool sameInitGoal(int init, int goal) const;
      bool operator==(const KthQuery &other) const;

  private:
      bool        _solved;
      KthReal     _totalTime;
      KthReal     _smoothTime;
      int         _init;
      int         _goal;
      vector<int> _path;
      int         _generatedSamples;
      int         _connectedSamples;
      int         _generatedEdges;
  };
}

#endif // KTHQUERY_H

