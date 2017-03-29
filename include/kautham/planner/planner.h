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
 
 

#if !defined(_PLANNER_H)
#define _PLANNER_H

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <cmath>
#include <string>
#include <mt/transform.h>

using namespace std;

namespace Kautham {
/** \addtogroup GeometricPlanners
 *  @{
 */

   class Planner: public KauthamObject{
   public:
       Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
       virtual ~Planner() {}
       virtual bool                  trySolve()=0;
       virtual bool                  setParameters() = 0;
       virtual void                  moveAlongPath(unsigned int step);
       virtual bool                  solveAndInherit();
       inline virtual bool           filtersample(Sample* smp){(void)smp; return false;}
       inline string                 getIDName(){return _idName;}
       inline void                   setSampleSet(SampleSet* smpSet){_samples = smpSet;}
       inline Sample*                initSamp(std::size_t index = 0) {return _init.at(index);}
       inline Sample*                goalSamp(std::size_t index = 0) {return _goal.at(index);}
       inline std::size_t            getNumInitSamp() {return _init.size();}
       inline std::size_t            getNumGoalSamp() {return _goal.size();}
       inline void                   setInitSamp(Sample* init) {_init.clear(); _init.push_back(init);}
       inline void                   setGoalSamp(Sample* goal) {_goal.clear(); _goal.push_back(goal);}
       inline void                   addInitSamp(Sample* init) {_init.push_back(init);}
       inline void                   addGoalSamp(Sample* goal) {_goal.push_back(goal);}
       inline void                   clearInitSamp() {_init.clear();}
       inline void                   clearGoalSamp() {_goal.clear();}
       inline WorkSpace*             wkSpace(){return _wkSpace;}
       inline void                   setWorkSpace(WorkSpace *ws){_wkSpace = ws;}
       inline vector<Sample*>*       getPath(){if(_solved)return &_path;else return NULL;}
       inline int                    getSpeedFactor(){return _speedFactor;}
       inline void                   setSpeedFactor(int sf){_speedFactor = sf;}
       void                          clearSimulationPath();
       inline bool                   isSolved(){return _solved;}
       inline vector<Sample*>*       getSimulationPath(){if(_solved)return &_simulationPath;else return NULL;}
       inline void                   setCameraMovements(bool c){_hasCameraInformation = c;}
       inline bool                   hasCameraMovements(){return _hasCameraInformation;}
       inline void                   addCameraMovement(mt::Transform &t){_cameraPath.push_back(t);}
       inline void                   clearCameraMovement(){ _cameraPath.clear();}
       inline mt::Transform*         getCameraMovement(unsigned int step){
           if(_solved && _cameraPath.size() > 0 )
               return &_cameraPath.at(step % _simulationPath.size());
           else
               return NULL;
       }
       void                           exportSimulationPath();
       virtual inline SoSeparator*    getIvPathScene(){return _scenePath;}//_sceneCspace is initiallized to NULL
       virtual inline SoSeparator*    getIvCspaceScene(){return _sceneCspace;}//_sceneCspace is initiallized to NULL
       inline long int                getMaxNumSamples(){return _maxNumSamples;}
       inline PLANNERFAMILY           getFamily(){return _family;}
       inline int                     findIndex(Sample *s){return _samples->indexOf(s);}

   protected:
       Planner();
       std::string                   _idName;
       SampleSet*                    _samples;
       vector<Sample*>               _init;
       vector<Sample*>               _goal;
       vector<Sample*>               _path;
       vector<vector<vector<KthReal> > >  _StateBodies;
       vector<Sample*>               _simulationPath;
       vector<mt::Transform>         _cameraPath;
       WorkSpace*                    _wkSpace;
       bool                          _solved;
       SPACETYPE                     _spType;
       int                           _speedFactor;
       bool                          _hasCameraInformation;
       unsigned int                  _maxNumSamples;
       PLANNERFAMILY                 _family;
       SoSeparator*                  _sceneCspace;
       SoSeparator*                  _scenePath;
   };
   /** @}   end of Doxygen module */
}
#endif  //_PLANNER_H
