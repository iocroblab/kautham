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
 
 

#if !defined(_PLANNER_H)
#define _PLANNER_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <libkthutil/kauthamdefs.h>
#include <cmath>
#include <string>
#include <mt/transform.h>

using namespace std;

namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */

  class Planner: public KauthamObject{
    public:
    Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);

    ~Planner();
    virtual bool                  trySolve()=0;
    virtual bool                  setParameters() = 0;
    virtual void                  moveAlongPath(unsigned int step);
    virtual bool                  solveAndInherit();


    inline string                 getIDName(){return _idName;}
    inline void                   setSampleSet(SampleSet* smpSet){_samples = smpSet;}
    inline Sample*                initSamp(){return _init;}
    inline Sample*                goalSamp(){return _goal;}
    inline void                   setInitSamp(Sample* init){_init = init;}
    inline void                   setGoalSamp(Sample* goal){_goal = goal;}
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


    virtual inline SoSeparator*    getIvCspaceScene(){return _sceneCspace;}//_sceneCspace is initiallized to NULL

    inline long int               getMaxNumSamples(){return _maxNumSamples;}
    inline string getFamily(){return _family;}

	protected:
    Planner();
    std::string                   _idName;
    SampleSet*                    _samples;
    Sample*                       _init;
    Sample*                       _goal;
    vector<Sample*>               _path;
    vector<Sample*>               _simulationPath;
    vector<mt::Transform>         _cameraPath;
    WorkSpace*                    _wkSpace;
    bool                          _solved;
    SPACETYPE                     _spType;
    int                           _speedFactor;
    bool                          _hasCameraInformation;
    unsigned int                  _maxNumSamples;
    string                        _family;
    SoSeparator*                  _sceneCspace;




	};

  /** @}   end of Doxygen module "libPlanner */
}

#endif  //_PLANNER_H

