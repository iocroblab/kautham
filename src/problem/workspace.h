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

 
#if !defined(_WORKSPACE_H)
#define _WORKSPACE_H

#include <vector>
#include <sampling/robconf.h>
#include <util/kthutil/kauthamdefs.h>
#include <sampling/sample.h>
#include "robot.h"

using namespace std;

namespace Kautham{

/** \addtogroup libProblem
 *  @{
 */

  class WorkSpace {
  public:
      WorkSpace();
      KthReal               distanceCheck( Conf* conf, unsigned int robot ) ;
      bool                  collisionCheck( Conf* conf, unsigned int robot ) ;
      KthReal               distanceBetweenSamples(Sample& smp1, Sample& smp2,Kautham::SPACETYPE spc);
      vector<KthReal>*      distanceCheck(Sample* sample) ;
      bool                  collisionCheck(Sample* sample ) ;
      void                  moveRobotsTo(Sample* sample);
      void                  moveObstaclesTo(Sample* sample);
      void                  addRobot(Robot* robot);
      void                  addObstacle(Robot* obs);
      inline Robot*         getRobot(unsigned int i){if( i < robots.size() ) return robots[i]; return NULL;} 
      inline Robot*         getObstacle(unsigned int i){if(i < obstacles.size()) return obstacles[i]; return NULL;}
      inline int            getNumRobots(){return robots.size();}
      inline int            getNumObstacles(){return obstacles.size();}
      void                  addDistanceMapFile(string distanceFile);
      inline string         getDistanceMapFile(){return distanceMapFile;}
      void                  addDimensionsFile(string dfile);
      inline string         getDimensionsFile(){return dimensionsFile;}
      void                  addDirCase(string dirc);
      inline string         getDirCase(){return dirCase;}
      //void addNeighborhoodMapFile(string neighFile);
      //inline string getNeighborhoodMapFile(){return neighborhoodMapFile;};

      //      //! This method returns true if the all robots in the scene only accepts SE3 data;
      //      //! This method is deprecated. Maybe it never has been used.
      //      bool                  isSE3();

      //! This vector contains a pointers to the RobConf of each robot in the
      //! WorkSpace
      inline vector<RobConf*>&     getRobConfigMapping(){return _robConfigMap;}
      inline vector<RobConf*>&     getRobConfigMapping(Sample* sample){moveRobotsTo(sample); return _robConfigMap;}
      inline vector<RobConf*>&     getObsConfigMapping(){return _obsConfigMap;}
      inline vector<RobConf*>&     getObsConfigMapping(Sample* sample){moveObstaclesTo(sample); return _obsConfigMap;}
      bool                         inheritSolution(vector<Sample*>& path);
      void                         eraseSolution();
      void                         setPathVisibility(bool vis);

      //! This method attaches an object object to a robot link. The obstacle is designated by its index.
      bool                  attachObstacle2RobotLink(string robot, string link, unsigned int obs );

      //! This method detaches an object previously attached to a Robot link.
      bool                  detachObstacleFromRobotLink(string robot, string link );

      static void           resetCollCheckCounter();
      static unsigned int   getCollCheckCounter();
      static void           increaseCollCheckCounter();
      inline Sample*        getLastRobSampleMovedTo(){return _lastRobSampleMovedTo;}
      inline Sample*        getLastObsSampleMovedTo(){return _lastObsSampleMovedTo;}
      void                  setInitObsSample(Sample* initsample);
      inline Sample*        getInitObsSample(){return _initObsSample;}
      inline int            getNumRobControls(){return numRobControls;} //!< Returns the number of robot controls
      inline int            getNumObsControls(){return numObsControls;} //!< Returns the number of obstacle controls
      inline int            setNumRobControls(int numcontrols){numRobControls = numcontrols;} //!< Sets the number of robot controls
      inline int            setNumObsControls(int numcontrols){numObsControls = numcontrols;} //!< Sets the number of obstacle controls
      inline string         getRobControlsName() const {return robControlsName;} //!< Returns the string containing the robot control names, separated by the vertical line character
      inline string         getObsControlsName() const {return obsControlsName;} //!< Returns the string containing the obstacle control names, separated by the vertical line character
      inline void           setRobControlsName(string controlsname){robControlsName=controlsname;} //!< Sets the string containing the robot control names, separated by the veritcal line character
      inline void           setObsControlsName(string controlsname){obsControlsName=controlsname;} //!< Sets the string containing the obstacle control names, separated by the veritcal line character

  protected:
      virtual void          updateScene() = 0;
      vector<Robot*>        robots;
      vector<Robot*>        obstacles;
      vector<KthReal>       distVec;
      //! This attribute groups the configurations of the robots
      vector<RobConf*>      _robConfigMap;
      vector<RobConf*>      _obsConfigMap;
      vector<RobWeight*>    _robWeight;
      string                distanceMapFile;
      string                dimensionsFile;
      string                dirCase;
      //string neighborhoodMapFile;

  private:
      bool                  armed;
      static unsigned int   _countWorldCollCheck;
      int                   numRobControls;  //!< This is the number of controls used to command the robots
      int                   numObsControls;  //!< This is the number of controls used to command the obstacles
      string                robControlsName; //!< Names of the robot controls, as a string, separated with the vertical bar character.
      string                obsControlsName; //!< Names of the obstacle controls, as a string, separated with the vertical bar character. 
      Sample*               _lastRobSampleMovedTo;
      Sample*               _lastObsSampleMovedTo;
      Sample*               _initObsSample;
  };

  /** @}   end of Doxygen module "libProblem" */
}

#endif  //_WORKSPACE_H

