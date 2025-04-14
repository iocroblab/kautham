/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

#include <kautham/sampling/robconf.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/sampling/sample.h>
#include <kautham/problem/robot.h>

#include <vector>


using namespace std;

namespace Kautham{

/** \addtogroup Problem
 *  @{
 */

  class WorkSpace {
  public:
      WorkSpace();
      virtual ~WorkSpace() {}
      double               distanceCheck( Conf* conf, unsigned int robot ) ;
      bool                  collisionCheck( Conf* conf, unsigned int robot ) ;
      double               distanceBetweenSamples(Sample& smp1, Sample& smp2,Kautham::SPACETYPE spc);
      vector<double>      *distanceCheck(Sample* sample) ;
      double                cumDistanceCheck(Sample *sample);
      bool                  collisionCheck(Sample* sample , string *message = NULL, std::pair<std::pair<int, string>, std::pair<int, int> > *colliding_elements = NULL);
      void                  moveRobotsTo(Sample* sample);
      void                  moveObstaclesTo(Sample* sample);
      void                  addRobot(Robot* robot);
      void                  addObstacle(Robot* obs);
      Robot*                getRobot(const std::string& _name);
      inline Robot*         getRobot(unsigned int i){if( i < robots.size() ) return robots[i]; return NULL;}
      inline Robot*         getObstacle(string obstaclename){map<string, Robot*>::iterator it = obstacles.find(obstaclename);
                                                             if(it!=obstacles.end()) return it->second; else return NULL;}
      inline unsigned       getNumRobots(){return robots.size();}
      inline unsigned       getNumObstacles(){return obstacles.size();}

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
      bool                  attachObstacle2RobotLink(uint robot, uint link, string obsname);

      //! This method detaches an object previously attached to a Robot link.
      bool                  detachObstacle(string obsname);

      void removeRobot(unsigned index);
      void removeObstacle(string obsname);


      static void           resetCollCheckCounter();
      static unsigned int   getCollCheckCounter();
      static void           increaseCollCheckCounter();
      inline Sample*        getLastRobSampleMovedTo(){return _lastRobSampleMovedTo;}
      inline Sample*        getLastObsSampleMovedTo(){return _lastObsSampleMovedTo;}
      void                  setInitObsSample(Sample* initsample);
      inline Sample*        getInitObsSample(){return _initObsSample;}
      inline unsigned       getNumRobControls(){return numRobControls;} //!< Returns the number of robot controls
      inline unsigned       getNumObsControls(){return numObsControls;} //!< Returns the number of obstacle controls
      inline void           setNumRobControls(unsigned numcontrols){numRobControls = numcontrols;} //!< Sets the number of robot controls
      inline void           setNumObsControls(unsigned numcontrols){numObsControls = numcontrols;} //!< Sets the number of obstacle controls
      inline string         getRobControlsName() const {return robControlsName;} //!< Returns the string containing the robot control names, separated by the vertical line character
      inline string         getObsControlsName() const {return obsControlsName;} //!< Returns the string containing the obstacle control names, separated by the vertical line character
      inline void           setRobControlsName(string controlsname){robControlsName=controlsname;} //!< Sets the string containing the robot control names, separated by the veritcal line character
      inline void           setObsControlsName(string controlsname){obsControlsName=controlsname;} //!< Sets the string containing the obstacle control names, separated by the veritcal line character
      inline map<string, Robot*> getObstaclesMap(){ return obstacles;}

       void                 storeInitialObjectPoses();
       void                 storeNewInitialObjectPoses();
       bool                 restoreInitialObjectPoses();

    //!< Returns the robot control names.
    inline std::vector<std::string> getRobControlsNames() const
    {
        std::vector<std::string> controls;
        std::stringstream ss(robControlsName);
        std::string name;

        // Split the string by the vertical line '|'
        while (std::getline(ss, name, '|')) {
            controls.push_back(name);
        }

        return controls;
    }

    //!< Returns the obstacle control names.
    inline std::vector<std::string> getObsControlsNames() const
    {
        std::vector<std::string> controls;
        std::stringstream ss(obsControlsName);
        std::string name;

        // Split the string by the vertical line '|'
        while (std::getline(ss, name, '|')) {
            controls.push_back(name);
        }

        return controls;
    }

  protected:
      virtual void          updateScene() = 0;
      vector<Robot*>        robots;
      map<string, Robot*> obstacles;
      vector<double>       distVec;
      //! This attribute groups the configurations of the robots
      vector<RobConf*>      _robConfigMap;
      vector<RobConf*>      _obsConfigMap;
      vector<RobWeight*>    _robWeight;
      //string neighborhoodMapFile;
      vector<RobConf*>      _obstaclePoses;

  private:
      bool                  armed;
      static unsigned       _countWorldCollCheck;
      unsigned              numRobControls;  //!< This is the number of controls used to command the robots
      unsigned              numObsControls;  //!< This is the number of controls used to command the obstacles
      string                robControlsName; //!< Names of the robot controls, as a string, separated with the vertical bar character.
      string                obsControlsName; //!< Names of the obstacle controls, as a string, separated with the vertical bar character.
      Sample*               _lastRobSampleMovedTo;
      Sample*               _lastObsSampleMovedTo;
      Sample*               _initObsSample;
  };

  /** @}   end of Doxygen module "Problem" */
}

#endif  //_WORKSPACE_H
