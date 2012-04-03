/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2010 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
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

#ifndef MOUSEJUMPWIDGET_H
#define MOUSEJUMPWIDGET_H

#include <QWidget>
#include <QtGui>
#include <QProcess>
#include <libdevice/device.h>
#include <libproblem/problem.h>
#include <libutil/data_ioc_cell.hpp>
#include <libutil/CHostLookup.h>
#include <libguiding/pathtoguide.h>
#include "gui.h"
#include "axis.h"
#include <string.h>
#include <libproblem/inversekinematic.h>
#include <iostream>
#include <fstream>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/fields/SoMFVec3f.h>

using namespace libDevice;
using namespace libProblem;
using namespace libGuiding;
using namespace kautham;
namespace interboost = boost::interprocess;

namespace libGUI{
  typedef KthReal pathPoint[6]; // used to store the path in x, y, z, Yaw, Pitch, Roll format
  struct GuiNode{
    pathPoint point;
    pathPoint unitVec;
    GuiNode(){for(int i=0; i<6; i++){point[i]=0.;unitVec[i]=0.;}}
  };
 
  class TeleoperationWidget : public QWidget{
    Q_OBJECT
    private slots:
      void                  acts();
      void                  changeCamera();
      void                  getCamera();
      void                  startTeleoperation();
      void                  stopTeleoperation();
      void                  changeTranScale(int value);
      void                  changeRotScale(int value);
      void                  changeRobot();
      void                  connectCell();
      void                  confConnection();
      void                  rosClientEnd(int exitCode, QProcess::ExitStatus exitStatus);
      void                  enableAxial();
    public:
      TeleoperationWidget(Problem* prob, Device* hap, GUI* gui, kautham::data_ioc_cell* dataCell);
      ~TeleoperationWidget();
      	
      inline int getFreqPubli(){return _freqPubli;}
      inline void setFreqPubli(int freq){_freqPubli = freq;}
      	
      inline string getNodeName(){return _nodeName;}
      inline void setNodeName(string nom){_nodeName = nom;}
      
      inline string getIPMaster(){return _ipMaster;}
      inline void setIPMaster(string ipM){_ipMaster = ipM;}

      inline string getIPLocal(){return _ipNode;}
      inline void setIPLocal(string ipL){_ipNode = ipL;}

    private:
      void                  synchronization(bool applyForces = true);
      void                  teleoperation();
      Problem*              _problem;
      Device*               _haptic;
      Link*                 _tcpLink;
      GUI*                  _gui;
      bool                  _synchronized;

      mt::Transform         _w2tcp0;
      mt::Transform         _tcp2tcpci;
      mt::Transform         _h2hip0;
      mt::Transform         _w2h;

      mt::Transform         _h2newHip; //!> Used in the resynchronization

      mt::Transform         _whip2tcp0;
      mt::Rotation          _lastHIPRot;

      bool                  _inAction;
      bool                  _lastAction;
      bool                  _callNewH;
      SoSeparator*          _hapticBox;
      SoSeparator*          _EFrame, *_KFrame, *_KAFrame, *_KBFrame;
      //SoLineSet*            _forceVector;
      SoCoordinate3*        _forceVectorComp;
      SoSFVec3f*            _scaVec ;
      SoSFVec3f*            _posVec, *_posKFrame, *_posKAFrame, *_posKBFrame;
      SoSFRotation*         _rotVec, *_rotKFrame, *_rotKAFrame, *_rotKBFrame;
      float                 _rotScale;
      float                 _tranScale;
      KthReal               _hapticBoxSize[6];
      vector<GuiNode*>      _guidingPath[2];
      KthReal               _threshold; //!> Guiding threshold to change direction near a node.
      KthReal               EPSILON_1, EPSILON_2, EPSILON_3;
      KthReal               _maxForces[6];

      //! Added to increase the guiding capabilities.
      PathToGuide*          _pathsObj[2];
      RobLayout             _currentLayout;
      ofstream              _theFile;

      //! The _dataCell, the _publisher and the suscriber conform the core of interchange strategy 
      //! to use with ROS message system.
      data_ioc_cell*		    _dataCell;
      QProcess*             _rosClient;
      string                _nodeName;
      string                _ipMaster; 
      string                _ipNode;
      int                   _freqPubli;
      clock_t               _entertime, _endtime;


      void                  setupUI();
      void                  updateGuidingPath();
      unsigned int          findNearNode(vector<GuiNode*>& path, pathPoint& tcploc, KthReal& dist );
      void                  findGuideVector(vector<GuiNode*>& path, pathPoint& tcploc, pathPoint& vecTo);
      mt::Transform         getNewH2Hip();
      mt::Transform         getNewH2HipVec(vector<GuiNode*>& path, mt::Transform& tcp);
      mt::Transform         getNewH2HipBB(vector<GuiNode*>& path, mt::Transform& tcp);
      void                  calculateForce(pathPoint delta, KthReal dist, KthReal forces[]);
      void                  setJumpForce(mt::Transform& tcp);
      int                   setGuideForce(mt::Transform& tcp, mt::Transform& w2h, bool& projDirecPos);

      //XXXXXXXXXXXXXXXXXXXXXXXX
      QWidget *widget;
      QVBoxLayout *verticalLayout_5;
      QGroupBox *groupBox;
      QGridLayout *gridLayout_7;
      QVBoxLayout *verticalLayout_11;
      QGridLayout *gridLayout_4;
      QVBoxLayout *verticalLayout;
      QPushButton *_cameraTop;
      QPushButton *_cameraBottom;
      QVBoxLayout *verticalLayout_4;
      QPushButton *_cameraLeft;
      QPushButton *_cameraRight;
      QVBoxLayout *verticalLayout_10;
      QPushButton *_cameraFront;
      QPushButton *_cameraRear;
      QGroupBox *groupBox_3;
      QGridLayout *gridLayout_3;
      QVBoxLayout *verticalLayout_2;
      QHBoxLayout *horizontalLayout_5;
      QLabel *label;
      QLineEdit *settingX;
      QLabel *label_7;
      QLineEdit *settingRX;
      QHBoxLayout *horizontalLayout_6;
      QLabel *label_2;
      QLineEdit *settingY;
      QLabel *label_8;
      QLineEdit *settingRY;
      QHBoxLayout *horizontalLayout_7;
      QLabel *label_3;
      QLineEdit *settingZ;
      QLabel *label_9;
      QLineEdit *settingRZ;
      QPushButton *_setCamera;
      QGroupBox *groupBox_4;
      QGridLayout *gridLayout;
      QVBoxLayout *verticalLayout_3;
      QHBoxLayout *horizontalLayout_8;
      QLabel *label_4;
      QLineEdit *currentX;
      QLabel *label_10;
      QLineEdit *currentRX;
      QHBoxLayout *horizontalLayout_9;
      QLabel *label_5;
      QLineEdit *currentY;
      QLabel *label_12;
      QLineEdit *currentRY;
      QHBoxLayout *horizontalLayout_10;
      QLabel *label_6;
      QLineEdit *currentZ;
      QLabel *label_11;
      QLineEdit *currentRZ;
      QPushButton *_getCamera;
      QGroupBox *groupBox_2;
      QGridLayout *gridLayout_9;
      QGridLayout *gridLayout_8;
      QGroupBox *groupBox_6;
      QGridLayout *gridLayout_5;
      QHBoxLayout *horizontalLayout_12;
      QPushButton *_cmdConfConnection;
      QPushButton *_cmdConnectCell;
      QHBoxLayout *horizontalLayout;
      QGroupBox *_groupRobots;
      QGridLayout *gridLayout_2;
      QHBoxLayout *horizontalLayout_3;
      QRadioButton *_radBttRobot0;
      QRadioButton *_radBttRobot1;
      QGroupBox *groupBox_5;
      QGridLayout *gridLayout_6;
      QHBoxLayout *horizontalLayout_2;
      QRadioButton *_radCamera;
      QRadioButton *_radWorld;
      QHBoxLayout *horizontalLayout_11;
      QVBoxLayout *verticalLayout_6;
      QLabel *_lblTransScale;
      QSlider *_sliderTransScale;
      QVBoxLayout *verticalLayout_7;
      QLabel *_lblRotScale;
      QSlider *_sliderRotScale;
      QHBoxLayout *horizontalLayout_13;
      QCheckBox *_chkMagnetic;
      QCheckBox *_chkAxial;
      QHBoxLayout *horizontalLayout_4;
      QPushButton *_startOperation;
      QPushButton *_stopOperation;
      //XXXXXXXXXXXXXXXXXXXXXXXX

  };

}

#endif // MOUSEJUMPWIDGET_H
