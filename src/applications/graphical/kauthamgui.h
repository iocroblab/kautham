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


  
 
#if !defined(_APPLICATION_H)
#define _APPLICATION_H

#include <QtCore>
#include <problem/problem.h>
#include <libgui/gui.h>
#include <Inventor/Qt/SoQt.h>
#include <QtGui>
#include <QFile>
#include <QObject>
#include <util/kthutil/kauthamdefs.h>
#include <string>

using namespace std;
using namespace Kautham;

class Application:public QObject {
	Q_OBJECT
public:
  Application();
  ~Application();

public slots:
  void              openFile();
  void              saveFile();
  void              saveAsFile();
  void              closeProblem();

private:
  void              initApp();
  void              setActions();
  //! Saves the background colors of the active tabs to the settings file
  void              saveTabColors();
  //!  This method setups the problem and create the WorkSpace, the Planner and
  //!  its corresponding LocalPlanner.
  bool              problemSetup(string path);


  QSettings         *settings;
  QFile*            xmlFile;
  Problem*          _problem;
  GUI*              mainWindow;
  PROBLEMSTATE      appState;
};

#endif  //_APPLICATION_H
