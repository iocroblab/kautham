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


#include <string>

#include <QtWidgets>
#include <QFile>
#include <QObject>

#include <Inventor/Qt/SoQt.h>

#ifndef Q_MOC_RUN
#include <kautham/problem/problem.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include "gui.h"
#endif


using namespace std;
using namespace Kautham;

/** \addtogroup Application
 *  @{
 */
class Application:public QObject {
	Q_OBJECT
public:
  Application();
  ~Application();

public slots:
  void              openFile(QString problemFile = "");
  void              saveFile();
  void              saveAsFile();
  void              closeProblem();
  void              quit();
  //! Clears recent files list
  void clearRecentFiles();

private:
  void              initApp();
  void              setActions();
  //! Saves the background colors of the active tabs to the settings file
  void              saveTabColors();
  //!  This method setups the problem and create the WorkSpace, the Planner and
  //!  its corresponding LocalPlanner.
  bool              problemSetup(string problemFile);

  //! Updates recent files list
  void updateRecentFiles(string problemFile);

  //! Adds recent files to file menu
  void setRecentFilesAction();

  QSettings         *settings;
  QFile*            xmlFile;
  Problem*          _problem;
  GUI*              mainWindow;
  PROBLEMSTATE      appState;
  bool              abort;
};
 /** @}   end of Doxygen module "Application" */
#endif  //_APPLICATION_H
