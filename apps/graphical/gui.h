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

#if !defined(_GUI_H)
#define _GUI_H


#include <vector>
#include <string>

#include <QtGui>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>

#ifndef Q_MOC_RUN
#include "ui_RobotSim.h"
#include <kautham/planner/planner.h>
#include <kautham/problem/problem.h>
#include <kautham/sampling/sampling.h>
#include <kautham/util/libkin/inversekinematic.h>
#include <kautham/problem/robot.h>
#include "controlwidget.h"
#include "plannerwidget.h"
#include "defaultpathdialog.h"
#endif

using namespace std;

namespace Kautham {

/** \addtogroup Application
 *  @{
 */

	enum WHERETYPE{
		TOOLBAR,
		FILEMENU,
		ACTIONMENU,
		FILETOOL,
        ACTIONTOOL,
        RECENTFILESMENU
	};

    struct Viewer{
        SoQtExaminerViewer *window;
		SoSeparator *root;
		string title;
		QWidget *tab;
    Viewer(){
      window = NULL;
      root = NULL;
      title = "";
      tab = NULL;
    }
    //~Viewer(){
    //  root = NULL;
    //  title = "";
    //  tab = NULL;
    //  if(window != NULL) delete window;
    //}
	};

	class GUI:public QMainWindow, private Ui::kauthamMain {
	  Q_OBJECT
    signals:
      void stopSimulation();

	public slots:
    void                setText(string s);
    void                about();
    void                plannersparameters();
    void                changeActiveBackground();
    void                scene2VRML();
    void                changeDockAreaForOutput(Qt::DockWidgetArea area);
    //! Enables/Disables the use of bounding boxes as collision models
    void                toogleBBOXflag();
    //! Sets the directories where robot and obstacles models will be looked for
    void                setModelsDefaultPath();
    //! Changes cursor to waiting/arrow cursor
    void changeCursor(bool waiting);
    void                attachObject();

    public:
    //GUI(QWidget *p=0);
    GUI();
    void stopPathSimulation() {emit stopSimulation();}
    QString getFilePath();
    void                clearText();
    bool                addViewerTab(string title, SoSeparator *root);
    void                removePropTab(string title);
    void                removeViewerTab(string title);
    SoQtExaminerViewer* getViewerTab(string title);
    SoSeparator*        getRootTab(string title);
    //bool              setTable(string s);
    bool                setAction(WHERETYPE typ, QAction *ac);
    bool                setAction(WHERETYPE typ, string name, string shortcut, QIcon icon,
                        QObject* receiver, const char *member);

    //! Sets menuRecentFiles as a submenu from menuBar
    void                setRecentFilesMenu();

    //! Shows/hides recent files menu
    void                showRecentFiles(bool visible);

    bool                setToogleAction(WHERETYPE typ, string name, string shortcut, string iconame,
                        QObject* receiver, const char *member);
    bool                addSeparator(WHERETYPE typ);
    bool                restart();
    bool                addToProblemTree(WorkSpace *workSpace);
    bool                addRobControlWidget(Problem* prob, vector<Kautham::DOFWidget *> robDOFWidgets);
    bool                addObsControlWidget(Problem* prob, vector<Kautham::DOFWidget *> obsDOFWidgets);

    bool                addConstrainedControlWidget( Robot* rob, Problem* prob);
    Kautham::DOFWidget *addDOFWidget( Robot* rob );
    bool                setSampleWidget(Problem* problem);
    bool                addPlanner(Planner *plan, SampleSet* samp);

    bool                addInverseKinematic(InverseKinematic* ikine);
    bool                addWidgetToTab(QWidget* widget, QString name);

    const mt::Transform getActiveCameraTransfom();
    bool                setActiveCameraPosition(float x, float y, float z );
    bool                setActiveCameraRotation(float qx, float qy, float qz, float qw);
    bool                setActiveCameraPointAt(float x, float y, float z );
    bool                setActiveCameraTransform(mt::Transform tra);
    std::string         getActiveViewTitle();
	
    ControlWidget*		getRobControlWidget();
    ControlWidget*		getObsControlWidget();
    PlannerWidget*		getPlannerWidget();

    //! Hides the Introduction Tab and shows the Properties and DOF tabs
    void showProblemAppearance();

    //! Shows the Introduction Tab and hides the Properties and DOF tabs
    void showInitialAppearance();

    int			 		indexRobControlsTab;
    int			 		indexObsControlsTab;
    int                 indexPlannerTab;

  private:
    vector<Viewer>      viewers;
    bool                boolPlanVis;
	};

    /** @}   end of Doxygen module "Application" */
}

#endif  //_GUI_H


