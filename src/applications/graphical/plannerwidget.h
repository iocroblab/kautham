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

#if !defined(_PLANNERWIDGET_H)
#define _PLANNERWIDGET_H

#include "kauthamwidget.h"
#include <planner/planner.h>
#include <QtGui>
#include "gui.h"


using namespace std;

namespace Kautham{

/** \addtogroup Application
 *  @{
 */
class GUI;//needed here because GUI has #include "plannerwidget.h"

	class PlannerWidget: public KauthamWidget{
		Q_OBJECT
	private slots:
    void getPathCall();
    void saveDataCall();
    void loadDataCall();
    //void setIniGoalCall();
    void moveAlongPath();
    void showSample(int index);
    void tryConnect();
    void chkCameraClick();
    public slots:
    void simulatePath();

	public:
    PlannerWidget(Planner* plan, SampleSet* samp, bool camera = false, GUI* gui = NULL );
    inline bool ismoving() {return _ismoving;}
		
	private:
    PlannerWidget();
    QHBoxLayout *hboxLayout;
    QHBoxLayout *hboxLayout2;
    QPushButton *btnGetPath;
    QPushButton *btnSaveData;
    QPushButton *btnMove;
    QCheckBox   *chkCamera;
    QPushButton *btnLoadData;
    QSpinBox *spnInit, *spnGoal;
    QLabel* tmpLabel;
    GUI*          _gui;
    Planner*      _planner;
    SampleSet*    _samples;
    QTimer*       _plannerTimer;
    unsigned int  _stepSim;
    bool          _ismoving;

    // Added to provide access to the local Planner
    QLabel      *label;
    QSpinBox    *_spFrom;
    QLabel      *label_2;
    QSpinBox    *_spTo;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *_cmbTry;
    QLabel      *_lblRes;
	
	};


    class PlannersWidget:public QWidget {
        Q_OBJECT
    public:
        PlannersWidget(Planner *planner, SampleSet *sampleSet, bool camera = false,
                       QWidget *parent = 0, Qt::WindowFlags f = 0);
        inline bool isMoving() {return _isMoving;}
    signals:
        void sendText(string text);

    private slots:
        void getPath();
        void saveData();
        void loadData();
        void moveAlongPath();
        void showSample(int index);
        void tryConnect();
        void setCamera();

    private:
        bool _isMoving;
        QComboBox *localFromBox;
        QComboBox *localToBox;
        QComboBox *globalPlannerBox;
        QComboBox *globalFromBox;
        QComboBox *globalToBox;
        QPushButton *moveButton;
        QComboBox *linkBox;
        QCheckBox *cameraCheckBox;

    };

    /** @}   end of Doxygen module "Application" */
}



#endif	//_PLANNERWIDGET_H
