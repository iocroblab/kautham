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

#if !defined(_PLANNERWIDGET_H)
#define _PLANNERWIDGET_H


#include "gui.h"
#include "kauthamwidget.h"
#include <kautham/planner/planner.h>
#include <QtWidgets>

namespace Kautham{

/** \addtogroup Application
 *  @{
 */
    
    class GUI;

	class PlannerWidget: public KauthamWidget{
		Q_OBJECT
    public:
        PlannerWidget(Problem* pr, GUI *g, bool camera = false);
        ~PlannerWidget();

    signals:
        void changeCursor(bool waiting);

    public slots:
        void stopSimulation();

    private slots:
        void getPath();
        void saveData();
        void moveAlongPath();
        void showSample(int index);
        void tryConnect();
        void chkCameraClick();
        void simulatePath();
        bool taskmotionPathLoad();

    private:
        //void stopSimulation();
        void startSimulation();
        void tryConnectIOC();
        void tryConnectOMPL();
        void tryConnectOMPLC();
        void tryConnectODE();
        void saveDataIOC();
        void saveDataOMPL();
        void saveDataOMPLC();
        void saveDataODE();
        QString getFilePath();
        bool setTable(string s);//reimplemented
        void loadSampleFromLine(Sample *Robsmp,std::stringstream  &confstream);

        PlannerWidget();
        QHBoxLayout *hboxLayout;
        QHBoxLayout *hboxLayout2;
        QPushButton *btnGetPath;
        QPushButton *btnSaveData;
        QPushButton *moveButton;
        QCheckBox   *chkCamera;
        QPushButton *btnLoadData;
        QSpinBox *globalFromBox, *globalToBox;
        QLabel *tmpLabel;
        Problem *_problem;
        Planner *_planner;
        SampleSet *_samples;
        QTimer *_plannerTimer;
        uint _stepSim;
        bool _ismoving;
        GUI *_thegui;

        // Added to provide access to the local Planner
        QLabel      *label;
        QSpinBox    *localFromBox;
        QLabel      *label_2;
        QSpinBox    *localToBox;
        QHBoxLayout *horizontalLayout_2;
        QPushButton *_cmbTry;
        QLabel      *connectLabel;
    };
    /** @}   end of Doxygen module "Application" */
}



#endif	//_PLANNERWIDGET_H
