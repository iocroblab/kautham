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

#ifndef BRONCHOWIDGET_H
#define BRONCHOWIDGET_H

#include <QWidget>
#include <QtGui>
#include <libgui/gui.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <vector>
#include <string>
#include <libproblem/robot.h>
#include <libproblem/problem.h>
#include <libkthutil/kauthamdefs.h>

using namespace Kautham;

namespace Ui {
    class bronchoWidget;
}

namespace GUIBRO {
    class bronchoWidget : public QWidget {
		Q_OBJECT

	private slots:
		void              alphaSliderChanged(int val);
		void              xiSliderChanged(int val);
		void              zetaSliderChanged(int val);
		void              zetaSliderChanged1();
		void              zetaSliderReleased();
		void              setNavMode(int state);
		void              setCameraMode(int state);
		void			  collisionCheck();
		void			  advanceBronchoscope();
	public:
        bronchoWidget(Robot* rob, Problem* prob, GUI *gui ); //QWidget *parent = 0
		~bronchoWidget();

	protected:
		void changeEvent(QEvent *e);
		void updateView();
		void updateLookAt();

	private:
        Ui::bronchoWidget *ui;
			//vector<QSlider*>  sliders;
			//vector<QLabel*>   labels;
			//QGridLayout       *gridLayout;
			//QVBoxLayout       *vboxLayout;
			vector<KthReal>   values;
			Robot*            _robot;
		Problem*          _ptProblem;
		QTimer*           timer;
		KthReal           lastZsliderPos;
		GUI*          _gui;
		bool		  _cameraView;
		mt::Transform _homeView;
		bool _updateValues;
		//int		  _stepAdvance;
	};
}

#endif // BRONCHOWIDGET_H
