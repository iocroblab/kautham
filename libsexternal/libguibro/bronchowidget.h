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

using namespace libGUI;

namespace Ui {
	class bronchoWidget;
}

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
		bronchoWidget(Robot* rob, Problem* prob, int offset, GUI* gui ); //QWidget *parent = 0
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
	int               _globalOffset;
		Problem*          _ptProblem;
		QTimer*           timer;
		KthReal           lastZsliderPos;
		GUI*          _gui;
		bool		  _cameraView;
		mt::Transform _homeView;
		bool _updateValues;
		//int		  _stepAdvance;
	};

#endif // BRONCHOWIDGET_H
