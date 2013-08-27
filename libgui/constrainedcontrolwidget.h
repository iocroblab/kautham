#if !defined(_CONSTRAINEDCONTROLWIDGET_H)
#define _CONSTRAINEDCONTROLWIDGET_H

#include <QtGui>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <vector>
#include <string>
#include <libproblem/robot.h>
#include <libproblem/problem.h>
#include <libproblem/constrainedkinematic.h>
#include <libkthutil/kauthamdefs.h>


using namespace std;
using namespace Kautham;
using namespace libProblem;


namespace libGUI {
	class ConstrainedControlWidget:public QWidget{
		Q_OBJECT

	private slots:
		void              sliderChanged(int val);
	public:
		ConstrainedControlWidget( Robot* rob, Problem* prob, int offset );
		~ConstrainedControlWidget();
		inline vector<KthReal>   *getValues(){return &values;}
    void setValues(vector<KthReal> &val);
	private:
		ConstrainedKinematic *_conKin;
		vector<QSlider*>  sliders;
		vector<QLabel*>   labels;
		QGridLayout       *gridLayout;
		QVBoxLayout       *vboxLayout;
		vector<KthReal>   values;
		Robot*            _robot;
    int               _globalOffset;
    Problem*          _ptProblem;
	};


}

#endif  //_CONSTRAINEDCONTROLWIDGET_H


