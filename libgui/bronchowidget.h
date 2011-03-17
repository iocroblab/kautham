#ifndef BRONCHOWIDGET_H
#define BRONCHOWIDGET_H

#include <QWidget>
#include <QtGui>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <vector>
#include <string>
#include <libproblem/robot.h>
#include <libproblem/problem.h>
#include <libutil/kauthamdefs.h>

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
public:
    bronchoWidget(Robot* rob, Problem* prob, int offset); //QWidget *parent = 0
    ~bronchoWidget();

protected:
    void changeEvent(QEvent *e);

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
};

#endif // BRONCHOWIDGET_H
