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
 
 
#include <QtGui>
#include "controlwidget.h"
#include <QString>


namespace Kautham {

    ControlWidget::ControlWidget(Problem* prob, vector<DOFWidget*> DOFWidgets, bool robot) {
        _ptProblem = prob;
        robWidget = robot;
        _DOFWidgets = DOFWidgets;
        string names = "This|is|a|test";
        if (robot) {
            names = _ptProblem->wSpace()->getRobControlsName();
        } else {
            names = _ptProblem->wSpace()->getObsControlsName();
        }

        mainLayout = new QVBoxLayout(this);
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));

        scrollArea = new QScrollArea();
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
        scrollArea->setWidgetResizable(true);
        mainLayout->addWidget(scrollArea);

        scrollAreaWidget = new QWidget();
        scrollAreaWidget->setObjectName(QString::fromUtf8("scrollAreaWidget"));
        scrollArea->setWidget(scrollAreaWidget);

        controlsLayout = new QVBoxLayout(scrollAreaWidget);
        controlsLayout->setObjectName(QString::fromUtf8("controlsLayout"));

        QLabel *tempLabel;
        QSlider *tempSlider;
        QString content(names.c_str());
        QStringList cont = content.split("|");
        for (QStringList::const_iterator iterator = cont.constBegin();
             iterator != cont.constEnd(); ++iterator) {
            tempLabel = new QLabel();
            tempLabel->setObjectName((*iterator).toUtf8().constData());
            content = (*iterator).toUtf8().constData();
            tempLabel->setText(content.append(" = 0.5"));
            controlsLayout->addWidget(tempLabel);
            labels.push_back(tempLabel);

            tempSlider = new QSlider();
            tempSlider->setObjectName((*iterator).toUtf8().constData());
            tempSlider->setOrientation(Qt::Horizontal);
            tempSlider->setMinimum(0);
            tempSlider->setMaximum(1000);
            tempSlider->setSingleStep(1);
            tempSlider->setValue(500);
            connect(tempSlider,SIGNAL(valueChanged(int)),this,SLOT(sliderChanged(int)));
            controlsLayout->addWidget(tempSlider);
            sliders.push_back(tempSlider);
        }

        btnUpdate = new QPushButton();
        if (robWidget) {
            btnUpdate->setText("Update to last moved sample");
        } else {
            btnUpdate->setText("Update to initial sample");
        }
        btnUpdate->setObjectName(QString::fromUtf8("btnUpdate"));
        connect(btnUpdate,SIGNAL(clicked()),this,SLOT(updateControls()));
        mainLayout->addWidget(btnUpdate);

        values.resize(sliders.size());
        for(int i=0; i<values.size(); i++)
            values[i]=0.5;
    }

    ControlWidget::~ControlWidget(){
        for(uint i = 0; i < sliders.size(); i++) {
            delete (QSlider*)sliders[i];
            delete (QLabel*)labels[i];
        }
        sliders.clear();
        labels.clear();
        values.clear();
        for (uint i = 0; _DOFWidgets.size(); i++) {
            delete (DOFWidget*)_DOFWidgets[i];
        }
        _DOFWidgets.clear();
        delete mainLayout;
        delete scrollArea;
        delete controlsLayout;
        delete btnUpdate;
    }

    void ControlWidget::updateControls(){
        Sample *s;
        if (robWidget) {
            s  = _ptProblem->wSpace()->getLastRobSampleMovedTo();
        } else {
            s  = _ptProblem->wSpace()->getInitObsSample();
        }
        if (s != NULL){
            setValues(s->getCoords());
        }
    }

    void ControlWidget::sliderChanged(int value){
        QString tmp;
        for(unsigned int i=0; i<sliders.size(); i++){
            values[i]=(KthReal)((QSlider*)sliders[i])->value()/1000.0;

            tmp = labels[i]->text().left(labels[i]->text().indexOf("=") + 2);
            labels[i]->setText( tmp.append( QString().setNum(values[i],'g',5)));
        }

        if (robWidget) {
            _ptProblem->setCurrentRobControls(values);
        } else {
            _ptProblem->setCurrentObsControls(values);
        }

        Sample *sample = new Sample(values.size());
        sample->setCoords(values);

        vector <float> params;
        if (robWidget) {
            _ptProblem->wSpace()->moveRobotsTo(sample);
            for (uint i = 0; i < _DOFWidgets.size(); i++) {
                _ptProblem->wSpace()->getRobot(i)->control2Parameters(values,params);
                ((DOFWidget*)_DOFWidgets.at(i))->setValues(params);
            }
        } else {
            _ptProblem->wSpace()->moveObstaclesTo(sample);
            for (uint i = 0; i < _DOFWidgets.size(); i++) {
                _ptProblem->wSpace()->getObstacle(i)->control2Parameters(values,params);
                ((DOFWidget*)_DOFWidgets.at(i))->setValues(params);
            }
        }
    }

    void ControlWidget::setValues(vector<KthReal> coords){
        QString tmp;
        for(unsigned int i = 0; i < coords.size(); i++) {
            ((QSlider*)sliders[i])->setValue((int)(coords[i]*1000.0));

            tmp = labels[i]->text().left(labels[i]->text().indexOf("=") + 2);
            labels[i]->setText( tmp.append( QString().setNum(coords[i],'g',5)));
        }

        for (int j = 0; j < values.size(); j++)
            values[j] = coords[j];

        if (robWidget) {
            _ptProblem->setCurrentRobControls(values);
        } else {
            _ptProblem->setCurrentObsControls(values);
        }

        Sample *sample = new Sample(values.size());
        sample->setCoords(values);

        vector <float> params;
        if (robWidget) {
            _ptProblem->wSpace()->moveRobotsTo(sample);
            for (uint i = 0; i < _DOFWidgets.size(); i++) {
                _ptProblem->wSpace()->getRobot(i)->control2Parameters(values,params);
                ((DOFWidget*)_DOFWidgets.at(i))->setValues(params);
            }
        } else {
            _ptProblem->wSpace()->moveObstaclesTo(sample);
            for (uint i = 0; i < _DOFWidgets.size(); i++) {
                _ptProblem->wSpace()->getObstacle(i)->control2Parameters(values,params);
                ((DOFWidget*)_DOFWidgets.at(i))->setValues(params);
            }
        }
    }

}

