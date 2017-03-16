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


/* Author: Nestor Garcia Hidalgo */


#include "dofwidget.h"


using namespace std;


namespace Kautham {
    DOFWidget::DOFWidget(Robot* robot, QWidget *parent, Qt::WindowFlags f):QWidget(parent, f) {
        setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Minimum);

        QGridLayout *gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        setLayout(gridLayout);

        QScrollArea* scrollArea = new QScrollArea();
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        gridLayout->addWidget(scrollArea);

        QWidget *scrollAreaWidget = new QWidget();
        scrollAreaWidget->setObjectName(QString::fromUtf8("scrollAreaWidget"));
        scrollArea->setWidget(scrollAreaWidget);
        scrollArea->setWidgetResizable(true);

        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->setObjectName(QString::fromUtf8("vBoxLayout"));
        vBoxLayout->setContentsMargins(6,6,6,6);
        vBoxLayout->setSpacing(6);
        scrollAreaWidget->setLayout(vBoxLayout);

        QStringList names = QString(robot->getDOFNames().c_str()).split("|");

        lowValues.resize(names.size());
        highValues.resize(names.size());
        for (uint i = 0; i < 3; ++i) {
            //label value will be defined in meters
            lowValues[i] = robot->getLimits(i)[0];
            highValues[i] = robot->getLimits(i)[1];
        }
        for (uint i = 3; i < 6; ++i) {
            //label value will be unitary and adimesional
            lowValues[i] = 0.;
            highValues[i] = 1.;
        }
        for (int i = 6; i < names.size(); ++i) {
            //label value will be defined in radians
            lowValues[i] = *robot->getLink(i-5)->getLimits(true);
            highValues[i] = *robot->getLink(i-5)->getLimits(false);
        }

        QString name;
        QFrame *frame;
        QVBoxLayout *frameLayout;
        QLabel *label;
        vector <KthReal> currentValues;
        currentValues.resize(names.size());
        labels.resize(names.size());
        for (int i = 0; i < names.size(); ++i){
            currentValues[i] = robot->getOffMatrix()[i];

            name = names.at(i);

            frame = new QFrame();
            frame->setObjectName(name+"Frame");
            frame->setFrameShape(QFrame::StyledPanel);
            vBoxLayout->addWidget(frame);

            frameLayout = new QVBoxLayout();
            frameLayout->setObjectName(name+"Layout");
            frameLayout->setContentsMargins(3,3,3,3);
            frameLayout->setSpacing(1);
            frame->setLayout(frameLayout);

            QPalette palette = frame->palette();
            palette.setColor(backgroundRole(),QColor(255,255,255) );
            frame->setPalette(palette);
            frame->setAutoFillBackground(true);

            label = new QLabel("<b>"+name+"</b>");
            label->setObjectName(name+"Name");
            frameLayout->addWidget(label);

            label = new QLabel();
            label->setObjectName(name+"Value");
            if (i < 3) {
                label->setToolTip("meters");
            } else if (i > 5) {
                label->setToolTip(robot->getLink(i-6)->getRotational()?"radians":"meters");
            }
            label->setIndent(9);
            frameLayout->addWidget(label);
            labels[i] = label;
        }

        setValues(currentValues);
    }


    DOFWidget::~DOFWidget() {
        for (uint i = 0; i < lowValues.size(); ++i) {
            delete (QLabel*)labels[i];
        }
        lowValues.clear();
        highValues.clear();
    }


    void DOFWidget::setValues(vector<KthReal> &values) {
        if (values.size() == labels.size()) {
            double value;
            for (uint i = 0; i < values.size(); ++i) {
                value = values.at(i);
                if (value > 1.0) value = 1.0;
                if (value < 0.0) value = 0.0;

                value = lowValues[i] + value*(highValues[i]-lowValues[i]);
                ((QLabel*)labels[i])->setText(QString::number(value,'f',3));
            }
        }
    }


    void DOFWidget::writeGUI(string text) {
        emit sendText(text);
    }
}

