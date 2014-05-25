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


namespace Kautham {
    DOFWidget::DOFWidget(Robot* robot):QWidget() {
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

        low.resize(names.size());
        high.resize(names.size());
        for (uint i = 0; i < 3; i++) {
            //label value will be defined in milimeters
            low[i] = robot->getLimits(i)[0];
            high[i] = robot->getLimits(i)[1];
        }
        for (uint i = 3; i < 6; i++) {
            //label value will be unitary and adimesional
            low[i] = 0.;
            high[i] = 1.;
        }
        for (uint i = 6; i < names.size(); i++) {
            //label value will be defined in radians
            low[i] = *robot->getLink(i-5)->getLimits(true);
            high[i] = *robot->getLink(i-5)->getLimits(false);
        }

        QString name;
        QFrame *frame;
        QVBoxLayout *frameLayout;
        QLabel *label;
        values.resize(names.size());
        labels.resize(names.size());
        for (uint i = 0; i < names.size(); i++){
            values[i] = robot->getOffMatrix()[i];

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

            label = new QLabel("<b>"+name+"</b>");
            label->setObjectName(name+"Name");
            frameLayout->addWidget(label);

            label = new QLabel();
            label->setObjectName(name+"Value");
            if (i < 3) {
                label->setToolTip("milimeters");
            } else if (i > 5) {
                label->setToolTip(robot->getLink(i-6)->getRotational()?"radians":"milimeters");
            }
            label->setIndent(9);
            frameLayout->addWidget(label);
            labels[i] = label;
        }

        setValues(values);
    }

    DOFWidget::~DOFWidget() {
        for (uint i = 0; i < values.size(); i++) {
            delete (QLabel*)labels[i];
        }
        values.clear();
        low.clear();
        high.clear();
    }


    void DOFWidget::writeGUI(string text) {
        emit sendText(text);
    }


    void DOFWidget::setValues(vector<KthReal> &val) {
        if (val.size() == labels.size()) {
            double realval;
            for (uint i = 0; i < val.size(); i++) {
                values[i] = val.at(i);
                if (values[i] > 1.0) values[i] = 1.0;
                if (values[i] < 0.0) values[i] = 0.0;

                realval = low[i] + values[i]*(high[i]-low[i]);
                ((QLabel*)labels[i])->setText(QString::number(realval,'f',3));
            }
        }
    }
}

