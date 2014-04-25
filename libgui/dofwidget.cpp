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
 
#include <QtGui>
#include "dofwidget.h"
#include <QString>


namespace Kautham {

    DOFWidget::DOFWidget(Robot* rob) {
        names = QString(rob->getDOFNames().c_str()).split("|");

        QWidget* tmpWid = new QWidget();

        gridLayout = new QGridLayout(tmpWid);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

        vboxLayout = new QVBoxLayout();
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));

        QLabel* tmpLab;
        QSlider* tmpSli;
        QString name;

        low.resize(names.size());
        high.resize(names.size());
        for (uint i = 0; i < 3; i++) {
            //label value will be defined in milimeters
            low[i] = rob->getLimits(i)[0];
            high[i] = rob->getLimits(i)[1];
        }
        for (uint i = 3; i < 6; i++) {
            //label value will be unitary and adimesional
            low[i] = 0.;
            high[i] = 1.;
        }
        double to_degrees = 180./M_PI;
        for (uint i = 6; i < names.size(); i++) {
            //label value will be defined in degrees
            low[i] = *rob->getLink(i-5)->getLimits(true);
            high[i] = *rob->getLink(i-5)->getLimits(false);

            if (rob->getLink(i-5)->getRotational()) {
                low[i] *= to_degrees;
                high[i] *= to_degrees;
            }
        }

        values.resize(names.size());
        labels.resize(names.size());
        sliders.resize(names.size());
        for (uint i = 0; i < names.size(); i++){
            values[i] = rob->getOffMatrix()[i];

            tmpLab = new QLabel(this);
            name = names.at(i).toUtf8().constData();
            tmpLab->setObjectName(name);
            vboxLayout->addWidget(tmpLab);
            labels[i] = tmpLab;

            tmpSli = new QSlider(this);
            tmpSli->setObjectName(names.at(i).toUtf8().constData());
            tmpSli->setOrientation(Qt::Horizontal);
            tmpSli->setMinimum(0);
            tmpSli->setMaximum(1000);
            tmpSli->setSingleStep(10);
            tmpSli->setValue(500);
            QObject::connect(tmpSli,SIGNAL(valueChanged(int)),SLOT(sliderChanged(int)));
            vboxLayout->addWidget(tmpSli);
            sliders[i] = tmpSli;
        }

        setValues(values);

        gridLayout->addLayout(vboxLayout, 0, 1, 1, 1);

        QScrollArea* scrollArea = new QScrollArea();
        scrollArea->setWidget(tmpWid);
        scrollArea->setWidgetResizable(true);

        QGridLayout *grid;
        grid = new QGridLayout(this);
        grid->addWidget(scrollArea);
    }

    DOFWidget::~DOFWidget(){
        for (uint i = 0; i < sliders.size(); i++){
            delete (QSlider*)sliders[i];
            delete (QLabel*)labels[i];
        }
        values.clear();
    }

    void DOFWidget::sliderChanged(int value){
        //values only can be changed if the control sliders changed
        setValues(values);
    }

    void DOFWidget::writeGUI(string text){
      emit sendText(text);
    }

    void DOFWidget::setValues(vector<KthReal> &val){
        if (val.size() == sliders.size()){
            double realval;
            for(uint i = 0; i < val.size(); i++) {
                values[i] = val.at(i);
                if (values[i] > 1.0) values[i] = 1.0;
                if (values[i] < 0.0) values[i] = 0.0;

                ((QSlider*)sliders[i])->setValue((int)(values[i]*1000.0));

                realval = low[i] + values[i]*(high[i]-low[i]);
                ((QLabel*)labels[i])->setText(QString(names[i] + " = " +
                                                      QString::number(realval,'f',3)));
            }
        }
    }

}

