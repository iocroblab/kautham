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
 
 
#include "controlwidget.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


using namespace std;


namespace Kautham {
    ControlWidget::ControlWidget(Problem *problem, vector<DOFWidget *> DOFWidgets,
                                 bool isRobotControlWidget, QWidget *parent,
                                 Qt::WindowFlags f):QWidget(parent, f) {
        prob = problem;
        isRobWidget = isRobotControlWidget;
        DOFWids = DOFWidgets;

        QVBoxLayout *mainLayout = new QVBoxLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QScrollArea *scrollArea = new QScrollArea();
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
        scrollArea->setWidgetResizable(true);
        mainLayout->addWidget(scrollArea);

        QWidget *scrollAreaWidget = new QWidget();
        scrollAreaWidget->setObjectName(QString::fromUtf8("scrollAreaWidget"));
        scrollArea->setWidget(scrollAreaWidget);

        QGridLayout *controlsLayout = new QGridLayout();
        controlsLayout->setObjectName(QString::fromUtf8("controlsLayout"));
        scrollAreaWidget->setLayout(controlsLayout);

        QStringList names;
        if (isRobWidget) {
            names = QString(prob->wSpace()->getRobControlsName().c_str()).split("|");
        } else {
            names = QString(prob->wSpace()->getObsControlsName().c_str()).split("|");
        }
        unsigned dim = names.size();
        sliders.resize(dim);
        lineEdits.resize(dim);
        values.resize(dim);
        QLabel *label;
        QLineEdit *lineEdit;
        QSlider *slider;
        QString name;
        QSignalMapper *lineEditSignalMapper = new QSignalMapper(this);
        QSignalMapper *sliderSignalMapper = new QSignalMapper(this);
        for (uint i = 0; i < dim; ++i) {
            name = names.at(i);

            label = new QLabel("<b>"+name+"</b>");
            label->setObjectName(name+"Label");
            controlsLayout->addWidget(label,2*i,0,1,1);

            lineEdit = new QLineEdit("0.500");
            lineEdit->setObjectName(name+"LineEdit");
            connect(lineEdit,SIGNAL(editingFinished()),lineEditSignalMapper,SLOT(map()));
            lineEditSignalMapper->setMapping(lineEdit,i);
            controlsLayout->addWidget(lineEdit,2*i,1,1,1);
            lineEdits[i] = lineEdit;

            slider = new QSlider();
            slider->setObjectName(name+"slider");
            slider->setOrientation(Qt::Horizontal);
            slider->setMinimum(0);
            slider->setMaximum(1000);
            slider->setSingleStep(1);
            slider->setPageStep(1);
            slider->setValue(500);
            connect(slider,SIGNAL(valueChanged(int)),sliderSignalMapper,SLOT(map()));
            sliderSignalMapper->setMapping(slider,i);
            controlsLayout->addWidget(slider,2*i+1,0,1,2);
            sliders[i] = slider;

            values[i] = 0.5;
        }
        connect(lineEditSignalMapper,SIGNAL(mapped(int)),this,SLOT(lineEditChanged(int)));
        connect(sliderSignalMapper,SIGNAL(mapped(int)),this,SLOT(sliderChanged(int)));

        QIcon updateIcon;
        updateIcon.addFile(":/icons/reload_16x16.png");
        updateIcon.addFile(":/icons/reload_22x22.png");

        QPushButton *updateButton = new QPushButton(updateIcon,"Update");
        if (isRobWidget) {
            updateButton->setToolTip("Update to last moved sample");
        } else {
            updateButton->setToolTip("Update to initial sample");
        }
        updateButton->setObjectName(QString::fromUtf8("btnUpdate"));
        connect(updateButton,SIGNAL(clicked()),this,SLOT(updateControls()));
        mainLayout->addWidget(updateButton);
    }


    ControlWidget::~ControlWidget(){
        for (uint i = 0; i < values.size(); ++i) {
            delete (QSlider *)sliders[i];
            delete (QLineEdit *)lineEdits[i];
        }
        sliders.clear();
        lineEdits.clear();

        values.clear();

        for (uint i = 0; DOFWids.size(); ++i) {
            delete (DOFWidget *)DOFWids.at(i);
        }
        DOFWids.clear();
    }


    void ControlWidget::sliderChanged(int index) {
        if (((int)(values[index]*1000.0)) != ((QSlider *)sliders[index])->value()) {
            values[index] = (KthReal)((QSlider *)sliders[index])->value()/1000.0;
            ((QLineEdit *)lineEdits[index])->setText(QString::number(values[index],'f',3));

            if (isRobWidget) {
                prob->setCurrentRobControls(values);
            } else {
                prob->setCurrentObsControls(values);
            }

            Sample *sample = new Sample(values.size());
            sample->setCoords(values);

            vector <KthReal> params;
            if (isRobWidget) {
                prob->wSpace()->moveRobotsTo(sample);
                for (uint i = 0; i < DOFWids.size(); ++i) {
                    prob->wSpace()->getRobot(i)->control2Parameters(values,params);
                    DOFWids.at(i)->setValues(params);
                }
            } else {
                prob->wSpace()->moveObstaclesTo(sample);
                for (uint i = 0; i < DOFWids.size(); ++i) {
                    prob->wSpace()->getObstacle(i)->control2Parameters(values,params);
                    DOFWids.at(i)->setValues(params);
                }
            }
        }
    }


    void ControlWidget::lineEditChanged(int index) {
        bool ok;
        KthReal value = (((QLineEdit *)lineEdits[index])->text()).toFloat(&ok);

        if (ok) {
            if (value > 1.0) {
                value = 1.0;
                writeGUI("Controls values must be between 0 and 1");
            } else if (value < 0.0) {
                value = 0.0;
                writeGUI("Controls values must be between 0 and 1");
            }

            values[index] = value;
            ((QLineEdit *)lineEdits[index])->setText(QString::number(value,'f',3));
            ((QSlider *)sliders[index])->setValue((int)(value*1000.0));

            if (isRobWidget) {
                prob->setCurrentRobControls(values);
            } else {
                prob->setCurrentObsControls(values);
            }

            Sample *sample = new Sample(values.size());
            sample->setCoords(values);

            vector <KthReal> params;
            if (isRobWidget) {
                prob->wSpace()->moveRobotsTo(sample);
                for (uint i = 0; i < DOFWids.size(); ++i) {
                    prob->wSpace()->getRobot(i)->control2Parameters(values,params);
                    DOFWids.at(i)->setValues(params);
                }
            } else {
                prob->wSpace()->moveObstaclesTo(sample);
                for (uint i = 0; i < DOFWids.size(); ++i) {
                    prob->wSpace()->getObstacle(i)->control2Parameters(values,params);
                    DOFWids.at(i)->setValues(params);
                }
            }
        } else {
            ((QLineEdit *)lineEdits[index])->setText(QString::number(values[index],'f',3));
            writeGUI("Please enter a valid control value");
        }
    }


    void ControlWidget::updateControls(){


//        //  Auxiliar lambda function for pose ploting
//        auto plot_pose = [](const mt::Transform* pose, const char* pose_name) {
//            std::cout << "-- " << pose_name << " ------------------------" << std::endl;
//            for (unsigned int i=0; i<3; ++i){
//                for (unsigned int j=0; j<3; ++j)    std::cout << std::setw(12) << pose->getRotation().getMatrix()[i][j] << "  ";
//                std::cout << "     " << std::setw(12) << pose->getTranslation()[i] << std::endl;
//            }
//        };

////        std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
//        Link* tmp_link_7 = prob->wSpace()->getRobot(0)->getLink(7);
//        plot_pose(&prob->wSpace()->getRobot(0)->getLastLinkTransform(), "Right arm TCP tf");
//        plot_pose(&prob->wSpace()->getRobot(1)->getLastLinkTransform(), "Left arm TCP tf");

//        Eigen::VectorXf q(7);
//        for (unsigned int i=1; i<8; ++i)    q(i-1) = prob->wSpace()->getRobot(0)->getLink(i)->getValue();
//        std::cout << "updateControls q = " << q.transpose() << std::endl;



        Sample *sample;
        if (isRobWidget) {
            sample = prob->wSpace()->getLastRobSampleMovedTo();
        } else {
            sample = prob->wSpace()->getInitObsSample();
        }

        if (sample != NULL){
            setValues(sample->getCoords());
        }
    }


    void ControlWidget::setValues(vector<KthReal> coords){
        for (uint i = 0; i < coords.size(); ++i) {
            values[i] = coords[i];

            ((QSlider *)sliders[i])->setValue((int)(values[i]*1000.0));

            ((QLineEdit *)lineEdits[i])->setText(QString::number(values[i],'f',3));
        }

        if (isRobWidget) {
            prob->setCurrentRobControls(values);
        } else {
            prob->setCurrentObsControls(values);
        }

        Sample *sample = new Sample(values.size());
        sample->setCoords(values);

        vector <KthReal> params;
        if (isRobWidget) {
            prob->wSpace()->moveRobotsTo(sample);
            for (uint i = 0; i < DOFWids.size(); ++i) {
                prob->wSpace()->getRobot(i)->control2Parameters(values,params);
                ((DOFWidget *)DOFWids.at(i))->setValues(params);
            }
        } else {
            prob->wSpace()->moveObstaclesTo(sample);
            for (uint i = 0; i < DOFWids.size(); ++i) {
                prob->wSpace()->getObstacle(i)->control2Parameters(values,params);
                ((DOFWidget *)DOFWids.at(i))->setValues(params);
            }
        }
    }


    void ControlWidget::writeGUI(string text){
        emit sendText(text);
    }
}
