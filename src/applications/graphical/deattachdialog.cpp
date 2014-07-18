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


#include "deattachdialog.h"


namespace Kautham {
    DeattachDialog::DeattachDialog(QWidget *parent, Qt::WindowFlags f):QDialog(parent, f) {
        wSpace = NULL;
        obs = rob = link = 0;

        setWindowTitle("Choose the object to be attached and the link where will be attached");
        setModal(true);
        setObjectName(QString::fromUtf8("DeattachDialog"));

        QGridLayout *mainLayout = new QGridLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QLabel *label = new QLabel("Attach obstacle");
        label->setObjectName(QString::fromUtf8("obstacleLabel"));
        mainLayout->addWidget(label,0,0);

        obstacleBox = new QComboBox();
        obstacleBox->setObjectName(QString::fromUtf8("obstacleBox"));
        obstacleBox->setEditable(false);
        connect(obstacleBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeObstacle(int)));
        mainLayout->addWidget(obstacleBox,0,1);

        label = new QLabel("to robot");
        label->setObjectName(QString::fromUtf8("robotLabel"));
        mainLayout->addWidget(label,1,0);

        robotBox = new QComboBox();
        robotBox->setObjectName(QString::fromUtf8("robotBox"));
        robotBox->setEditable(false);
        connect(robotBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeRobot(int)));
        mainLayout->addWidget(robotBox,1,1);

        label = new QLabel("at link");
        label->setObjectName(QString::fromUtf8("linkLabel"));
        mainLayout->addWidget(label,2,0);

        linkBox = new QComboBox();
        linkBox->setObjectName(QString::fromUtf8("linkBox"));
        linkBox->setEditable(false);
        connect(linkBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeLink(int)));
        mainLayout->addWidget(linkBox,2,1);

        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok |
                                                           QDialogButtonBox::Cancel,
                                                           Qt::Horizontal);
        connect(buttonBox,SIGNAL(accepted()),this,SLOT(accept()));
        connect(buttonBox,SIGNAL(rejected()),this,SLOT(reject()));
        mainLayout->addWidget(buttonBox,3,0,1,2);
    }

    void DeattachDialog::set(WorkSpace *workSpace) {
        wSpace = workSpace;
        obs = rob = link = 0;

        fillObstacleBox();
        fillRobotBox();
        fillLinkBox();
    }

    bool DeattachDialog::run() {
        if (exec()) {
            return(wSpace->detachObstacleFromRobotLink(wSpace->getRobot(rob)->getName(),
                                                       wSpace->getRobot(rob)->getLink(link)->getName(),
                                                       obs));
        } else {
            return false;
        }
    }

    void DeattachDialog::fillObstacleBox() {
        obstacleBox->clear();
        for (uint i = 0; i < wSpace->getNumObstacles(); ++i) {
            obstacleBox->addItem(wSpace->getObstacle(i)->getName().c_str());
        }
        obstacleBox->setCurrentIndex(obs);
    }

    void DeattachDialog::fillRobotBox() {
        robotBox->clear();
        for (uint i = 0; i < wSpace->getNumRobots(); ++i) {
            robotBox->addItem(wSpace->getRobot(i)->getName().c_str());
        }
        robotBox->setCurrentIndex(rob);
    }

    void DeattachDialog::fillLinkBox() {
        linkBox->clear();
        Robot *robot = wSpace->getRobot(rob);
        for (uint i = 0; i < robot->getNumLinks(); ++i) {
            linkBox->addItem(robot->getLink(i)->getName().c_str());
        }
        linkBox->setCurrentIndex(link);
    }

    void DeattachDialog::changeObstacle(int index) {
        if (index < 0 || index >= wSpace->getNumObstacles()) {
            obstacleBox->setCurrentIndex(obs);
        } else {
            obs = index;
        }
    }

    void DeattachDialog::changeRobot(int index) {
        if (index < 0 || index >= wSpace->getNumRobots()) {
            robotBox->setCurrentIndex(rob);
        } else {
            rob = index;
            fillLinkBox();
        }
    }

    void DeattachDialog::changeLink(int index) {
        if (index < 0 || index >= wSpace->getRobot(rob)->getNumLinks()) {
            linkBox->setCurrentIndex(link);
        } else {
            link = index;
        }
    }
}
