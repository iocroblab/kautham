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


#include "attachobjectdialog.h"


namespace Kautham {
    SelectRobotLinkDialog::SelectRobotLinkDialog(QWidget *parent, Qt::WindowFlags f):QDialog(parent, f) {
        wSpace = NULL;
        robot = link = 0;

        setWindowTitle("Choose a link to attach the object to");
        setModal(true);
        setObjectName(QString::fromUtf8("SelectRobotLinkDialog"));

        QGridLayout *mainLayout = new QGridLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QLabel *label = new QLabel("Robot: ");
        label->setObjectName(QString::fromUtf8("robotLabel"));
        mainLayout->addWidget(label,0,0);

        robotBox = new QComboBox();
        robotBox->setObjectName(QString::fromUtf8("robotBox"));
        robotBox->setEditable(false);
        connect(robotBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeRobot(int)));
        mainLayout->addWidget(robotBox,0,1);

        label = new QLabel("Link: ");
        label->setObjectName(QString::fromUtf8("linkLabel"));
        mainLayout->addWidget(label,1,0);

        linkBox = new QComboBox();
        linkBox->setObjectName(QString::fromUtf8("linkBox"));
        linkBox->setEditable(false);
        connect(linkBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeLink(int)));
        mainLayout->addWidget(linkBox,1,1);

        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok |
                                                           QDialogButtonBox::Cancel,
                                                           Qt::Horizontal);
        connect(buttonBox,SIGNAL(accepted()),this,SLOT(accept()));
        connect(buttonBox,SIGNAL(rejected()),this,SLOT(reject()));
        mainLayout->addWidget(buttonBox,2,0,1,2);
    }


    void SelectRobotLinkDialog::set(WorkSpace *workSpace) {
        wSpace = workSpace;
        robot = link = 0;

        fillRobotBox();
        fillLinkBox();
    }


    bool SelectRobotLinkDialog::getRobotLink(uint *robotIndex, uint *linkIndex) {
        if (exec()) {
            *robotIndex = robot;
            *linkIndex = link;
            return true;
        } else {
            return false;
        }
    }


    void SelectRobotLinkDialog::fillRobotBox() {
        robotBox->clear();
        for (uint i = 0; i < wSpace->getNumRobots(); ++i) {
            robotBox->addItem(wSpace->getRobot(i)->getName().c_str());
        }
        robotBox->setCurrentIndex(robot);
    }

    void SelectRobotLinkDialog::fillLinkBox() {
        linkBox->clear();
        Robot *rob = wSpace->getRobot(robot);
        for (uint i = 0; i < rob->getNumLinks(); ++i) {
            linkBox->addItem(rob->getLink(i)->getName().c_str());
        }
        linkBox->setCurrentIndex(link);
    }


    void SelectRobotLinkDialog::changeRobot(int index) {
        if (index < 0 || index >= int(wSpace->getNumRobots())) {
            robotBox->setCurrentIndex(robot);
        } else {
            robot = index;
            fillLinkBox();
        }
    }


    void SelectRobotLinkDialog::changeLink(int index) {
        if (index < 0 || index >= int(wSpace->getRobot(robot)->getNumLinks())) {
            linkBox->setCurrentIndex(link);
        } else {
            link = index;
        }
    }


    AttachObjectDialog::AttachObjectDialog(QWidget *parent, Qt::WindowFlags f):QDialog(parent, f) {
        setWindowTitle("Manage the attached objects");
        setModal(true);
        setObjectName(QString::fromUtf8("AttachObjectDialog"));

        QGridLayout *mainLayout = new QGridLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QLabel *label = new QLabel("Attachable objects");
        label->setObjectName(QString::fromUtf8("attachableLabel"));
        label->setMaximumHeight(31);
        mainLayout->addWidget(label,0,0);

        label = new QLabel("Attached objects");
        label->setObjectName(QString::fromUtf8("attachedLabel"));
        label->setMaximumHeight(31);
        mainLayout->addWidget(label,0,2);

        QIcon obstacleIcon, robotIcon, linkIcon, attachIcon, detachIcon;
        obstacleIcon.addFile(":/icons/obstacle.png");
        robotIcon.addFile(":/icons/robot.png");
        linkIcon.addFile(":/icons/link.png");
        attachIcon.addFile(":/icons/right_16x16.png");
        attachIcon.addFile(":/icons/right_22x22.png");
        detachIcon.addFile(":/icons/left_16x16.png");
        detachIcon.addFile(":/icons/left_22x22.png");

        attachableTable = new QTableWidget();
        attachableTable->setObjectName(QString::fromUtf8("attachableTable"));
        attachableTable->setColumnCount(1);
        QTableWidgetItem *header = new QTableWidgetItem();
        header->setText("Obstacle");
        header->setIcon(obstacleIcon);
        header->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        header->setFlags(header->flags() ^ Qt::ItemIsEditable);
        attachableTable->setHorizontalHeaderItem(0,header);
        attachableTable->horizontalHeader()->setStretchLastSection(true);
        attachableTable->verticalHeader()->hide();
        attachableTable->setIconSize(QSize(20,20));
        attachableTable->setShowGrid(false);
        attachableTable->setSelectionBehavior(QTableWidget::SelectRows);
        attachableTable->setMinimumWidth(120);
        mainLayout->addWidget(attachableTable,1,0,4,1);

        attachedTable = new QTableWidget();
        attachedTable->setObjectName(QString::fromUtf8("attachedTable"));
        attachedTable->setColumnCount(3);
        header = new QTableWidgetItem();
        header->setText("Obstacle");
        header->setIcon(obstacleIcon);
        header->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        header->setFlags(header->flags() ^ Qt::ItemIsEditable);
        attachedTable->setHorizontalHeaderItem(0,header);
        header = new QTableWidgetItem();
        header->setText("Robot");
        header->setIcon(robotIcon);
        header->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        header->setFlags(header->flags() ^ Qt::ItemIsEditable);
        attachedTable->setHorizontalHeaderItem(1,header);
        header = new QTableWidgetItem();
        header->setText("Link");
        header->setIcon(linkIcon);
        header->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        header->setFlags(header->flags() ^ Qt::ItemIsEditable);
        attachedTable->setHorizontalHeaderItem(2,header);
        attachedTable->horizontalHeader()->setStretchLastSection(true);
        attachedTable->verticalHeader()->hide();
        attachedTable->setIconSize(QSize(20,20));
        attachedTable->setShowGrid(false);
        attachedTable->setSelectionBehavior(QTableWidget::SelectRows);
        attachedTable->setMinimumWidth(360);
        mainLayout->addWidget(attachedTable,1,2,4,1);

        QPushButton *button = new QPushButton(attachIcon,"");
        button->setObjectName(QString::fromUtf8("attachButton"));
        button->setToolTip("Attach");
        button->setMaximumWidth(31);
        connect(button,SIGNAL(clicked()),this,SLOT(attach()));
        mainLayout->addWidget(button,2,1);

        button = new QPushButton(detachIcon,"");
        button->setObjectName(QString::fromUtf8("detachButton"));
        button->setToolTip("Detach");
        button->setMaximumWidth(31);
        connect(button,SIGNAL(clicked()),this,SLOT(detach()));
        mainLayout->addWidget(button,3,1);

        dialog = new SelectRobotLinkDialog(this);
    }


    void AttachObjectDialog::set(WorkSpace *workSpace) {
        wSpace = workSpace;
        dialog->set(workSpace);
        obsMap.clear();

        for (int i = attachableTable->rowCount()-1; i >= 0; --i) {
            delete attachableTable->takeItem(i,0);
            attachableTable->removeRow(i);
        }
        Robot *obs;
        QTableWidgetItem *item;
        int index = 0;
        for (unsigned i = 0; i < wSpace->getNumObstacles(); ++i) {
            obs = wSpace->getObstacle(i);
            if (obs->isAttachable()) {
                item = new QTableWidgetItem(obs->getName().c_str());
                item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                attachableTable->insertRow(index);
                attachableTable->setItem(index,0,item);
                obsMap.insert(item,i);
                index++;
            }
        }
        attachableTable->setCurrentCell(0,0);

        for (int i = attachedTable->rowCount()-1; i >= 0; --i) {
            delete attachedTable->takeItem(i,0);
            delete attachedTable->takeItem(i,1);
            delete attachedTable->takeItem(i,2);
            attachedTable->removeRow(i);
        }
        attachedTable->setCurrentCell(0,0);
    }


    void AttachObjectDialog::attach() {
        int index = attachableTable->currentRow();
        QTableWidgetItem *item = attachableTable->item(index,0);
        if (item != NULL) {
            if (obsMap.contains(item)) {
                uint obs = obsMap.value(item);
                uint robot, link;
                if (dialog->getRobotLink(&robot,&link)) {
                    if (wSpace->attachObstacle2RobotLink(robot,link,obs)) {
                        index = attachableTable->currentRow();
                        item = attachableTable->takeItem(index,0);
                        attachableTable->removeRow(index);
                        attachableTable->setCurrentCell(0,0);

                        index = attachedTable->rowCount();
                        attachedTable->insertRow(index);
                        attachedTable->setItem(index,0,item);
                        item = new QTableWidgetItem(wSpace->getRobot(robot)->
                                                    getName().c_str());
                        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                        attachedTable->setItem(index,1,item);
                        item = new QTableWidgetItem(wSpace->getRobot(robot)->
                                                    getLink(link)->getName().c_str());
                        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                        attachedTable->setItem(index,2,item);
                        attachedTable->setCurrentCell(index,0);

                        writeGUI("Object could be attached successfully");
                    } else {
                        writeGUI("Object couldn't be attached");
                    }
                }
            }
        }
    }


    void AttachObjectDialog::detach() {
        int index = attachedTable->currentRow();
        QTableWidgetItem *item = attachedTable->item(index,0);
        if (item != NULL) {
            if (obsMap.contains(item)) {
                uint obs = obsMap.value(item);
                if (wSpace->detachObstacle(obs)) {
                    item = attachedTable->takeItem(index,0);
                    delete attachedTable->takeItem(index,1);
                    delete attachedTable->takeItem(index,2);
                    attachedTable->removeRow(index);
                    attachedTable->setCurrentCell(0,0);

                    index = attachableTable->rowCount();
                    attachableTable->insertRow(index);
                    attachableTable->setItem(index,0,item);
                    attachableTable->setCurrentCell(index,0);

                    writeGUI("Object could be detached successfully");
                } else {
                    writeGUI("Object couldn't be detached");
                }
            }
        }
    }


    void AttachObjectDialog::writeGUI(string text){
        emit sendText(text);
    }
}
