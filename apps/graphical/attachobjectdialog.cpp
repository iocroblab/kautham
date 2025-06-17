/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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
    
        // Clear attachableTable
        for (int i = attachableTable->rowCount()-1; i >= 0; --i) {
            delete attachableTable->takeItem(i,0);
            attachableTable->removeRow(i);
        }
    
        // Fill attachableTable (only attachable obstacles)
        int index = 0;
        for (const auto& element : wSpace->getObstaclesMap()) {
            Robot* obs = element.second;
            if (obs->isAttachable()) {
                QTableWidgetItem* item = new QTableWidgetItem(obs->getName().c_str());
                item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                attachableTable->insertRow(index);
                attachableTable->setItem(index,0,item);
                index++;
            }
        }
        if (attachableTable->rowCount() > 0)
            attachableTable->setCurrentCell(0,0);
    
        // Clear attachedTable
        for (int i = attachedTable->rowCount()-1; i >= 0; --i) {
            delete attachedTable->takeItem(i,0);
            delete attachedTable->takeItem(i,1);
            delete attachedTable->takeItem(i,2);
            attachedTable->removeRow(i);
        }
    
        // Fill attachedTable (all attached obstacles, with robot and link info)
        int attachedIndex = 0;
        for (const auto& element : wSpace->getObstaclesMap()) {
            Robot* obs = element.second;
            if (obs->isAttached()) {
                Robot* attachedRobot = obs->getRobotAttachedTo();
                Link* attachedLink = obs->getLinkAttachedTo();
    
                QTableWidgetItem* nameItem = new QTableWidgetItem(obs->getName().c_str());
                nameItem->setFlags(nameItem->flags() ^ Qt::ItemIsEditable);
    
                QString robotName = attachedRobot ? QString::fromStdString(attachedRobot->getName()) : "N/A";
                QString linkName = attachedLink ? QString::fromStdString(attachedLink->getName()) : "N/A";
    
                QTableWidgetItem* robotItem = new QTableWidgetItem(robotName);
                robotItem->setFlags(robotItem->flags() ^ Qt::ItemIsEditable);
    
                QTableWidgetItem* linkItem = new QTableWidgetItem(linkName);
                linkItem->setFlags(linkItem->flags() ^ Qt::ItemIsEditable);
    
                attachedTable->insertRow(attachedIndex);
                attachedTable->setItem(attachedIndex, 0, nameItem);
                attachedTable->setItem(attachedIndex, 1, robotItem);
                attachedTable->setItem(attachedIndex, 2, linkItem);
                attachedIndex++;
            }
        }
        if (attachedTable->rowCount() > 0)
            attachedTable->setCurrentCell(0,0);
    }


    void AttachObjectDialog::attach() {
        int index = attachableTable->currentRow();
        QTableWidgetItem *item = attachableTable->item(index,0);
        if (item != NULL) {
            std::string obsname = item->text().toStdString();
            uint robot, link;
            if (dialog->getRobotLink(&robot, &link)) {
                if (wSpace->attachObstacle2RobotLink(robot, link, obsname)) {
                    writeGUI("Object could be attached successfully");
                    set(wSpace);
                } else {
                    writeGUI("Object couldn't be attached");
                }
            }
        }
    }


    void AttachObjectDialog::detach() {
        int index = attachedTable->currentRow();
        QTableWidgetItem *item = attachedTable->item(index,0);
        if (item != NULL) {
            std::string obsname = item->text().toStdString(); // Get name directly from table
            if (wSpace->detachObstacle(obsname)) {
                writeGUI("Object could be detached successfully");
                set(wSpace); // Refresh tables from model state
            } else {
                writeGUI("Object couldn't be detached");
            }
        }
    }
    

    void AttachObjectDialog::writeGUI(string text){
        emit sendText(text);
    }
}
