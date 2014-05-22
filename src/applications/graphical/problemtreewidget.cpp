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


#include "problemtreewidget.h"


namespace Kautham {
    ProblemTreeWidget::ProblemTreeWidget(Qt::Orientation orientation,
                                         QWidget *parent):QSplitter(orientation, parent) {
        problemTree = new QTreeWidget();
        problemTree->setObjectName(QString::fromUtf8("problemTree"));
        problemTree->setIconSize(QSize(20,20));
        problemTree->setEnabled(true);
        problemTree->header()->hide();
        addWidget(problemTree);

        QTreeWidgetItem *___qtreewidgetitem = problemTree->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("kauthamMain", "Values", 0, QApplication::UnicodeUTF8));
        ___qtreewidgetitem->setText(0, QApplication::translate("kauthamMain", "Attributes", 0, QApplication::UnicodeUTF8));

        infoTable = new QTableWidget();
        addWidget(infoTable);

        workspace = NULL;
        robotIcon.addFile(":/icons/robot.png");
        obstacleIcon.addFile(":/icons/obstacle.png");
        scaleIcon.addFile(":/icons/scale.png");
        homeIcon.addFile(":/icons/home.png");
        linkIcon.addFile(":/icons/link.png");
        homeLabels = QString("X Y Z WX WY WZ TH").split(" ");
    }


    bool ProblemTreeWidget::setTree(WorkSpace *workSpace) {
        if (workSpace == NULL) return false;

        if (workspace != NULL && workspace != workSpace) delete workspace;

        workspace = workSpace;
        problemTree->clear();
        infoTable->clear();

        for (uint i = 0; i < workspace->getNumRobots(); i++) {
            if (addRobot(workspace->getRobot(i),false) == NULL) {
                problemTree->clear();
                return false;
            }
        }

        for (uint i = 0; i < workspace->getNumObstacles(); i++) {
            if (addRobot(workspace->getObstacle(i),true) == NULL) {
                problemTree->clear();
                return false;
            }
        }

        problemTree->expandAll();
        problemTree->resizeColumnToContents(0);
        problemTree->resizeColumnToContents(1);
        problemTree->collapseAll();

        return true;
    }


    QTreeWidgetItem *ProblemTreeWidget::addRobot(Robot *robot, bool isObstacle) {
        QTreeWidgetItem *item;

        item = addName(robot->getName(),isObstacle);
        if (item == NULL) return NULL;

        if (addScale(robot->getScale(),item) == NULL) return NULL;

        if (addHome(robot->getHomePos()->first,item) == NULL) return NULL;

        addInvKin(robot->getIkine(),item);

        if (addLinks(robot,item) == NULL) return NULL;

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addName(string name, bool isObstacle) {
        QTreeWidgetItem *item = new QTreeWidgetItem(problemTree);
        item->setToolTip(0,isObstacle?"Obstacle":"Robot");
        item->setIcon(0,isObstacle?obstacleIcon:robotIcon);
        item->setText(1,name.c_str());
        item->setToolTip(1,"Name");

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addScale(KthReal scale, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setToolTip(0,"Scale");
        item->setIcon(0,scaleIcon);
        item->setText(1,QString::number(scale));

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addHome(SE3Conf homeConf, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setToolTip(0,"Home");
        item->setIcon(0,homeIcon);

        QTreeWidgetItem *subItem;
        vector <KthReal> coord;

        coord = homeConf.getPos();
        for (uint i = 0; i < 3; i++) {
            subItem = new QTreeWidgetItem(item);
            subItem->setText(0,homeLabels.at(i));
            subItem->setToolTip(0,"Position");
            //Limits
            /*low = robot->getLimits(i)[0];
            high = robot->getLimits(i)[1];
            if (low != high) {
                tmp << " [" << low << ", " << high << "]";
            } else {
                tmp << " (fixed)";
            }*/
            subItem->setText(1,QString::number(coord.at(i)));
            subItem->setToolTip(1,"milimeters");
        }

        coord.clear();
        coord = homeConf.getAxisAngle();
        for (uint i = 0; i < 4; i++) {
            subItem = new QTreeWidgetItem(item);
            subItem->setText(0,homeLabels.at(i+3));
            subItem->setToolTip(0,"Orientation");
            subItem->setText(1,QString::number(coord.at(i)));
            subItem->setToolTip(1,"radians");
        }
    }


    QTreeWidgetItem *ProblemTreeWidget::addInvKin(InverseKinematic *invKin, QTreeWidgetItem *parentItem) {
        if (invKin != NULL) {
            if (invKin->type() != UNIMPLEMENTED || invKin->type() != NOINVKIN) {
                QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
                item->setText(0,"InvKin");
                item->setToolTip(0,"Inverse Kinematics");
                item->setText(1,invKin->name().c_str());
                item->setToolTip(1,"Name");

                return item;
            }
        }

        return NULL;
    }


    QTreeWidgetItem *ProblemTreeWidget::addLinks(Robot *robot, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setText(0,"Links");
        for (uint i = 0; i < robot->getNumLinks(); i++) {
            if (addLink(robot->getLink(i),item) == NULL) return NULL;
        }

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addLink(Link *link, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setToolTip(0,"Link");
        item->setIcon(0,linkIcon);
        item->setText(1,link->getName().c_str());
        item->setToolTip(1,"Name");

        return item;
    }
}
