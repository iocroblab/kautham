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
        problemTree->setColumnCount(2);
        //problemTree->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
        connect(problemTree,SIGNAL(itemCollapsed(QTreeWidgetItem *)),
                this,SLOT(resizeProblemTree()));
        connect(problemTree,SIGNAL(itemExpanded(QTreeWidgetItem *)),
                this,SLOT(resizeProblemTree()));
        connect(problemTree,SIGNAL(currentItemChanged(QTreeWidgetItem *,QTreeWidgetItem *)),
                this,SLOT(updateInfoTable(QTreeWidgetItem *)));
        addWidget(problemTree);

        infoTable = new QTableWidget();
        infoTable->setObjectName(QString::fromUtf8("infoTable"));
        infoTable->horizontalHeader()->setStretchLastSection(true);
        infoTable->horizontalHeader()->hide();
        infoTable->verticalHeader()->hide();
        infoTable->setIconSize(QSize(20,20));
        infoTable->setShowGrid(false);
        addWidget(infoTable);
        //infoTable->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
        //infoTable->resize(infoTable->width(),90);

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

        workspace = workSpace;

        problemTree->clear();
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

        problemTree->collapseAll();
        resizeProblemTree();

        showDefaultTable();
        int height = problemTree->height()+infoTable->height()-90;
        problemTree->resize(problemTree->width(),height);
        infoTable->resize(infoTable->width(),90);

        return true;
    }


    void ProblemTreeWidget::resizeProblemTree() {
        problemTree->resizeColumnToContents(0);
        problemTree->resizeColumnToContents(1);
    }


    void ProblemTreeWidget::updateInfoTable(QTreeWidgetItem *item) {
        showDefaultTable();

        if (item != NULL) {
            QTreeWidgetItem *parentItem = item->parent();
            if (parentItem != NULL) {
                if (parentItem->text(0) == "Links") {
                    if (linkMap.contains(item)) {
                        Link *link = linkMap.value(item);

                        QTableWidgetItem *item;
                        item = new QTableWidgetItem(linkIcon,link->getName().c_str());
                        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                        item->setFlags(Qt::ItemIsEnabled);
                        infoTable->setColumnCount(1);
                        infoTable->setHorizontalHeaderItem(0,item);
                        infoTable->horizontalHeader()->show();

                        QString text;
                        infoTable->setRowCount(2);

                        text = "Lower limit ";
                        text.append(QString::number(*link->getLimits(true)));
                        item = new QTableWidgetItem(text);
                        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                        item->setFlags(Qt::ItemIsEnabled);
                        infoTable->setItem(0,0,item);

                        text = "Higher limit ";
                        text.append(QString::number(*link->getLimits(false)));
                        item = new QTableWidgetItem(text);
                        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                        item->setFlags(Qt::ItemIsEnabled);
                        infoTable->setItem(1,0,item);

                        Link * tmpLink = link->getParent();
                        if (tmpLink == NULL) {

                        } else {
                            infoTable->setRowCount(infoTable->rowCount()+1);
                            text = "Parent link ";
                            text.append(tmpLink->getName().c_str());
                            item = new QTableWidgetItem(text);
                            item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                            item->setFlags(Qt::ItemIsEnabled);
                            infoTable->setItem(2,0,item);
                        }

                        int numChilds = link->numChilds();
                        if (numChilds > 0) {
                            uint rows = infoTable->rowCount();
                            if (numChilds == 1) {
                                infoTable->setRowCount(rows+1);
                                text = "Children link ";
                                text.append(link->getChild(0)->getName().c_str());
                                item = new QTableWidgetItem(text);
                                item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                                item->setFlags(Qt::ItemIsEnabled);
                                infoTable->setItem(rows,0,item);
                            } else {
                                infoTable->setRowCount(rows+1+numChilds);
                                item = new QTableWidgetItem("Children links");
                                item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                                item->setFlags(Qt::ItemIsEnabled);
                                infoTable->setItem(rows,0,item);

                                ++rows;
                                for (uint i = 0; i < link->numChilds(); ++i) {
                                    item = new QTableWidgetItem(link->getChild(i)->getName().c_str());
                                    item->setTextAlignment(Qt::AlignRight|Qt::AlignVCenter);
                                    item->setFlags(Qt::ItemIsEnabled);
                                    infoTable->setItem(rows+i,0,item);
                                }
                            }

                        }

                        infoTable->verticalHeader()->setStretchLastSection(false);
                        infoTable->resizeRowsToContents();
                    }
                }
            }
        }
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
        linkMap.insert(item,link);

        return item;
    }

    void ProblemTreeWidget::showDefaultTable() {
        for (uint i = 0; i < infoTable->rowCount(); ++i) {
            for (uint j = 0; j < infoTable->columnCount(); ++j) {
                delete infoTable->takeItem(i,j);
                delete infoTable->takeHorizontalHeaderItem(j);
            }
            delete infoTable->takeVerticalHeaderItem(i);
        }

        infoTable->clear();
        infoTable->setColumnCount(1);
        infoTable->setRowCount(1);
        QTableWidgetItem *defaultItem = new QTableWidgetItem("Select an item from the tree to see more information");
        defaultItem->setTextAlignment(Qt::AlignCenter);
        defaultItem->setFlags(Qt::ItemIsEnabled);
        infoTable->verticalHeader()->setStretchLastSection(true);
        infoTable->setItem(0,0,defaultItem);
        infoTable->horizontalHeader()->hide();
    }
}
