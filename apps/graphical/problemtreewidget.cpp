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
        problemTree->header()->setStretchLastSection(true);
        problemTree->header()->hide();
        problemTree->setIconSize(QSize(20,20));
        problemTree->setColumnCount(1);
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

        linkMap.clear();

        problemTree->clear();
        //robots
        for (uint i = 0; i < workspace->getNumRobots(); i++) {
            if (addRobot2Tree(workspace->getRobot(i),false) == NULL) {
                problemTree->clear();
                return false;
            }
        }
        //obstacles
        for (uint i = 0; i < workspace->getNumObstacles(); i++) {
            if (addRobot2Tree(workspace->getObstacle(i),true) == NULL) {
                problemTree->clear();
                return false;
            }
        }
        problemTree->collapseAll();

        showDefaultTable();
        int height = problemTree->height()+infoTable->height()-90;
        problemTree->resize(problemTree->width(),height);
        infoTable->resize(infoTable->width(),90);

        return true;
    }


    void ProblemTreeWidget::updateInfoTable(QTreeWidgetItem *currentItem) {
        clearTable();
        if (currentItem != NULL) {
            QTreeWidgetItem *parentItem = currentItem->parent();
            if (parentItem != NULL) {
                if (parentItem->text(0) == "Links") {
                    if (linkMap.contains(currentItem)) {
                        Link *link = linkMap.value(currentItem);

                        //header
                        QTableWidgetItem *item;
                        item = new QTableWidgetItem(linkIcon,link->getName().c_str());
                        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                        item->setFlags(Qt::ItemIsEnabled);
                        infoTable->setColumnCount(1);
                        infoTable->setHorizontalHeaderItem(0,item);
                        infoTable->horizontalHeader()->show();

                        Link *parentLink = link->getParent();
                        if (parentLink == NULL) {//base link
                            //find robot
                            string robotName = parentItem->parent()->text(0).toStdString();
                            Robot *robot;
                            uint i = 0;
                            bool found = false;
                            while (!found && i < workspace->getNumRobots()) {
                                robot = workspace->getRobot(i);
                                if (robot->getName() == robotName) {
                                    found = true;
                                } else {
                                    ++i;
                                }
                            }

                            if (!found) {//it is an obstacle link
                                i = 0;
                                while (!found && i < workspace->getNumObstacles()) {
                                    robot = workspace->getObstacle(i);
                                    if (robot->getName() == robotName) {
                                        found = true;
                                    } else {
                                        ++i;
                                    }
                                }
                            }

                            if (found) {
                                //limits
                                addBaseDOFs2Table(robot);
                            }
                        } else {//not base link
                            infoTable->setRowCount(2);

                            //limits
                            addLinkLimits2Table(link);

                            //parent
                            addParent2Table(parentLink);
                        }

                        //children
                        addChildren2Table(link);

                        infoTable->verticalHeader()->setStretchLastSection(false);
                        infoTable->resizeRowsToContents();
                    } else {
                        showDefaultTable();
                    }
                } else {
                    showDefaultTable();
                }
            } else {
                showDefaultTable();
            }
        } else {
            showDefaultTable();
        }
    }


    QTreeWidgetItem *ProblemTreeWidget::addRobot2Tree(Robot *robot, bool isObstacle) {
        QTreeWidgetItem *item;
        //name
        item = addName2Tree(robot->getName(),isObstacle);
        if (item == NULL) return NULL;

        //scale
        if (addScale2Tree(robot->getScale(),item) == NULL) return NULL;

        //home
        if (addHome2Tree(robot->getHomePos()->first,item) == NULL) return NULL;

        //inverse kinematics
        addInvKin2Tree(robot->getIkine(),item);//No need for a robot to have inverse kinematics

        //links
        if (addLinks2Tree(robot,item) == NULL) return NULL;

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addName2Tree(string name, bool isObstacle) {
        QTreeWidgetItem *item = new QTreeWidgetItem(problemTree);
        item->setIcon(0,isObstacle?obstacleIcon:robotIcon);
        item->setText(0,name.c_str());
        item->setToolTip(0,isObstacle?"Obstacle name":"Robot name");

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addScale2Tree(KthReal scale, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setIcon(0,scaleIcon);
        item->setText(0,QString::number(scale));
        item->setToolTip(0,"Scale");

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addHome2Tree(SE3Conf homeConf, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setIcon(0,homeIcon);
        item->setToolTip(0,"Home configuration");

        QTreeWidgetItem *subItem;
        vector <KthReal> coord;

        QString text;
        coord = homeConf.getPos();
        for (uint i = 0; i < 3; i++) {
            text = homeLabels.at(i);
            text.append(" = ");
            text.append(QString::number(coord.at(i)));
            subItem = new QTreeWidgetItem(item);
            subItem->setText(0,text);
            subItem->setToolTip(0,"Position (meters)");
        }

        coord.clear();
        coord = homeConf.getAxisAngle();
        for (uint i = 0; i < 4; i++) {
            text = homeLabels.at(i+3);
            text.append(" = ");
            text.append(QString::number(coord.at(i)));
            subItem = new QTreeWidgetItem(item);
            subItem->setText(0,text);
            if (i < 3) {
                subItem->setToolTip(0,"Orientation axis");
            } else {
                subItem->setToolTip(0,"Orientation angle (radians)");
            }
        }

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addInvKin2Tree(InverseKinematic *invKin, QTreeWidgetItem *parentItem) {
        if (invKin != NULL) {
            if (invKin->type() != UNIMPLEMENTED || invKin->type() != NOINVKIN) {
                QString text = "InvKin: ";
                text.append(invKin->name().c_str());
                QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
                item->setText(0,text);
                item->setToolTip(0,"Inverse Kinematics");

                return item;
            }
        }

        return NULL;
    }


    QTreeWidgetItem *ProblemTreeWidget::addLinks2Tree(Robot *robot, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setText(0,"Links");
        for (uint i = 0; i < robot->getNumLinks(); i++) {
            if (addLink2Tree(robot->getLink(i),item) == NULL) return NULL;
        }

        return item;
    }


    QTreeWidgetItem *ProblemTreeWidget::addLink2Tree(Link *link, QTreeWidgetItem *parentItem) {
        QTreeWidgetItem *item = new QTreeWidgetItem(parentItem);
        item->setIcon(0,linkIcon);
        item->setText(0,link->getName().c_str());
        item->setToolTip(0,"Link name");
        linkMap.insert(item,link);

        return item;
    }

    void ProblemTreeWidget::showDefaultTable() {
        clearTable();
        infoTable->setColumnCount(1);
        infoTable->setRowCount(1);
        QString text = "Select an item from the tree to get more information ";
        text.append("\n(At present, only implemented for links)");
        QTableWidgetItem *defaultItem = new QTableWidgetItem(text);
        defaultItem->setTextAlignment(Qt::AlignCenter);
        defaultItem->setFlags(Qt::ItemIsEnabled);
        infoTable->verticalHeader()->setStretchLastSection(true);
        infoTable->setItem(0,0,defaultItem);
        infoTable->horizontalHeader()->hide();
    }


    void ProblemTreeWidget::clearTable() {
        for (int i = 0; i < infoTable->rowCount(); ++i) {
            for (int j = 0; j < infoTable->columnCount(); ++j) {
                delete infoTable->takeItem(i,j);
                delete infoTable->takeHorizontalHeaderItem(j);
            }
            delete infoTable->takeVerticalHeaderItem(i);
        }
        infoTable->clear();
    }


    void ProblemTreeWidget::addBaseDOFs2Table(Robot *robot) {
        infoTable->setRowCount(3);
        float low, high;
        QString text;
        QTableWidgetItem *item;
        for (uint i = 0; i < 3; ++i) {
            text = homeLabels.at(i);
            low = robot->getLimits(i)[0];
            high = robot->getLimits(i)[1];
            if (low == high) {//fixed
                text.append(" is fixed");
                item = new QTableWidgetItem(text);
            } else {//movable
                text.append(" limits = [");
                text.append(QString::number(low));
                text.append(", ");
                text.append(QString::number(high));
                text.append("]");
                item = new QTableWidgetItem(text);
                item->setToolTip("meters");
            }
            item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
            item->setFlags(Qt::ItemIsEnabled);
            infoTable->setItem(i,0,item);
        }
    }


    void ProblemTreeWidget::addLinkLimits2Table(Link *link) {
        QString text;
        QTableWidgetItem *item;
        if (link->getMovable()) {
            text = "Limits = [";
            text.append(QString::number(*link->getLimits(true)));
            text.append(", ");
            text.append(QString::number(*link->getLimits(false)));
            text.append("]");
            item = new QTableWidgetItem(text);
            item->setToolTip(link->getRotational()?"radians":"meters");
        } else {
            text = "Fixed";
            item = new QTableWidgetItem(text);
        }
        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        item->setFlags(Qt::ItemIsEnabled);
        infoTable->setItem(0,0,item);
    }


    void ProblemTreeWidget::addParent2Table(Link *parent) {
        QString text = "Parent link: ";
        text.append(parent->getName().c_str());
        QTableWidgetItem *item = new QTableWidgetItem(text);
        item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        item->setFlags(Qt::ItemIsEnabled);
        infoTable->setItem(1,0,item);
    }


    void ProblemTreeWidget::addChildren2Table(Link *link) {
        QString text;
        QTableWidgetItem *item;
        int numChilds = link->numChilds();
        if (numChilds > 0) {
            uint rows = infoTable->rowCount();
            if (numChilds == 1) {
                infoTable->setRowCount(rows+1);
                text = "Children link: ";
                text.append(link->getChild(0)->getName().c_str());
                item = new QTableWidgetItem(text);
                item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                item->setFlags(Qt::ItemIsEnabled);
                infoTable->setItem(rows,0,item);
            } else {
                infoTable->setRowCount(rows+1+numChilds);
                item = new QTableWidgetItem("Children links:");
                item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                item->setFlags(Qt::ItemIsEnabled);
                infoTable->setItem(rows,0,item);

                ++rows;
                for (uint i = 0; i < link->numChilds(); ++i) {
                    text = "    ";
                    text.append(link->getChild(i)->getName().c_str());
                    item = new QTableWidgetItem(text);
                    item->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
                    item->setFlags(Qt::ItemIsEnabled);
                    infoTable->setItem(rows+i,0,item);
                }
            }
        }
    }


    void ProblemTreeWidget::writeGUI(string text){
        emit sendText(text);
    }
}
