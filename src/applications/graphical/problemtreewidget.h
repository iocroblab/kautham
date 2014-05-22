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
#include <problem/workspace.h>


namespace Kautham {
    class ProblemTreeWidget:public QSplitter {
        Q_OBJECT
    public:
        ProblemTreeWidget(Qt::Orientation orientation = Qt::Vertical,
                          QWidget *parent = 0);
        bool setTree(WorkSpace *workSpace);

    private slots:
        void resizeProblemTree();
        void updateInfoTable(QTreeWidgetItem *item);

    private:
        QTreeWidgetItem *addRobot(Robot *robot, bool isObstacle);
        QTreeWidgetItem *addName(string name, bool isObstacle);
        QTreeWidgetItem *addScale(KthReal scale, QTreeWidgetItem *parentItem);
        QTreeWidgetItem *addHome(SE3Conf homeConf, QTreeWidgetItem *parentItem);
        QTreeWidgetItem *addInvKin(InverseKinematic *invKin, QTreeWidgetItem *parentItem);
        QTreeWidgetItem *addLinks(Robot *robot, QTreeWidgetItem *parentItem);
        QTreeWidgetItem *addLink(Link*link, QTreeWidgetItem *parentItem);
        void showDefaultTable();

        QTreeWidget *problemTree;
        QTableWidget *infoTable;
        WorkSpace  *workspace;
        QMap <QTreeWidgetItem *, Link *> linkMap;
        QIcon robotIcon;
        QIcon obstacleIcon;
        QIcon scaleIcon;
        QIcon homeIcon;
        QIcon linkIcon;
        QStringList homeLabels;
    };
}
