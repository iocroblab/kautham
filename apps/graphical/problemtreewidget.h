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


#include <QtWidgets>
#include <kautham/problem/workspace.h>


#if !defined(_PROBLEMTREEWIDGET_H)
#define _PROBLEMTREEWIDGET_H

namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
     * \brief The ProblemTreeWidget class is the class that implements the Problem Tree Widget
     */
    class ProblemTreeWidget:public QSplitter {
        Q_OBJECT
    public:
        /*!
         * \brief ProblemTreeWidget Constructs the widget
         * \param orientation orientation of the widget
         * \param parent parent of the widget
         */
        ProblemTreeWidget(Qt::Orientation orientation = Qt::Vertical,
                          QWidget *parent = 0);

        /*!
         * \brief setTree sets the tree
         * \param workSpace workspace where robots and obstacles to add to the tree are stored
         * \return
         */
        bool setTree(WorkSpace *workSpace);

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string text);

    private slots:
        /*!
         * \brief updateInfoTable updates the information shown in the table
         * \param currentItem current item to show the information of
         */
        void updateInfoTable(QTreeWidgetItem *currentItem);

    private:
        /*!
         * \brief addRobot2Tree adds a robot/obstacle item to the tree
         * \param robot robot to add
         * \param isObstacle wether if the robot is an obstacle or not
         * \return
         */
        QTreeWidgetItem *addRobot2Tree(Robot *robot, bool isObstacle);

        /*!
         * \brief addName2Tree adds a robot/obstacle name item to the tree
         * \param name name of the robot/obstacle
         * \param isObstacle wether if the robot is an obstacle or not
         * \return
         */
        QTreeWidgetItem *addName2Tree(string name, bool isObstacle);

        /*!
         * \brief addScale2Tree adds a scale item to the tree
         * \param scale scale
         * \param parentItem parent of the item
         * \return
         */
        QTreeWidgetItem *addScale2Tree(KthReal scale, QTreeWidgetItem *parentItem);

        /*!
         * \brief addHome2Tree adds a home item to the tree
         * \param homeConf home configuration to add
         * \param parentItem parent of the itemm
         * \return
         */
        QTreeWidgetItem *addHome2Tree(SE3Conf homeConf, QTreeWidgetItem *parentItem);

        /*!
         * \brief addInvKin2Tree adds an inverse kinematics item to the tree
         * \param invKin inverse kinematics to add
         * \param parentItem parent of the itemm
         * \return
         */
        QTreeWidgetItem *addInvKin2Tree(InverseKinematic *invKin, QTreeWidgetItem *parentItem);

        /*!
         * \brief addLinks2Tree adds a links item to the tree
         * \param robot robot whose links will be added
         * \param parentItem parent of the itemm
         * \return
         */
        QTreeWidgetItem *addLinks2Tree(Robot *robot, QTreeWidgetItem *parentItem);

        /*!
         * \brief addLink2Tree adds a link item to the tree
         * \param link link to add
         * \param parentItem parent of the itemm
         * \return
         */
        QTreeWidgetItem *addLink2Tree(Link*link, QTreeWidgetItem *parentItem);

        /*!
         * \brief showDefaultTable show the default table appearance
         */
        void showDefaultTable();

        /*!
         * \brief clearTable clears the table
         */
        void clearTable();

        /*!
         * \brief addBaseDOFs2Table adds a base DOFs item to the table
         * \param robot robot whose base DOFs will be added
         */
        void addBaseDOFs2Table(Robot *robot);

        /*!
         * \brief addLinkLimits2Table adds a limits item to the table
         * \param link link whose limits will be added
         */
        void addLinkLimits2Table(Link *link);

        /*!
         * \brief addParent2Table adds a parent link item to the table
         * \param parent parent link to add
         */
        void addParent2Table(Link *parent);

        /*!
         * \brief addChildren2Table adds a children links item to the table
         * \param link link whose children parents will be added
         */
        void addChildren2Table(Link *link);

        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);


        /*!
         * \brief problemTree problem tree of the widget
         */
        QTreeWidget *problemTree;

        /*!
         * \brief infoTable table of the widget
         */
        QTableWidget *infoTable;

        /*!
         * \brief workspace workspace where robots and obstacles are stored
         */
        WorkSpace  *workspace;

        /*!
         * \brief linkMap map between tree items and links
         */
        QMap <QTreeWidgetItem *, Link *> linkMap;

        /*!
         * \brief robotIcon robot icon shown in the widget
         */
        QIcon robotIcon;

        /*!
         * \brief obstacleIcon obstacle icon shown in the widget
         */
        QIcon obstacleIcon;

        /*!
         * \brief scaleIcon scale icon shown in the widget
         */
        QIcon scaleIcon;

        /*!
         * \brief homeIcon home icon shown in the widget
         */
        QIcon homeIcon;

        /*!
         * \brief linkIcon link icon shown in the widget
         */
        QIcon linkIcon;

        /*!
         * \brief homeLabels names of robot base DOF's
         */
        QStringList homeLabels;
    };
    /** @}   end of Doxygen module "Application" */
}


#endif // _PROBLEMTREEWIDGET_H
