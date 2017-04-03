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


#ifndef ATTACHOBJECTDIALOG_H
#define ATTACHOBJECTDIALOG_H


#include <QtGui>
#ifndef Q_MOC_RUN
#include <kautham/problem/problem.h>
#endif

namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
    * \brief The SelectRobotLinkDialog class allows the user to
    * select the robot link where an object will be attached to
    */
    class SelectRobotLinkDialog : public QDialog {
        Q_OBJECT
    public:
        /*!
        * \brief SelectRobotLinkDialog Constructs the dialog
        * \param parent Parent of the dialog
        * \param f Window flags
        */
        SelectRobotLinkDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

        /*!
        * \brief set Fills the dialog with information of the robots and links
        * \param workSpace where the robots and links are
        */
        void set(WorkSpace *workSpace);

        /*!
        * \brief getRobotLink executes the dialog
        * \param robotIndex index of the selected robot if the user closes correctly the dialog
        * \param linkIndex index of the selected link if the user closes correctly the dialog
        * \return true if the user closes correctly the dialog
        */
        bool getRobotLink(uint *robotIndex, uint* linkIndex );

    private slots:
        /*!
        * \brief changeRobot changes the index of the selected robot
        */
        void changeRobot(int index);

        /*!
        * \brief changeLink changes the index of the selected link
        */
        void changeLink(int index);

    private:
        /*!
         * \brief wSpace where robots and obstacles are stored
         */
        WorkSpace  *wSpace;

        /*!
        * \brief robotBox QComboBox with the robot names of the workspace
        */
        QComboBox *robotBox;

        /*!
        * \brief linkBox QComboBox with the link names of the workspace
        */
        QComboBox *linkBox;

        /*!
        * \brief robot index of selected robot
        */
        int robot;

        /*!
        * \brief link index of selected robot
        */
        int link;


        /*!
        * \brief fillRobotBox fill the robot QComboBox with the robots of wSpace
        */
        void fillRobotBox();

        /*!
        * \brief fillLinktBox fill the link QComboBox with the links of the selected robot
        */
        void fillLinkBox();
    };


    /*!
    * \brief The AttachObjectDialog class allows the user to detach and attach objects to robot links
    */
    class AttachObjectDialog : public QDialog {
        Q_OBJECT
    public:
        /*!
         * \brief AttachObjectDialog Constructs the dialog
         * \param parent Parent of the dialog
         * \param f Window flags
         */
        AttachObjectDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

        /*!
        * \brief set Fills the dialog with information of the objects, robots and links of the workspace
        * \param workSpace where the objects, robots and links are
        */
        void set(WorkSpace *workSpace);

    private slots:
        /*!
        * \brief attach tries to attach the selected object to the selected robot link
        */
        void attach();

        /*!
        * \brief detach tries to detach the selected object
        */
        void detach();

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string text);

    private:
        /*!
         * \brief workspace workspace where robots and obstacles are stored
         */
        WorkSpace  *wSpace;

        /*!
         * \brief attachableTable table of attachable objects
         */
        QTableWidget *attachableTable;

        /*!
         * \brief attachedTable table of attached objects
         */
        QTableWidget *attachedTable;

        /*!
        * \brief dialog lets the user select the robot link where an object will be attached to
        */
        SelectRobotLinkDialog *dialog;

        /*!
         * \brief obsMap map between table items and indexes of objects
         */
        QMap <QTableWidgetItem *, uint> obsMap;



        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);
    };
    /** @}   end of Doxygen module "Application" */
}
#endif // ATTACHOBJECTDIALOG_H
