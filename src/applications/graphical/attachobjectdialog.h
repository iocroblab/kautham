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
#include <problem/problem.h>


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
        * \brief AttachDialog Constructs the dialog
        * \param parent Parent of the dialog
        * \param f Window flags
        */
        SelectRobotLinkDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

        void set(WorkSpace *workSpace);

        bool getRobotLink(uint *robotIndex, uint* linkIndex );

    private slots:
        void changeRobot(int index);
        void changeLink(int index);

    private:
        /*!
         * \brief workspace workspace where robots and obstacles are stored
         */
        WorkSpace  *wSpace;

        QComboBox *robotBox, *linkBox;
        int robot, link;

        void fillRobotBox();
        void fillLinkBox();
    };


    /*!
    * \brief The AttachObjectDialog class allows the user to attach objects to robot links
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

        void set(WorkSpace *workSpace);

    private slots:
        void attach();
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
