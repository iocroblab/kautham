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
 
#if !defined(_DOFWIDGET_H)
#define _DOFWIDGET_H


#include <QtWidgets>
#ifndef Q_MOC_RUN
#include <kautham/problem/problem.h>
#endif

namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
     * \brief The DOFWidget class is the class that implements the DOF's Widget.
     *It shows the DOF values to the user
     */
	class DOFWidget:public QWidget{
        Q_OBJECT
    public:
        /*!
         * \brief DOFWidget Constructs the widget
         * \param robot robot where DOFs are stored
         * \param parent parent of the widget
         * \param f window flags of the widget
         */
        DOFWidget(Robot *robot, QWidget *parent = 0, Qt::WindowFlags f = 0);

        /*!
         * \brief ~DOFWidget Destructs the widget
         */
        ~DOFWidget();

        /*!
         * \brief setValues sets new values for DOFs
         * \param values new values of the DOFs
         */
        void setValues(vector<KthReal> &values);

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string newContent);

    private:
        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);

        /*!
         * \brief labels labels where DOF's values are shown
         */
        vector<QLabel*> labels;

        /*!
         * \brief lowValues lower limits of the DOFs
         */
        vector<KthReal> lowValues;

        /*!
         * \brief highValues higher limits of the DOFs
         */
        vector<KthReal> highValues;
    };
    /** @}   end of Doxygen module "Application" */
}

#endif  //_DOFWIDGET_H
