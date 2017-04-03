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
 
 
#if !defined(_CONTROLWIDGET_H)
#define _CONTROLWIDGET_H


#include <QtGui>
#ifndef Q_MOC_RUN
#include <kautham/problem/problem.h>
#include "dofwidget.h"
#endif

namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
     * \brief The ControlWidget class is the class that implements the Controls' Widget.
     *It lets the user to change the controls of the robots and the obstacles
     */
    class ControlWidget:public QWidget {
		Q_OBJECT
    public:
        /*!
         * \brief ControlWidget Constructs the widget
         * \param problem problem where the robots/obstacles are stored
         * \param DOFWidgets DOFWidgets related to the robots/obstacles
         * \param isRobotControlWidget wether is a widget for robot controls or not
         * \param parent parent of the widget
         * \param f window flags f the widget
         */
        ControlWidget(Problem *problem, vector<DOFWidget *> DOFWidgets,
                      bool isRobotControlWidget, QWidget *parent = 0,
                      Qt::WindowFlags f = 0);

        /*!
         * \brief ~ControlWidget Destructs the widget
         */
        ~ControlWidget();

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string text);

    private slots:
        /*!
         * \brief sliderChanged updates the control whose slider has changed
         * \param index index of the slider that has changed
         */
        void sliderChanged(int index);

        /*!
         * \brief lineEditChanged updates the control whose lineEdit has changed
         * \param index index of the lineEdit that has changed
         */
        void lineEditChanged(int index);

        /*!
         * \brief updateControls update the controls to the last moved sample or to the
         *initial sample if it is a robot widget or an osbtacle widget repectively
         */
        void updateControls();

    private:
        /*!
         * \brief setValues sets new values for the controls
         * \param coords new values of the controls
         */
        void setValues(vector <KthReal> coords);

        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);


        /*!
         * \brief DOFWids DOFWidgets related to the robots/obstacles
         */
        vector<DOFWidget *> DOFWids;

        /*!
         * \brief sliders sliders for the user to change the controls
         */
        vector<QSlider *> sliders;

        /*!
         * \brief lineEdits line edits for the user to change the controls
         */
        vector<QLineEdit *> lineEdits;

        /*!
         * \brief values values of the controls
         */
        vector<KthReal> values;

        /*!
         * \brief prob problem where the robots/obstacles are stored
         */
        Problem *prob;

        /*!
         * \brief isRobWidget wether is a widget for robot controls or not
         */
        bool isRobWidget;
	};
    /** @}   end of Doxygen module "Application" */
}

#endif  //_CONTROLWIDGET_H


