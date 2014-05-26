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
#include <problem/problem.h>
#include "dofwidget.h"


namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
     * \brief The ControlWidget class
     */
    class ControlWidget:public QWidget {
		Q_OBJECT
    public:
        ControlWidget(Problem *problem, vector<DOFWidget *> DOFWidgets,
                      bool isRobotControlWidget, QWidget *parent = 0,
                      Qt::WindowFlags f = 0);
        ~ControlWidget();

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string text);

    private slots:
        void sliderChanged(int index);
        void lineEditChanged(int index);
        void updateControls();

    private:
        void setValues(vector <KthReal> coords);

        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);

        vector<DOFWidget *> DOFWids;
        vector<QSlider *> sliders;
        vector<QLineEdit *> lineEdits;
        vector<KthReal> values;
        Problem *prob;
        bool               isRobWidget;
	};
    /** @}   end of Doxygen module "Application" */
}

#endif  //_CONTROLWIDGET_H


