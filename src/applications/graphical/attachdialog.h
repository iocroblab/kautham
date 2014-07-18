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
#include <problem/problem.h>


namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
    * \brief The AttachDialog class allows the user to attach objects to robot links
    */
    class AttachDialog : public QDialog {
        Q_OBJECT
    public:
        /*!
         * \brief AttachDialog Constructs the dialog
         * \param parent Parent of the dialog
         * \param f Window flags
         */
        AttachDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

        void set(WorkSpace *workSpace);

        /*!
         * \brief  run Executes the dialog and lets the user to attach objects to robot links
         */
        bool run();

    private slots:
        void changeObstacle(int index);
        void changeRobot(int index);
        void changeLink(int index);

    private:
        WorkSpace *wSpace;
        QComboBox *obstacleBox, *robotBox, *linkBox;
        int obs, rob, link;

        void fillObstacleBox();
        void fillRobotBox();
        void fillLinkBox();
    };
    /** @}   end of Doxygen module "Application" */
}
