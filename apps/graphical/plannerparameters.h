/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Alexander Perez, Jan Rosell */

#ifndef PLANNERPARAMETERSWIDGET_H
#define PLANNERPARAMETERSWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QDialog>

//The following config header file contains the version number of kautham and is automatically generated from the
//CMakeList.txt file using the configure file option
#include <kautham/kauthamConfig.h>

#include <string>
#include <sstream>      // std::ostringstream

namespace Kautham {

/** \defgroup Application
 *  \brief contains the GUI, the console application and the ROS node
 *
 *  \todo Add detailed description of Application
 *
 *  @{
 */

class PlannerParametersWidget:public QDialog {
  Q_OBJECT
public:
  PlannerParametersWidget(QWidget* parent):QDialog(parent){setupUi(this);}
    QTextBrowser *textBrowser;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(650, 400);
        QIcon icon(QString::fromUtf8(":/icons/logo.svg"));
        Form->setWindowIcon(icon);
        //Form->setFocusPolicy( Qt::ClickFocus );

        textBrowser = new QTextBrowser(Form);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(0, 0, 650, 400));
        textBrowser->setOpenLinks(false);

        Form->setWindowTitle(QApplication::translate("Form", "Planners Parameters", 0));
        std::string text = "<b>General Planners Parameters</b>\
                <br><br><em>_Cspace Drawn:</em> If multi-robot problem, draws the cspace of the robot indicated (counting from 0).\
                <br>Default value is 0.\
                <br><br><em>_Incremental (0/1):</em> If set continues the growing of the planner data structure.\
                <br> Default value is 0.\
                <br><br><em>_Path Drawn (0/1):</em> If set draws the planner data in the workspace (it has effect after computing the solution).\
                <br> Default value is 1.\
                <br><br><em>_Max Planning Time: </em>Maximum allowed time in seconds to find a solution.\
                <br> Default value is 10.\
                <br><br><em>_Simplify Solution: </em> (0) Raw solution, (1) Smoothed solution using Bsplines, (2) Simplified solution.\
                <br> Default value is 2.\
                <br><br><em>_Speed Factor:</em> Speeds the animation of the solution (integer value).\
                <br> Default value is 1.\
                <br><br>\
                <b>PRM</b>\
                <br><br><em>BounceDistanceThreshold:</em> Maximum advance distance in the random bounce walk during the expand phase.\
                <br><br><em>BounceSteps:</em> Maximum number of steps of the random bounce walk during the expand phase.\
                <br><br><em>DistanceThreshold:</em> Maximum distance to search for neighbors nodes to connect the sampled new configuration.\
                <br><br><em>MaxNearestNeighbors:</em> Maximum number of neighbors to try connections with the sampled new configuration.\
                <br><br><em>MinExpandTime:</em> Time slot devoted to the expand phase.\
                <br><br><em>MinGrowTime:</em>  Time slot devoted to the growing phase.\
                <br><br><em>Sampler 0(r), 1(h) 2(sdk) 3(g):</em> Sampling source to be chosen among random, halton, sdk - deterministic-, gaussian\
                <br><br>\
                <b>RRT</b>\
                <br><br><em>Goal Bias: </em> The fraction of time the goal is picked as the state to expand towards.\
                <br><br><em>Range:</em> The maximum length of a motion to be added to a tree.\
                <br><br>\
                <b>RRT Connect</b>\
                <br><br><em>Range:</em> The maximum length of a motion to be added to a tree.\
                <br><br>\
                <b>RRT Star</b>\
                <br><br>\
                <b>SBL</b>\
                <br><br>\
                <b>EST</b>\
                           ";




        textBrowser->setHtml(QApplication::translate("Form", text.c_str(), 0));
        QMetaObject::connectSlotsByName(Form);
    } // setupUi



};

/** @}   end of Doxygen module "Application" */
}

#endif // PLANNERPARAMETERSWIDGET_H
