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

/* Author: Alexander Perez, Jan Rosell */

#ifndef PLANNERPARAMETERSWIDGET_H
#define PLANNERPARAMETERSWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QTextBrowser>
#include <QtGui/QDialog>

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

        Form->setWindowTitle(QApplication::translate("Form", "Planners Parameters", 0, QApplication::UnicodeUTF8));
        std::string text = "<b>General Planners Parameters</b>\
                <br><br><em>Cspace Drawn:</em>If multi-robot problem, draws the cspace of the robot indicated (counting from 0).\
                <br><em>Incremental (0/1):</em> If set continues the growing of the planner data structure.\
                <br><em>Path Drawn (0/1):</em> If set draws the planner data in the workspace (it has effect after computing the solution)\
                <br><em>Max Planning Time: </em>Maximum allowed time in seconds to find a solution.\
                <br><em>Simplify Solution: </em> (0) Raw solution, (1) Smoothed solution using Bsplines, (2) Simplified solution\
                <br><em>Speed Factor:</em> Speeds the animation of the solution (integer value)\
                <br><br>\
                <b>PRM</b>\
                <br><br><em>BounceDistanceThreshold:</em> Maximum advance distance in the random bounce walk during the expand phase.\
                <br><em>BounceSteps:</em> Maximum number of steps of the random bounce walk during the expand phase.\
                <br><em>DistanceThreshold:</em> Maximum distance to search for neighbors nodes to connect the sampled new configuration.\
                <br><em>MaxNearestNeighbors:</em> Maximum number of neighbors to try connections with the sampled new configuration.\
                <br><em>MinExpandTime:</em> Time slot devoted to the expand phase.\
                <br><em>MinGrowTime:</em>  Time slot devoted to the growing phase.\
                <br><em>Sampler 0(r), 1(h) 2(sdk) 3(g) 4(gl):</em> Sampling source to be chosen among random, halton, sdk - deterministic-, gaussian, gaussian-like (deterministic based on sdk)\
                <br><br>\
                <b>RRT</b>\
                <br><br><em>Goal Bias: </em> The fraction of time the goal is picked as the state to expand towards.\
                <br><em>Range:</em> The maximum length of a motion to be added to a tree.\
                <br><br>\
                <b>RRT Connect</b>\
                <br><br><em>Range:</em> The maximum length of a motion to be added to a tree.\
                <br><br>\
          <br><br><em> To be continued...</em>\
                <b>RRT Star</b>\
                <br><br><em>DelayCC (0/1): </em>\
                <br><em>Goal Bias:</em> \
                <br><em>K-Neigh Factor:</em> \
                <br><em>KD:</em> \
                <br><em>KI:</em> \
                <br><em>KP:</em> \
                <br><em>Node Rejection:</em> \
                <br><em>Optimize (0)none/dist(1)/clear(2)/PMD(3):</em> \
                <br><em>Range:</em> The maximum length of a motion to be added to a tree.\
                <br><br>\
                <b>SBL</b>\
                <br><br>\
                <b>EST</b>\
                           ";




        textBrowser->setHtml(QApplication::translate("Form", text.c_str(), 0, QApplication::UnicodeUTF8));
        QMetaObject::connectSlotsByName(Form);
    } // setupUi



};

/** @}   end of Doxygen module "Application" */
}

#endif // PLANNERPARAMETERSWIDGET_H
