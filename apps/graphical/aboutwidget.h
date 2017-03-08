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

#ifndef ABOUTWIDGET_H
#define ABOUTWIDGET_H

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

class AboutWidget:public QDialog {
  Q_OBJECT
public:
  AboutWidget(QWidget* parent):QDialog(parent){setupUi(this);}
    QTextBrowser *textBrowser;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(671, 502);
        Form->setMaximumSize(QSize(701, 552));
        Form->setMinimumSize(QSize(701,552));
        QIcon icon(QString::fromUtf8(":/icons/logo.svg"));
        Form->setWindowIcon(icon);
        textBrowser = new QTextBrowser(Form);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(0, 0, 701, 552));
        textBrowser->setOpenLinks(false);

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "The Kautham Project", 0, QApplication::UnicodeUTF8));

        std::ostringstream otext;
        otext << "<!DOCTYPE html><html><body><h2>The Kautham Project - "<<std::endl<<"Version "<< MAJOR_VERSION <<"."<< MINOR_VERSION <<"."<< PATCH_VERSION <<"</h2><br>"<<std::endl;

        std::string text2 = "<b>The Kautham Project</b> is a software tool developped at the <b>Service and Industrial Robotics group</b>\
                (SIR, http://robotics.upc.edu)  of the <b>Institute of Industrial and \
                Control Engineering</b> (IOC, http://ioc.upc.edu) of the <b>Universitat Politècnica de Catalunya</b> (UPC, http://www.upc.edu), \
                for teaching and research in robot motion planning.\
                <br><br>The tool allows to cope with problems with one or more robots, being a generic\
                robot defined as a kinematic tree with a mobile base, i.e. the tool can plan\
                and simulate from simple two degrees of freedom free-flying robots to multi-robot\
                scenarios with mobile manipulators equipped with anthropomorphic hands.\
                <br><br>The main core of planners is provided by the Open Motion Planning Library (OMPL, http://ompl.kavrakilab.org).\
                Different basic planners can be flexibly used and parameterized, allowing\
                students to gain insight into the different planning algorithms. Among the\
                advanced features the tool allows to easily define the coupling between degrees of\
                freedom, the dynamic simulation and the integration with task planers. It is\
                principally being used in the research of motion planning strategies for dexterous dual arm\
                robotic systems. \
                <br><br><b>Webpage</b> = http://sir.upc.edu/kautham\
                <br><b>Contact</b> = Prof. Jan Rosell (jan.rosell@upc.edu)</body></html>\
                <br><br><img src=\":/icons/logo_ioc.png\" width=\"605\" \">";

        std::string text = otext.str()+text2;

        textBrowser->setHtml(QApplication::translate("Form", text.c_str(), 0, QApplication::UnicodeUTF8));

    } // retranslateUi

};

/** @}   end of Doxygen module "Application" */
}

#endif // ABOUTWIDGET_H
