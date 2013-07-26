/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/		

#include <QPushButton>


#include "planners_toolbar.h"


namespace libGUI{ 
  PlannerToolBar::PlannerToolBar(QWidget *Form, string loc, string glob, QObject* receiver, const char* member){
    label = new QLabel(Form);
    label->setObjectName(QString::fromUtf8("lblLocals"));
    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		sizePolicy.setHorizontalStretch(0);
		sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
		label->setSizePolicy(sizePolicy);
    addWidget(label);

		comboBox = new QComboBox(Form);
    comboBox->setObjectName(QString::fromUtf8("local_planners"));
    QStringList labelList= QString(loc.c_str()).split("|");
    for(int i=0; i< labelList.count(); i++)
      comboBox->addItem(tr(labelList.at(i).toUtf8().constData()));
    addWidget(comboBox);
    addSeparator();

    label_2 = new QLabel(Form);
    label_2->setObjectName(QString::fromUtf8("lblPlans"));
    sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
    label_2->setSizePolicy(sizePolicy);
    addWidget(label_2);

		comboBox_2 = new QComboBox(Form);
    comboBox_2->setObjectName(QString::fromUtf8("planners"));
    labelList= QString(glob.c_str()).split("|");
        for(int i=0; i< labelList.count(); i++)
          comboBox_2->addItem(tr(labelList.at(i).toUtf8().constData()));
    addWidget(comboBox_2);
    addSeparator();

    pushButton = new QPushButton(Form);
    pushButton->setObjectName(QString::fromUtf8("addPlannerButton"));
		QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
		sizePolicy1.setHorizontalStretch(0);
		sizePolicy1.setVerticalStretch(0);
		sizePolicy1.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
		pushButton->setSizePolicy(sizePolicy1);
    addWidget(pushButton);
		retranslateUi(Form);

    connect(pushButton, SIGNAL(clicked()), this, SLOT(pushAdd()));
    connect(this, SIGNAL(addPlanner(string,string)), receiver, member);
  }

  void PlannerToolBar::retranslateUi(QWidget *Form)	{
    label->setText(QApplication::translate("Form", "Local Planner", 0, QApplication::UnicodeUTF8));
		label_2->setText(QApplication::translate("Form", "Planner", 0, QApplication::UnicodeUTF8));
		pushButton->setText(QApplication::translate("Form", "Add Planner", 0, QApplication::UnicodeUTF8));
	} // retranslateUi

  void PlannerToolBar::pushAdd(){
    string loc = comboBox->currentText().toUtf8().constData();
    string plan = comboBox_2->currentText().toUtf8().constData();
    //string loc = comboBox->currentText().toLocal8Bit().constData();
    //string plan = comboBox_2->currentText().toLocal8Bit().constData();
    emit addPlanner(loc,plan);
  }
		
}
