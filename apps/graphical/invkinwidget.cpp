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


#include "invkinwidget.h"
#include <kautham/problem/robot.h>


namespace Kautham{


  InvKinWidget::InvKinWidget(InverseKinematic* ivKin):KauthamWidget(ivKin){
	  _invKin = ivKin;
    btnSolve = new QPushButton(this);
    btnSolve->setObjectName(QString::fromUtf8("solveButton"));

    vboxLayout->addWidget(btnSolve);
 
    btnSolve->setText(QApplication::translate("Form", "Get solution", 0));

    if(_invKin != NULL)
      connect(btnSolve, SIGNAL( clicked() ), this, SLOT( getSolution() ) );
    
  }

  //! This method calls the solve method of the associated Inverse Kinematic
  //! object.  Be sure you have a target set before call it. In order to
  //! have a generic method, you may set the target using the tableChanged and
  //! the setParametersFromString method of Inverser Kinematic object.
  void InvKinWidget::getSolution(){
    if(_invKin->solve())
    {
      //_invKin->getRobot().Kinematics(_invKin->getRobConf());
      RobConf rc = _invKin->getRobConf();
      _invKin->getRobot().Kinematics(rc);

       writeGUI("Inverse Kinematic: Solution found");

       uint n=rc.second.getDim();
       stringstream sstr;
       sstr<< "RnConf = ";
       for(uint i=0; i<n;i++)
       {
           sstr<<  rc.second.getCoordinate(i) << ", ";
       }
       writeGUI(sstr.str());
  }
    else
      writeGUI("Inverse Kinematic has not a solution.");
  }

}
