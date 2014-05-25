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

#include <QtGui>
#include <vector>
#include <string>
#include <problem/problem.h>
#include <util/kthutil/kauthamdefs.h>


using namespace std;


namespace Kautham {

/** \addtogroup Application
 *  @{
 */

	class DOFWidget:public QWidget{
		Q_OBJECT
    signals:
      void sendText(string newContent);

	public:
        DOFWidget(Robot *robot, QWidget *parent = 0, Qt::WindowFlags f = 0);
		~DOFWidget();
        void setValues(vector<KthReal> &values);

	private:
        void writeGUI(string text);
        vector<QLabel*> labels;
        vector<KthReal> currentValues;
        vector<KthReal> lowValues;
        vector<KthReal> highValues;
	};


    /** @}   end of Doxygen module "Application" */
}

#endif  //_DOFWIDGET_H


