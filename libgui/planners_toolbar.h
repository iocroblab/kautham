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


#if !defined(PLANNERS_TOOLBAR_H)
#define PLANNERS_TOOLBAR_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QToolBar>

using namespace std;


namespace Kautham{
/** \addtogroup libGUI
 *  @{
 */

	class PlannerToolBar: public QToolBar{
	Q_OBJECT
  signals:
    void            addPlanner(string loc, string glob);

  private slots:
    void            pushAdd();
  public:
    PlannerToolBar(QWidget *Form, string loc, string glob, QObject* receiver, const char* member);
	  
  private:
    PlannerToolBar();
    void            retranslateUi(QWidget *Form);
    QGridLayout*    gridLayout;
    QHBoxLayout*    horizontalLayout;
    QLabel*         label;
    QLabel*         label_2;
    QComboBox*      comboBox;
    QComboBox*      comboBox_2;
    QPushButton*    pushButton;

	};

    /** @}   end of Doxygen module "libGUI" */
}

#endif // PLANNERS_TOOLBAR_H
