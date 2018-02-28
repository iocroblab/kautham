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
 
#include <QtWidgets>


#if !defined(_KAUTHAMWIDGET_H)
#define _KAUTHAMWIDGET_H


#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/util/kthutil/kauthamobject.h>


using namespace std;

namespace Kautham{

/** \addtogroup  Application
 *  @{
 */

	class KauthamWidget: public QWidget{
	  Q_OBJECT
	  signals:
		  void            sendText(string newContent);
	  private slots:
          void            tableChanged(int row, int col);
	  public:
		  KauthamWidget(KauthamObject* kObj);

      /*!	This function fills the properties table in the widget window.
      *   s string has properties
      *   and values in secuential mode, separated by |
      */
        virtual  bool            setTable(string s);
      inline void     hideTable(){table->setVisible(false);}
    protected:
			void            writeGUI(string text);
            QTableWidget*   table;

	  private:
		  
          QGridLayout*    gridLayout;
		  KauthamObject*  _kauthObject;

    protected:
    	QVBoxLayout*    vboxLayout;
		};

/** @}   end of Doxygen module "Application" */
 }



#endif	//_KAUTHAMWIDGET_H

