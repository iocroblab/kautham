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
 
#include "kauthamwidget.h"

namespace Kautham{
  KauthamWidget::KauthamWidget(KauthamObject* kObj){
      _kauthObject= kObj;
      gridLayout = new QGridLayout(this);
      gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
      vboxLayout = new QVBoxLayout();
      vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
      table = new QTableWidget(this);
      table->setObjectName(QString::fromUtf8("table"));
      QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      sizePolicy.setHorizontalStretch(0);
      sizePolicy.setVerticalStretch(0);
      sizePolicy.setHeightForWidth(table->sizePolicy().hasHeightForWidth());
      table->setSizePolicy(sizePolicy);
      table->setMinimumSize(QSize(200, 200));
      table->setMaximumSize(QSize(250, 16777215));

      vboxLayout->addWidget(table);
      gridLayout->addLayout(vboxLayout, 0, 0, 1, 1);

      if (table->columnCount() < 2)
          table->setColumnCount(2);

      QTableWidgetItem *__colItem = new QTableWidgetItem();
      __colItem->setText(QApplication::translate("Form", "Property", 0));
      table->setHorizontalHeaderItem(0, __colItem);
      __colItem = new QTableWidgetItem();
      __colItem->setText(QApplication::translate("Form", "Value", 0));
      table->setHorizontalHeaderItem(1, __colItem);
      table->verticalHeader()->hide();

      if (table->rowCount() < 1)
          table->setRowCount(0);

      if(_kauthObject != NULL ){
          setTable(_kauthObject->getParametersAsString());
      }
  }


  bool KauthamWidget::setTable(string s){
      if (s.size()!=0) {
          disconnect(table, SIGNAL(cellChanged(int, int)), this, SLOT(tableChanged(int, int)));
          table->setSortingEnabled(true);
          QStringList cont = QString(s.c_str()).split("|");
          QStringList h,v;
          QStringList::const_iterator iterator;
          QTableWidgetItem *item;
          for (iterator = cont.constBegin(); iterator != cont.constEnd();
               ++iterator){
              h << (*iterator).toUtf8().constData();
              ++iterator;
              v << (*iterator).toUtf8().constData();
          }
          table->setRowCount(v.size());
          int i=0;
          for(iterator = v.constBegin(); iterator != v.constEnd(); ++iterator){
              item = new QTableWidgetItem((*iterator).toUtf8().constData());
              table->setItem(i,1,item);
              item= new QTableWidgetItem(h.at(i));
              table->setItem(i,0,item);
              //table->setVerticalHeaderItem(i,item);
              i++;
          }
          //table->sortItems(0); Bug found when having items with smallcaps and bigcaps
          connect(table, SIGNAL(cellChanged(int, int)), this, SLOT(tableChanged(int, int)));
          return true;
      }
      connect(table, SIGNAL(cellChanged(int, int)), this, SLOT(tableChanged(int, int)));
      return false;
	}


  void KauthamWidget::tableChanged(int row, int col){
      (void) col;//unused
      QString sal;
      QTableWidgetItem *item;
      if(_kauthObject != NULL ){
          item = table->item(row,0);
          if(item!=NULL){
              sal.append( item->text() + "|");
              item = table->item(row,1);
              sal.append(item->text());
              string strSal(sal.toUtf8().constData());
              try{
                  _kauthObject->setParametersFromString(strSal);
                  setTable(_kauthObject->getParametersAsString());
              }catch(...){
                  cout << "An error ocurred." << endl;
              }
          }
      }
  }


  void KauthamWidget::writeGUI(string text){
      emit sendText(text);
  }
}
