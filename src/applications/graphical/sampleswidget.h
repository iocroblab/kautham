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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */
 
#if !defined(UI_SAMPLES_H)
#define UI_SAMPLES_H


//#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QLineEdit>
#include <sampling/sampling.h>
#include <problem/problem.h>

using namespace std;

namespace Kautham{
/** \addtogroup libGUI
 *  @{
 */

  class SamplesWidget: public QWidget {
    Q_OBJECT

    signals:
      void sendText(string newContent);

    private slots:
      void collisionCheck();
      void distanceCheck();
      void addCurrent();
      void removeCurrent();
      void removeAll();
      void removeAllEx2();
      void sampling();
      void updateSampleList();
      void showSample(int index);
	  void changeEngine();
    public:
      SamplesWidget(SampleSet* samples, Sampler* sampler, Problem* prob);
      
    private:
      void writeGUI(string text);
      QGridLayout *gridLayout;
      QVBoxLayout *vboxLayout;
      QVBoxLayout *vboxLayout1;
      QLabel *label,*label_3;
      QComboBox *cboSampleList;
      QHBoxLayout *hboxLayout, *hboxLayout1;
      QPushButton *btnCollision;
      QPushButton *btnDistance;      
      QGroupBox *groupBox;
      QGridLayout *gridLayout1;
      QVBoxLayout *vboxLayout2;
      QRadioButton *rbtnAdd;
      QRadioButton *rbtnNew;
      QGroupBox *groupBox_2;
      QGridLayout *gridLayout2;
      QVBoxLayout *vboxLayout3;
      QRadioButton *rbtnRandom;
      QRadioButton *rbtnSDK;
      QRadioButton *rbtnHalton;
	  QRadioButton *rbtnGaussian;
	  QRadioButton *rbtnGaussianLike;
      QPushButton *btnSampling;
      QLineEdit *txtAmount;
      QPushButton *btnAddCurrent;
      QPushButton *btnRemoveCurrent;
      
      Problem*    _ptProblem;
      SampleSet*  _samples;
      Sampler*    _sampler;


  };

  /** @}   end of Doxygen module "libGUI" */
}


#endif // UI_SAMPLES_H

