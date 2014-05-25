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
 

#if !defined(_SAMPLESWIDGET_H)
#define _SAMPLESWIDGET_H


#include <QtGui>
#include <problem/problem.h>


namespace Kautham{
/** \addtogroup Application
 *  @{
 */
    enum COLLECTION {
        CURRENT,
        NEW
    };


    enum ENGINE {
        SDK,
        HALTON,
        RANDOM,
        GAUSSIAN,
        GAUSSIANLIKE
    };


    class SamplesWidget: public QWidget {
        Q_OBJECT
    signals:
        void sendText(string text);

    private slots:
        void changeSample(int index);
        void testCollision();
        void testDistance();
        void addSample();
        void removeSample();
        void getSamples();
        void updateSampleList();
        void copySampleList();
        void clearSampleList();
        void changeCollection(int index);
        void changeEngine(int index);

    public:
        SamplesWidget(SampleSet* sampleSet, Sampler* sampler, Problem* problem,
                      QWidget *parent = 0, Qt::WindowFlags f = 0);

    private:
        void writeGUI(string text);
        Problem *_problem;
        SampleSet *_sampleSet;
        Sampler *_sampler;
        QComboBox *_sampleList;
        QLineEdit  *_sampleAmount;
        COLLECTION collection;
    };

/** @}   end of Doxygen module "Application" */
}


#endif // _SAMPLESWIDGET_H

