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


#include <QtWidgets>

#ifndef Q_MOC_RUN
#include <kautham/problem/problem.h>
#endif

namespace Kautham{
    /** \addtogroup Application
    *  @{
    */

    /*!
     * \brief The COLLECTION enum is the sample's collection which samples will be add to
     */
    enum COLLECTION {
        CURRENT,
        NEW
    };

    /*!
     * \brief The ENGINE enum is the sampler type used to get new samples
     */
    enum ENGINE {
        SDK,
        HALTON,
        RANDOM,
        GAUSSIAN,
        GAUSSIANLIKE
    };

    /*!
     * \brief The SamplesWidget class is the class that implements the Samples' Widget
     */
    class SamplesWidget: public QWidget {
        Q_OBJECT
    public:
        /*!
         * \brief SamplesWidget Constructs the widget
         * \param problem problem to construct the widget of
         * \param parent parent widget
         * \param f window's flags
         */
        SamplesWidget(Problem *problem, QWidget *parent = 0, Qt::WindowFlags f = 0);

    signals:
        /*!
         * \brief sendText sends a message
         * \param text message to send
         */
        void sendText(string text);

    private slots:
        /*!
         * \brief changeSample changes current sample and moves robots to it
         * \param index index of current sample
         */
        void changeSample(int index);

        /*!
         * \brief testCollision checks whether the current sample is free of collision
         */
        void testCollision();

        /*!
         * \brief testDistance computes the distance between robots and obstacles
         */
        void testDistance();

        /*!
         * \brief addSample adds the current configuration as a sample
         */
        void addSample();

        /*!
         * \brief removeSample removes current sample from the sample's list
         */
        void removeSample();

        /*!
         * \brief getSamples gets the amount of samples specified in the widget,
         * using the selected sampler and adds them to the selected collection
         */
        void getSamples();

        /*!
         * \brief updateSampleList updates the sample's list
         */
        void updateSampleList();

        /*!
         * \brief copySampleList copies the two first samples in the sample's list in a new collection
         */
        void copySampleList();

        /*!
         * \brief clearSampleList clears the sample's list
         */
        void clearSampleList();

        /*!
         * \brief changeCollection changes the collection where new samples will be added
         * \param index
         */
        void changeCollection(int index);

        /*!
         * \brief changeEngine changes the sampler's type used to get new samples
         * \param index
         */
        void changeEngine(int index);

    private:
        /*!
         * \brief writeGUI writes a message in the GUI
         * \param text message to write
         */
        void writeGUI(string text);

        /*!
         * \brief prob problem to construct the widget of
         */
        Problem *prob;

        /*!
         * \brief sampleSet set of samples of the problem
         */
        SampleSet *sampleSet;

        /*!
         * \brief sampler sampler of the problem
         */
        Sampler *sampler;

        /*!
         * \brief sampleList shows to the user the list of samples
         */
        QComboBox *sampleList;

        /*!
         * \brief sampleAmount line edit where the user can specify the number of samples to get
         */
        QLineEdit  *sampleAmount;

        /*!
         * \brief collection the sample's collection which samples will be add to
         */
        COLLECTION collection;
    };
    /** @}   end of Doxygen module "Application" */
}


#endif // _SAMPLESWIDGET_H

