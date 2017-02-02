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
     

#include "sampleswidget.h"


using namespace std;


namespace Kautham {
    SamplesWidget::SamplesWidget(Problem *problem, QWidget *parent,
                                 Qt::WindowFlags f):QWidget(parent, f) {
        sampleSet = problem->getSampleSet();
        sampler = problem->getSampler();
        prob = problem;
        collection = CURRENT;

        QVBoxLayout *mainLayout = new QVBoxLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QHBoxLayout *hBoxLayout = new QHBoxLayout();
        hBoxLayout->setObjectName(QString::fromUtf8("sampleLayout"));
        mainLayout->addLayout(hBoxLayout);

        QLabel *label = new QLabel("Sample");
        label->setObjectName(QString::fromUtf8("sampleLabel"));
        label->setToolTip("Current sample");
        hBoxLayout->addWidget(label);

        sampleList = new QComboBox();
        sampleList->setObjectName(QString::fromUtf8("sampleList"));
        sampleList->setEditable(false);
        sampleList->setToolTip("Current list of samples");
        connect(sampleList,SIGNAL(currentIndexChanged(int)),this,SLOT(changeSample(int)));
        hBoxLayout->addWidget(sampleList);

        QPushButton *button = new QPushButton("Test collision");
        button->setObjectName(QString::fromUtf8("collisionButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(testCollision()));
        mainLayout->addWidget(button);

        button = new QPushButton("Test distance");
        button->setObjectName(QString::fromUtf8("distanceButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(testDistance()));
        mainLayout->addWidget(button);

        QGridLayout *gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        mainLayout->addLayout(gridLayout);

        QIcon addIcon;
        addIcon.addFile(":/icons/add_16x16.png");
        addIcon.addFile(":/icons/add_22x22.png");

        button = new QPushButton(addIcon,"Add");
        button->setObjectName(QString::fromUtf8("addButton"));
        button->setToolTip("Add current configuration as a sample");
        connect(button,SIGNAL(clicked()),this,SLOT(addSample()));
        gridLayout->addWidget(button,0,0);

        QIcon removeIcon;
        removeIcon.addFile(":/icons/remove_16x16.png");
        removeIcon.addFile(":/icons/remove_22x22.png");

        button = new QPushButton(removeIcon,"Remove");
        button->setObjectName(QString::fromUtf8("removeButton"));
        button->setToolTip("Remove current sample from collection");
        connect(button,SIGNAL(clicked()),this,SLOT(removeSample()));
        gridLayout->addWidget(button,0,1);

        QIcon getIcon;
        getIcon.addFile(":/icons/right_16x16.png");
        getIcon.addFile(":/icons/right_22x22.png");

        button = new QPushButton(getIcon,"Get");
        button->setObjectName(QString::fromUtf8("getButton"));
        button->setToolTip("Get samples");
        connect(button,SIGNAL(clicked()),this,SLOT(getSamples()));
        gridLayout->addWidget(button,1,0);

        QIcon updateIcon;
        updateIcon.addFile(":/icons/reload_16x16.png");
        updateIcon.addFile(":/icons/reload_22x22.png");

        button = new QPushButton(updateIcon,"Update");
        button->setObjectName(QString::fromUtf8("updateButton"));
        button->setToolTip("Update the samples in collection");
        connect(button,SIGNAL(clicked()),this,SLOT(updateSampleList()));
        gridLayout->addWidget(button,1,1);

        QIcon copyIcon;
        copyIcon.addFile(":/icons/copy_16x16.png");
        copyIcon.addFile(":/icons/copy_22x22.png");

        button = new QPushButton(copyIcon,"Copy");
        button->setObjectName(QString::fromUtf8("copyButton"));
        button->setToolTip("Copy the two first samples in a new collection");
        connect(button,SIGNAL(clicked()),this,SLOT(copySampleList()));
        gridLayout->addWidget(button,2,0);

        QIcon clearIcon;
        clearIcon.addFile(":/icons/trashcan_16x16.png");
        clearIcon.addFile(":/icons/trashcan_22x22.png");

        button = new QPushButton(clearIcon,"Clear");
        button->setObjectName(QString::fromUtf8("clearButton"));
        button->setToolTip("Remove all samples in collection");
        connect(button,SIGNAL(clicked()),this,SLOT(clearSampleList()));
        gridLayout->addWidget(button,2,1);

        QGroupBox *groupBox = new QGroupBox("Add to collection");
        groupBox->setObjectName(QString::fromUtf8("addGroupBox"));
        mainLayout->addWidget(groupBox);

        QVBoxLayout *vBoxLayout = new QVBoxLayout();
        vBoxLayout->setObjectName(QString::fromUtf8("addLayout"));
        vBoxLayout->setContentsMargins(0,9,0,0);
        groupBox->setLayout(vBoxLayout);

        QComboBox *comboBox = new QComboBox();
        comboBox->setObjectName(QString::fromUtf8("addComboBox"));
        comboBox->setEditable(false);
        comboBox->insertItem(CURRENT,"Current");
        comboBox->insertItem(NEW,"New");
        comboBox->setCurrentIndex(CURRENT);
        connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCollection(int)));
        vBoxLayout->addWidget(comboBox);

        groupBox = new QGroupBox("Engine");
        groupBox->setObjectName(QString::fromUtf8("engineGroupBox"));
        mainLayout->addWidget(groupBox);

        vBoxLayout = new QVBoxLayout();
        vBoxLayout->setObjectName(QString::fromUtf8("engineLayout"));
        vBoxLayout->setContentsMargins(0,9,0,0);
        groupBox->setLayout(vBoxLayout);

        comboBox = new QComboBox();
        comboBox->setObjectName(QString::fromUtf8("engineComboBox"));
        comboBox->setEditable(false);
        comboBox->insertItem(SDK,"SDK");
        comboBox->insertItem(HALTON,"Halton");
        comboBox->insertItem(RANDOM,"Random");
        comboBox->insertItem(GAUSSIAN,"Gaussian");
        if (typeid(*sampler) == typeid(SDKSampler)) {
            comboBox->setCurrentIndex(SDK);
        } else if (typeid(*sampler) == typeid(HaltonSampler)) {
            comboBox->setCurrentIndex(HALTON);
        } else if (typeid(*sampler) == typeid(RandomSampler)) {
            comboBox->setCurrentIndex(RANDOM);
        } else if (typeid(*sampler) == typeid(GaussianSampler)) {
            comboBox->setCurrentIndex(GAUSSIAN);
        }
        connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(changeEngine(int)));
        vBoxLayout->addWidget(comboBox);

        hBoxLayout = new QHBoxLayout();
        hBoxLayout->setObjectName(QString::fromUtf8("amountLayout"));
        mainLayout->addLayout(hBoxLayout);

        label = new QLabel("Amount");
        label->setObjectName(QString::fromUtf8("amountLabel"));
        label->setToolTip("Number of samples to calculate");
        hBoxLayout->addWidget(label);

        sampleAmount = new QLineEdit();
        sampleAmount->setObjectName(QString::fromUtf8("amountLineEdit"));
        sampleAmount->setToolTip("Number of samples to calculate");
        hBoxLayout->addWidget(sampleAmount);

        updateSampleList();
    }


    void SamplesWidget::changeSample(int index) {
        if (index >= 0 && index < int(sampleSet->getSize())) {
            prob->wSpace()->moveRobotsTo(sampleSet->getSampleAt(index));
        }
    }


    void SamplesWidget::testCollision() {
        stringstream sstr;
        if (sampleList->count() > 0) {
            Sample* sample = sampleSet->getSampleAt((sampleList->currentText()).toInt());
            sstr << "The sample No: " << sampleList->currentText().toUtf8().constData()
                 << " is: ";
            string message;
            if (prob->wSpace()->collisionCheck(sample,&message)) {
                sstr << "in COLLISION" << endl;
                sstr << message;
            } else {
                sstr << "FREE";
            }
            writeGUI(sstr.str());
        } else {
            writeGUI("First create a sample");
        }
    }


    void SamplesWidget::testDistance(){
        if (sampleList->count() > 0){
            Sample* sample = sampleSet->getSampleAt((sampleList->currentText()).toInt());
            vector<KthReal>* values = prob->wSpace()->distanceCheck(sample);
            if (values->size() > 0) {
                stringstream sstr;
                sstr.precision(10);
                sstr << "For sample No: " << sampleList->currentText().toUtf8().constData()
                     << " the distance check is: " << values->at(0);
                for (uint i = 1; i < values->size(); ++i) {
                    sstr<< ", " << values->at(i);
                }
                writeGUI(sstr.str());
            } else {
                writeGUI("There are no obstacles to test distance with.");
            }
        } else {
            writeGUI("Please, first create a sample.");
        }
    }


    void SamplesWidget::addSample(){
        int dim = prob->wSpace()->getNumRobControls();
        Sample* sample = new Sample(dim);
        sample->setCoords(prob->getCurrentRobControls());
        string message;
        if (!prob->wSpace()->collisionCheck(sample,&message)) {
            sampleSet->add(sample);
            updateSampleList();
        } else {
            writeGUI("Samples not added - COLLISION configuration!\n"+message);
        }
    }


    void SamplesWidget::removeSample(){
        if (!sampleSet->removeSampleAt((sampleList->currentText()).toInt())) {
            writeGUI("An unexpected error ocurred. Please try again");
        }
        updateSampleList();
    }


    void SamplesWidget::getSamples() {
        bool ok;
        unsigned numSamples = sampleAmount->text().toUInt(&ok,10);

        if (ok) {
            Sample *sample;
            int numFree = 0;

            if (collection == NEW && sampleSet->getSize() > 1) {
                copySampleList();
            }

            for (uint i = 0; i < numSamples; ++i) {
                sample = sampler->nextSample();
                sample->setFree(!prob->wSpace()->collisionCheck(sample));
                if (sample->isFree()) {
                    ++numFree;
                    sampleSet->add(sample);
                } else {
                    delete sample;
                }
            }

            QString text = sampleAmount->text() + " samples were generated, "
                    + QString::number(numFree) + " samples are free.";
            writeGUI(text.toUtf8().constData());
            sampleAmount->setText("");
            updateSampleList();
        } else {
            writeGUI("Please, enter a valid amount of samples.");
        }
    }


    void SamplesWidget::updateSampleList() {
        if (sampleList->count() < int(sampleSet->getSize())) {
            for (uint i = sampleList->count(); i < sampleSet->getSize(); ++i) {
                sampleList->addItem(QString::number(i));
            }
        } else {
            for (uint i = sampleList->count()+1; i > (sampleSet->getSize()+1); --i) {
                sampleList->removeItem(i);
            }
        }
    }


    void SamplesWidget::copySampleList() {
        if (sampleSet->getSize() > 0) {
            int dim = prob->wSpace()->getNumRobControls();
            Sample *initSample = new Sample(dim);
            Sample *goalSample = new Sample(dim);
            initSample->setCoords(sampleSet->getSampleAt(0)->getCoords());
            goalSample->setCoords(sampleSet->getSampleAt(1)->getCoords());
            sampleSet->clear();
            sampleSet->add(initSample);
            sampleSet->add(goalSample);

        } else {
            sampleSet->clear();
        }
        updateSampleList();
    }


    void SamplesWidget::clearSampleList() {
        sampleSet->clear();

        updateSampleList();
    }


    void SamplesWidget::changeEngine(int index) {
        switch (index) {
        case SDK:
            if (typeid(*sampler) != typeid(SDKSampler)) {
                delete sampler;
                sampler = new SDKSampler(prob->wSpace()->getNumRobControls(),2);
                prob->setSampler(sampler);
            }
            break;
        case HALTON:
            if (typeid(*sampler) != typeid(HaltonSampler)) {
                delete sampler;
                sampler = new HaltonSampler(prob->wSpace()->getNumRobControls());
                prob->setSampler(sampler);
            }
            break;
        case RANDOM:
            if (typeid(*sampler) != typeid(RandomSampler)) {
                delete sampler;
                sampler = new RandomSampler(prob->wSpace()->getNumRobControls());
                prob->setSampler(sampler);
            }
            break;
        case GAUSSIAN:
            if (typeid(*sampler) != typeid(GaussianSampler)) {
                delete sampler;
                sampler = new GaussianSampler(prob->wSpace()->getNumRobControls(),
                                               0.1,prob->wSpace());
                prob->setSampler(sampler);
            }
            break;

        default:
            break;
        }
    }


    void SamplesWidget::changeCollection(int index) {
        switch (index) {
        case CURRENT:
            collection = CURRENT;
            break;
        case NEW:
            collection = NEW;
            break;
        default:
            break;
        }
    }


    void SamplesWidget::writeGUI(string text){
        emit sendText(text);
    }
}
