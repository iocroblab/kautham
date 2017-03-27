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


#include "plannerwidget.h"
#include "gui.h"
#include <kautham/sampling/se3conf.h>
#include <kautham/sampling/robconf.h>

//#if defined(KAUTHAM_USE_IOC)
//#include <kautham/planner/ioc/iocplanner.h>
//#endif


namespace Kautham {
    PlannerWidget::PlannerWidget(Planner* plan, SampleSet* samp, bool camera):KauthamWidget(plan){
        _samples = samp;
        _planner = plan;
        _stepSim = 0;
        _ismoving = false;

        QGroupBox *groupBox = new QGroupBox(this);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));

        QGridLayout *gridLayoutGB = new QGridLayout(groupBox);
        gridLayoutGB->setObjectName(QString::fromUtf8("gridLayout"));

        QVBoxLayout *verticalLayoutGB = new QVBoxLayout();
        verticalLayoutGB->setObjectName(QString::fromUtf8("verticalLayout"));

        QHBoxLayout *horizontalLayoutGB = new QHBoxLayout();
        horizontalLayoutGB->setObjectName(QString::fromUtf8("horizontalLayout"));

        QLabel *labelGB = new QLabel(groupBox);
        labelGB->setObjectName(QString::fromUtf8("label"));

        horizontalLayoutGB->addWidget(labelGB);

        localFromBox = new QSpinBox(groupBox);
        localFromBox->setObjectName(QString::fromUtf8("localFromBox"));

        horizontalLayoutGB->addWidget(localFromBox);

        QLabel *label_2GB = new QLabel(groupBox);
        label_2GB->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayoutGB->addWidget(label_2GB);

        localToBox = new QSpinBox(groupBox);
        localToBox->setObjectName(QString::fromUtf8("localToBox"));

        horizontalLayoutGB->addWidget(localToBox);


        verticalLayoutGB->addLayout(horizontalLayoutGB);

        QHBoxLayout *horizontalLayout_2GB = new QHBoxLayout();
        horizontalLayout_2GB->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _cmbTry = new QPushButton(groupBox);
        _cmbTry->setObjectName(QString::fromUtf8("_cmbTry"));

        horizontalLayout_2GB->addWidget(_cmbTry);

        connectLabel = new QLabel(groupBox);
        connectLabel->setObjectName(QString::fromUtf8("label_3"));
        connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/tryconnect.xpm")));

        horizontalLayout_2GB->addWidget(connectLabel);


        verticalLayoutGB->addLayout(horizontalLayout_2GB);


        gridLayoutGB->addLayout(verticalLayoutGB, 0, 0, 1, 1);


        vboxLayout->addWidget(groupBox);

        tmpLabel = new QLabel(this);
        tmpLabel->setText("Init configuration is the sample:");
        globalFromBox = new QSpinBox(this);
        globalFromBox->setEnabled(_samples->getNumStarts() == 1);

        hboxLayout = new QHBoxLayout();
        hboxLayout->addWidget(tmpLabel);
        hboxLayout->addWidget(globalFromBox);
        vboxLayout->addLayout(hboxLayout);

        tmpLabel = new QLabel(this);
        tmpLabel->setText("Goal configuration is the sample:");
        globalToBox = new QSpinBox(this);
        globalToBox->setEnabled(_samples->getNumGoals() == 1);

        hboxLayout = new QHBoxLayout();
        hboxLayout->addWidget(tmpLabel);
        hboxLayout->addWidget(globalToBox);
        vboxLayout->addLayout(hboxLayout);

        if (camera) {
            chkCamera = new QCheckBox("Move the camera.");
            chkCamera->setChecked(false);
            vboxLayout->addWidget(chkCamera);
        } else {
            chkCamera = NULL;
        }

        hboxLayout = new QHBoxLayout();
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));

        btnGetPath = new QPushButton(this);
        btnGetPath->setObjectName(QString::fromUtf8("getPathButton"));

        hboxLayout->addWidget(btnGetPath);

        btnSaveData = new QPushButton(this);
        btnSaveData->setObjectName(QString::fromUtf8("saveButton"));
        btnSaveData->setDisabled(true);
        hboxLayout->addWidget(btnSaveData);

        vboxLayout->addLayout(hboxLayout);

        hboxLayout2 = new QHBoxLayout();
        hboxLayout2->setObjectName(QString::fromUtf8("hboxLayout2"));


        btnLoadData = new QPushButton(this);
        btnLoadData->setObjectName(QString::fromUtf8("loadButton"));

        hboxLayout2->addWidget(btnLoadData);

        moveButton = new QPushButton(this);
        moveButton->setObjectName(QString::fromUtf8("moveButton"));
        moveButton->setEnabled(false);

        hboxLayout2->addWidget(moveButton);

        vboxLayout->addLayout(hboxLayout2);

        btnGetPath->setText(QApplication::translate("Form", "Get Path", 0, QApplication::UnicodeUTF8));
        btnSaveData->setText(QApplication::translate("Form", "Save Data", 0, QApplication::UnicodeUTF8));
        btnLoadData->setText(QApplication::translate("Form", "Load Data", 0, QApplication::UnicodeUTF8));
        moveButton->setText(QApplication::translate("Form", "Start Move ", 0, QApplication::UnicodeUTF8));

        groupBox->setTitle(QApplication::translate("Form", "Local Planner", 0, QApplication::UnicodeUTF8));
        labelGB->setText(QApplication::translate("Form", "From:", 0, QApplication::UnicodeUTF8));
        label_2GB->setText(QApplication::translate("Form", "To:", 0, QApplication::UnicodeUTF8));
        _cmbTry->setText(QApplication::translate("Form", "Try Connect", 0, QApplication::UnicodeUTF8));
        connectLabel->setText(QString());

        _plannerTimer = new QTimer( this );

        if(_planner != NULL ){
            connect(btnGetPath, SIGNAL( clicked() ), this, SLOT( getPath() ) );
            connect(btnSaveData, SIGNAL( clicked() ), this, SLOT( saveData() ) );
            connect(btnLoadData, SIGNAL( clicked() ), this, SLOT( loadData() ) );
            connect(moveButton, SIGNAL( clicked() ), this, SLOT( simulatePath() ) );
            connect(_plannerTimer, SIGNAL(timeout()), this, SLOT(moveAlongPath()) );
            connect(globalFromBox, SIGNAL( valueChanged( int )), this, SLOT( showSample( int )));
            connect(globalToBox, SIGNAL( valueChanged( int )), this, SLOT( showSample( int )));
            connect(localFromBox, SIGNAL( valueChanged( int )), this, SLOT( showSample( int )));
            connect(localToBox, SIGNAL( valueChanged( int )), this, SLOT( showSample( int )));
            connect(_cmbTry, SIGNAL( clicked() ), this, SLOT( tryConnect( )));
            if (chkCamera) connect(chkCamera, SIGNAL( clicked() ), this, SLOT( chkCameraClick( )));

            globalFromBox->setMaximum(_samples->getSize()-1);
            globalToBox->setMaximum(_samples->getSize()-1);
            localFromBox->setMaximum(_samples->getSize()-1);
            localToBox->setMaximum(_samples->getSize()-1);
            globalFromBox->setValue( _planner->findIndex(_planner->initSamp()) );
            globalToBox->setValue( _planner->findIndex(_planner->goalSamp()) );
            localFromBox->setValue( 0 );
            localToBox->setValue( 1 );
        }
    }


    bool PlannerWidget::setTable(string s){
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
            #if defined(KAUTHAM_USE_IOC)
                globalFromBox->setMaximum(_samples->getSize()-1);
                globalToBox->setMaximum(_samples->getSize()-1);
                localFromBox->setMaximum(_samples->getSize()-1);
                localToBox->setMaximum(_samples->getSize()-1);
                globalFromBox->setValue( _planner->findIndex(_planner->initSamp()) );
                globalToBox->setValue( _planner->findIndex(_planner->goalSamp()) );
            #endif
            return true;
        }
        connect(table, SIGNAL(cellChanged(int, int)), this, SLOT(tableChanged(int, int)));
        return false;
   }



    PlannerWidget::~PlannerWidget() {
        if (_ismoving) {
            simulatePath();
        }
        delete hboxLayout;
        delete hboxLayout2;
        delete btnGetPath;
        delete btnSaveData;
        delete moveButton;
        delete chkCamera;
        delete btnLoadData;
        delete globalFromBox;
        delete globalToBox;
        delete tmpLabel;
        delete _plannerTimer;
        delete label;
        delete localFromBox;
        delete label_2;
        delete localToBox;
        delete horizontalLayout_2;
        delete _cmbTry;
        delete connectLabel;
    }


    void PlannerWidget::tryConnect() {
        if (_planner != NULL) {
            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    tryConnectIOC();
                break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER:
                    tryConnectOMPL();
                break;
                case OMPLCPLANNER:
                    tryConnectOMPLC();
                break;
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER:
                    tryConnectODE();
                break;
#endif
#endif
                case NOFAMILY:
                    writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
                break;
                default:
                    writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
                break;
            }
        } else {
            writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
        }
    }


    void PlannerWidget::tryConnectIOC() {
#if defined(KAUTHAM_USE_IOC)
        writeGUI("Sorry: Nothing implemented yet for ioc planners");
#endif
    }


    void PlannerWidget::tryConnectOMPL() {
#if defined(KAUTHAM_USE_OMPL)
        ((omplplanner::omplPlanner*)_planner)->SimpleSetup()->setup();

        ob::ScopedState<ob::CompoundStateSpace> fromState(((omplplanner::omplPlanner*)_planner)->getSpace());
        ((omplplanner::omplPlanner*)_planner)->
                smp2omplScopedState(_samples->getSampleAt(localFromBox->text().toInt()),&fromState);

        ob::ScopedState<ob::CompoundStateSpace> toState(((omplplanner::omplPlanner*)_planner)->getSpace());
        ((omplplanner::omplPlanner*)_planner)->
                smp2omplScopedState(_samples->getSampleAt(localToBox->text().toInt()),&toState);

        bool connected = ((ob::MotionValidator *)((ob::SpaceInformation *)((omplplanner::omplPlanner*)
                                                                           _planner)->
                                                  SimpleSetup()->getSpaceInformation().get())->
                          getMotionValidator().get())->checkMotion(fromState.get(),toState.get());
        if (connected) {
            connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/connect.xpm")));
            writeGUI("The samples can be connected.");
        } else {
            connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/noconnect.xpm")));
            writeGUI("The samples can NOT be connected.");
        }
#endif
    }


    void PlannerWidget::tryConnectOMPLC() {
#if defined(KAUTHAM_USE_OMPL)
        ((omplcplanner::omplcPlanner*)_planner)->SimpleSetup()->setup();

        ob::ScopedState<ob::CompoundStateSpace> fromState(((omplcplanner::omplcPlanner*)_planner)->getSpace());
        ((omplcplanner::omplcPlanner*)_planner)->
                smp2omplScopedState(_samples->getSampleAt(localFromBox->text().toInt()),&fromState);

        ob::ScopedState<ob::CompoundStateSpace> toState(((omplcplanner::omplcPlanner*)_planner)->getSpace());
        ((omplcplanner::omplcPlanner*)_planner)->
                smp2omplScopedState(_samples->getSampleAt(localToBox->text().toInt()),&toState);

        bool connected = ((ob::MotionValidator *)((ob::SpaceInformation *)((omplcplanner::omplcPlanner*)
                                                                           _planner)->
                                                  SimpleSetup()->getSpaceInformation().get())->
                          getMotionValidator().get())->checkMotion(fromState.get(),toState.get());
        if (connected) {
            connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/connect.xpm")));
            writeGUI("The samples can be connected.");
        } else {
            connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/noconnect.xpm")));
            writeGUI("The samples can NOT be connected.");
        }
#endif
    }


    void PlannerWidget::tryConnectODE() {
#if defined(KAUTHAM_USE_OMPL) && defined(KAUTHAM_USE_ODE)
        writeGUI("Sorry: Nothing implemented yet for ode planners");
#endif
    }


    void PlannerWidget::getPath() {
        emit changeCursor(true);

        bool result (false);
        if (_planner) {
            _planner->wkSpace()->moveObstaclesTo(_planner->wkSpace()->getInitObsSample());

            if (_samples->getNumStarts() == 1) {
                _planner->setInitSamp(_samples->getSampleAt(globalFromBox->text().toInt()));
            } else {
                _planner->clearInitSamp();
                for (std::size_t i(0); i < _samples->getNumStarts(); ++i) {
                    _planner->addInitSamp(_samples->getStart(i));
                }
            }

            if (_samples->getNumGoals() == 1) {
                _planner->setGoalSamp(_samples->getSampleAt(globalToBox->text().toInt()));
            } else {
                _planner->clearGoalSamp();
                for (std::size_t i(0); i < _samples->getNumGoals(); ++i) {
                    _planner->addGoalSamp(_samples->getGoal(i));
                }
            }

            result = _planner->solveAndInherit();
        }
        moveButton->setEnabled(result);
        btnSaveData->setEnabled(result);

        emit changeCursor(false);
    }


    void PlannerWidget::saveData(){
        if (_planner != NULL) {
            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
                case IOCPLANNER:
                    saveDataIOC();
                break;
#endif
#if defined(KAUTHAM_USE_OMPL)
                case OMPLPLANNER:
                    saveDataOMPL();
                break;
                case OMPLCPLANNER:
                    saveDataOMPLC();
                break;
#if defined(KAUTHAM_USE_ODE)
                case ODEPLANNER:
                    saveDataODE();
                break;
#endif
#endif
                case NOFAMILY:
                    writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
                break;
                default:
                    writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
                break;
            }
        } else {
            writeGUI("The planner is not configured properly!!. Something is wrong with your application.");
        }
    }


    void PlannerWidget::saveDataIOC() {
#if defined(KAUTHAM_USE_IOC)
        writeGUI("Sorry: Nothing implemented yet for ioc planners");
#endif
    }

    void PlannerWidget::saveDataOMPL() {
#if defined(KAUTHAM_USE_OMPL)
        emit changeCursor(true);
        QString filePath = getFilePath();
        if (!filePath.isEmpty()) {
            sendText(QString("Kautham is saving a planner data in a file: "+filePath).toUtf8().constData());
            QFile file(filePath);
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                stringstream sstr;
                if (_planner->isSolved()) {
                    Sample sample(((omplplanner::omplPlanner*)_planner)->initSamp()->getDim());
                    sample.setMappedConf(((omplplanner::omplPlanner*)_planner)->initSamp()->getMappedConf());
                    std::vector<ob::State*> &states(((omplplanner::omplPlanner*)_planner)->
                                                    SimpleSetup()->getSolutionPath().getStates());
                    for (std::vector<ob::State*>::iterator state(states.begin());
                         state != states.end(); ++state) {
                        ((omplplanner::omplPlanner*)_planner)->omplState2smp(*state,&sample);
                        for (std::vector<RobConf>::iterator robConf(sample.getMappedConf().begin());
                             robConf != sample.getMappedConf().end(); ++robConf) {
                            for (std::vector<float>::iterator coord(robConf->getSE3().getCoordinates().begin());
                                 coord != robConf->getSE3().getCoordinates().end(); ++coord) {
                                sstr << *coord << " ";
                            }
                            for (std::vector<float>::iterator coord(robConf->getRn().getCoordinates().begin());
                                 coord != robConf->getRn().getCoordinates().end(); ++coord) {
                                sstr << *coord;
                                std::vector<float>::iterator it1(coord);
                                it1++;
                                std::vector<RobConf>::iterator it2(robConf);
                                it2++;
                                if ((it1 != robConf->getRn().getCoordinates().end()) ||
                                        (it2 != sample.getMappedConf().end())) {
                                    sstr << " ";
                                }
                            }
                        }
                        sstr << endl;
                    }
                }

                QTextStream out(&file);
                out << sstr.str().c_str();

                sendText("File was saved successfully");
            } else {
                sendText("Sorry but the file couldn't be saved");
            }
            file.close();
        }
        emit changeCursor(false);
#endif
    }


    void PlannerWidget::saveDataOMPLC() {
#if defined(KAUTHAM_USE_OMPL)
        writeGUI("Sorry: Nothing implemented yet for non-ioc planners");
#endif
    }


    void PlannerWidget::saveDataODE() {
#if defined(KAUTHAM_USE_OMPL) && defined(KAUTHAM_USE_ODE)
        writeGUI("Sorry: Nothing implemented yet for non-ioc planners");
#endif
    }


    QString PlannerWidget::getFilePath() {
        QSettings settings("IOC","Kautham");
        QDir workDir;
        QString last_path = settings.value("last_path",workDir.absolutePath()).toString();
        QString filePath = QFileDialog::getSaveFileName(this->parentWidget(),
                                                        "Save planner data as...", last_path,
                                                        "Kautham Planner Solution (*.kps)");
        if (!filePath.isEmpty()) {
            uint pointIndex = filePath.lastIndexOf(".");
            uint slashIndex = filePath.lastIndexOf("/");
            if (pointIndex > slashIndex) filePath.truncate(pointIndex);
            filePath.append(".kps");
        }

        return filePath;
    }


    void PlannerWidget::loadData(){
            writeGUI("Sorry: loadData not yet implemented");
    }


    void PlannerWidget::simulatePath() {
        if (moveButton->text() == QApplication::translate("Form", "Start Move ", 0, QApplication::UnicodeUTF8)){
            startSimulation();
        } else {
            stopSimulation();
        }

    }

    void PlannerWidget::startSimulation() {
        _plannerTimer->start(200);
        moveButton->setText(QApplication::translate("Form", "Stop Move ", 0, QApplication::UnicodeUTF8));
        _ismoving = true;
    }

    void PlannerWidget::stopSimulation() {
        _plannerTimer->stop();
        moveButton->setText(QApplication::translate("Form", "Start Move ", 0, QApplication::UnicodeUTF8));
        _ismoving = false;
    }

    void PlannerWidget::moveAlongPath(){
        _planner->moveAlongPath(_stepSim);
        // It moves the camera if the associated planner provides the
        // transformation information of the camera
        //if( chkCamera && chkCamera->isChecked() && _planner->getCameraMovement(_stepSim) != NULL ) {
        //_gui->setActiveCameraTransform(*_planner->getCameraMovement( _stepSim ));
        //}

        _stepSim += _planner->getSpeedFactor();

    }

    void PlannerWidget::showSample(int index){
        int max;

        connectLabel->setPixmap(QPixmap(QString::fromUtf8(":/icons/tryconnect.xpm")));

        max = _samples->getSize();

        if (_samples->getSize() > 1) {
            globalFromBox->setMaximum(max-1);
            globalToBox->setMaximum(max-1);
            localFromBox->setMaximum(max-1);
            localToBox->setMaximum(max-1);
        }

        if (index >= 0 && index < max) {
            Sample *smp =  _samples->getSampleAt(index);
            _planner->wkSpace()->moveRobotsTo(smp );

            vector<KthReal> c = smp->getCoords();
            cout << "sample: ";

            for(unsigned i=0; i<c.size(); i++)
                cout << c[i] << ", ";

            cout << endl;

            if (smp->getMappedConf().size()!=0) {
                SE3Conf &s = smp->getMappedConf()[0].getSE3();
                cout << s.getPos().at(0) << " ";
                cout << s.getPos().at(1) << " ";
                cout << s.getPos().at(2) << endl;

            }

        } else {
            globalFromBox->setValue(0);
            globalToBox->setValue(0);
            localFromBox->setValue(0);
            localToBox->setValue(0);
        }
    }


    void PlannerWidget::chkCameraClick() {
        if (chkCamera) {
            if (chkCamera->isChecked()) {
                _planner->wkSpace()->setPathVisibility(false);
            } else {
                _planner->wkSpace()->setPathVisibility(true);
            }
        }
    }
}
