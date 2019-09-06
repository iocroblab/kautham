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

        btnGetPath->setText(QApplication::translate("Form", "Get Path", 0));
        btnSaveData->setText(QApplication::translate("Form", "Save Data", 0));
        btnLoadData->setText(QApplication::translate("Form", "Load TaskFile", 0));
        moveButton->setText(QApplication::translate("Form", "Start Move ", 0));

        groupBox->setTitle(QApplication::translate("Form", "Local Planner", 0));
        labelGB->setText(QApplication::translate("Form", "From:", 0));
        label_2GB->setText(QApplication::translate("Form", "To:", 0));
        _cmbTry->setText(QApplication::translate("Form", "Try Connect", 0));
        connectLabel->setText(QString());

        _plannerTimer = new QTimer( this );

        if(_planner != NULL ){
            connect(btnGetPath, SIGNAL( clicked() ), this, SLOT( getPath() ) );
            connect(btnSaveData, SIGNAL( clicked() ), this, SLOT( saveData() ) );
            connect(btnLoadData, SIGNAL( clicked() ), this, SLOT( taskmotionPathLoad() ) );
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

 
    void PlannerWidget::simulatePath() {
        if (moveButton->text() == QApplication::translate("Form", "Start Move ", 0)){
            startSimulation();
        } else {
            stopSimulation();
        }

    }

    void PlannerWidget::startSimulation() {
        _plannerTimer->start(200);
        moveButton->setText(QApplication::translate("Form", "Stop Move ", 0));
        _ismoving = true;
    }

    void PlannerWidget::stopSimulation() {
        _plannerTimer->stop();
        moveButton->setText(QApplication::translate("Form", "Start Move ", 0));
        _ismoving = false;
    }



    void PlannerWidget::moveAlongPath(){
        _stepSim = _planner->moveAlongPath(_stepSim);
        // It moves the camera if the associated planner provides the
        // transformation information of the camera
        //if( chkCamera && chkCamera->isChecked() && _planner->getCameraMovement(_stepSim) != NULL ) {
        //_gui->setActiveCameraTransform(*_planner->getCameraMovement( _stepSim ));
        //}

        _stepSim += _planner->getSpeedFactor();

    }


    void PlannerWidget::loadSampleFromLine(Sample *Robsmp,std::stringstream  &confstream)
    {
        //Load the configurations of the robots written in each line
        //loop for all the robots
        vector<RobConf> Robrc; //mapped configuration of the set of robots
        for (uint j = 0; j < _planner->wkSpace()->getNumRobots(); ++j) 
        {
            //std::cout << " robot "<< j << std::endl;
            RobConf *Robrcj = new RobConf; //Configuration for robot j
            vector<KthReal> se3coords; //SE3 part
            se3coords.resize(7);
            std::vector<KthReal> Rn; //Rn part
            //Mapped configuration: load the SE3 part, if ther is any
            if (_planner->wkSpace()->getRobot(j)->isSE3Enabled()) 
            {
            //std::cout << " se3 part ";
            //read the next 7 values
            for(int k=0; k<7; k++)
                confstream >> se3coords[k];
            //for(int k=0; k<7; k++)
            //  std::cout << " "<< se3coords[k];
            //std::cout << std::endl;
            }
            else{
                //If the robot does not have mobile SE3 dofs then the SE3 configuration of the init sample is maintained
                Robrcj->setSE3(_planner->initSamp()->getMappedConf()[0].getSE3());
            }
            SE3Conf se3;
            se3.setCoordinates(se3coords);
            Robrcj->setSE3(se3);
                      
            //Mapped configuration: load the Rn part, if ther is any
            if (_planner->wkSpace()->getRobot(j)->getNumJoints() > 0) 
            {
            //std::cout << " rn part: ";
            Rn.resize(_planner->wkSpace()->getRobot(j)->getNumJoints());
            //read the next n values
            for(uint k=0; k<_planner->wkSpace()->getRobot(j)->getNumJoints(); k++)
                confstream >> Rn[k];

            Robrcj->setRn(Rn);
            //for(int k=0; k<_planner->wkSpace()->getRobot(j)->getNumJoints(); k++)
            //  std::cout << " "<< Rn[k];
            //std::cout << std::endl;
            }
            else {
            //If the robot does not have mobile Rn dofs then the Rn is set to dim=0
            Robrcj->setRn(0);
            }

            //load the configuration of robot j to the configuration vector
            Robrc.push_back(*Robrcj);
        }

        //Set the mapped configuration of the sample
        //(its control values are kept to zero)               
        Robsmp->setMappedConf(Robrc);    
    }      


    bool PlannerWidget::taskmotionPathLoad(){
        //This functions reads a taskmotion.xml planning file and fills the paths to be followed
        //composed of the concatenation of transit and transfer paths,
        //and the vector with the attachements/detachements to be done at the begining/ending
        //of each transfer.
        std::ifstream xml_taskfile;
        xml_document *doc = new xml_document;
        std::vector<Sample*> _path2;

        //open xml file with the task plan
        QString last_path, path;
        QDir workDir;
        QString fileName;
        QSettings settings("IOC","Kautham");
        last_path = settings.value("last_path",workDir.absolutePath()).toString();
        path = QFileDialog::getOpenFileName(
                this->parentWidget(),
                "Choose a task file to open (*.xml)",
                last_path,
                "All task files (*.xml)");
        xml_taskfile.open(path.toStdString());
        xml_parse_result result = doc->load( xml_taskfile );

        //parse the file
        if (result) {
            //if the file was correctly parsed
            _planner->clearAttachData();
            _path2.clear();
            xml_node taskNode = doc->child("Task");
            string name;
            //Loop inside the task node for transit and transfer nodes
            for (xml_node::iterator it_task = taskNode.begin(); it_task != taskNode.end(); ++it_task) {
                name = it_task->name();
                try {
                    if (name == "Transit") {
                        std::cout<<"....Transit..."<<std::endl;
                        string transitconf;
                        //loop inside the transit node for all the configurations of the path
                        for (xml_node::iterator it_transit = it_task->begin(); it_transit != it_task->end(); ++it_transit) {
                            name = it_transit->name();
                            try {
                                if (name == "Conf") {
                                    transitconf = it_transit->child_value();
                                    std::cout<<"TRANSIT:"<<transitconf<<endl;
                                    //Create the sample
                                    Sample *Robsmp; //sample
                                    Robsmp=new Sample(_planner->wkSpace()->getNumRobControls());
                                    //fill the sample
                                    std::stringstream transitconfstream(transitconf);
                                    loadSampleFromLine(Robsmp, transitconfstream);
                                    //add the sample to the path
                                    _path2.push_back(Robsmp);
                                }
                            } catch(...) {
                                std::cout << "ERROR: Current transit tag does not have any configuration!!" << std::endl;
                                return false;
                            }
                        }
                    }
                    else if (name == "Transfer") {
                        //Read the attributes with the object to be transferred and the robot and link where it is attached to
                        //<Transfer object="1" robot="0" link="0">
                        string objectname = it_task->attribute("object").as_string();
                        string robotname = it_task->attribute("robot").as_string();
                        string linkname = it_task->attribute("link").as_string();
                        std::cout<< "....Transfer...object "<<objectname<<" attached to link "<<linkname<<" of robot "<<robotname<< std::endl;
                        string transferconf;
                        
                        //load the step of the path where transfer starts and an attach is required
                        _planner->loadAttachData(_path2.size(),"attach",stoi(objectname),stoi(robotname),stoi(linkname));

                        //loop inside the transfer node for all the configurations of the path
                        for (xml_node::iterator it_transfer = it_task->begin(); it_transfer != it_task->end(); ++it_transfer) {
                            name = it_transfer->name();
                            try {
                                if (name == "Conf") {
                                    transferconf = it_transfer->child_value();
                                    std::cout<<"TRANSFER:"<<transferconf<<endl;
                                    //Create the sample
                                    Sample *Robsmp; //sample
                                    Robsmp=new Sample(_planner->wkSpace()->getNumRobControls());
                                    //fill the sample
                                    std::stringstream transferconfstream(transferconf);
                                    loadSampleFromLine(Robsmp, transferconfstream);
                                    //add the sample to the path
                                    _path2.push_back(Robsmp);
                                }
                            } catch(...) {
                                std::cout << "ERROR: Current transfer tag does not have any configuration!!" << std::endl;
                                return false;
                            }
                        }
                        //load the step of the path where transfer ends and a dettach is required
                        _planner->loadAttachData(_path2.size()-1,"detach",stoi(objectname),stoi(robotname),stoi(linkname));
                    }                
                } catch(...) {
                    std::cout << "ERROR: Current task tag does not have any transit or transfer !!" << std::endl;
                    return false;
                }
            }
        } else {
            string message = "Problem file " + path.toStdString() + " couldn't be parsed";
            stringstream details;
            details << "Error: " << result.description() << endl <<
                       "Last successfully parsed character: " << result.offset;
            //throw KthExcp(message,details.str());
            std::cout<<"TRANSIT:"<<message<<endl;

            return false;
        }
        
        for(unsigned i=0; i<_path2.size();i++)
        {
            cout << "PATH: ";
            if (_path2.at(i)->getMappedConf().size()!=0) {
                SE3Conf &s = _path2.at(i)->getMappedConf()[0].getSE3();
                cout << s.getPos().at(0) << " ";
                cout << s.getPos().at(1) << " ";
                cout << s.getPos().at(2) << endl;
            }
        }
        _planner->loadExternalPath(_path2);
        
        moveButton->setEnabled(true);
        btnSaveData->setEnabled(true);

        _stepSim = 0; //to start simulation from the begining
        _planner->clearSimulationPath();

        return true;
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
