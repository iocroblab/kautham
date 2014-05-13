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


  
#include <Inventor/Qt/SoQt.h>
#include <QFile>
#include <QSettings>
#include <QString>
#include <QMessageBox>
#include <sstream>
#include <problem/ivworkspace.h>
#include <util/kthutil/kauthamdefs.h>
#include "kauthamgui.h"
#include "dofwidget.h"
#include <QApplication>
#include <QWidget>
#include <Inventor/Qt/SoQt.h>


Application::Application() {
    settings = new QSettings("IOC","Kautham");
    Q_INIT_RESOURCE(kauthamRes);
    initApp();
}

void Application::initApp(){
    mainWindow = new GUI();
    settings->beginGroup("mainWindow");

    mainWindow->resize(settings->value("size",QSize(1024,768)).toSize());
    mainWindow->move(settings->value("pos",QPoint(0,0)).toPoint()-mainWindow->pos());
    if (settings->value("fullScreen",false).toBool()) {
        mainWindow->showFullScreen();
    }
    settings->endGroup();

    SoQt::show(mainWindow);
    setActions();
    mainWindow->setText("Open a problem file to start...");
    appState = INITIAL ;
    _problem = NULL;
}

Application::~Application() {
    settings->beginGroup("mainWindow");
    settings->setValue("size",mainWindow->size());
    settings->setValue("pos",mainWindow->pos());
    settings->setValue("fullScreen",mainWindow->isFullScreen());
    settings->endGroup();

    if (appState == PROBLEMLOADED) {
        saveTabColors();
        closeProblem();
    }

    delete settings;
}

void Application::setActions(){
    mainWindow->setAction(FILETOOL,"&Open","CTRL+O",":/icons/fileopen.xpm",this,SLOT(openFile()));
    mainWindow->setAction(FILETOOL,"&Save","CTRL+S",":/icons/filesave.xpm",this,SLOT(saveFile()));
    mainWindow->setAction(FILETOOL,"Save &as","CTRL+A",":/icons/saveas.xpm",this,SLOT(saveAsFile()));
    mainWindow->addSeparator(TOOLBAR);

    // Creating the planner toolbar in the main Window. This list may change.
    //string loc = Problem::localPlannersNames();
    //string glob = Problem::plannersNames();


    //mainWindow->createPlannerToolBar(loc, glob,this,SLOT(changePlanner(string,string)));
    //mainWindow->setToogleAction(ACTIONTOOL,"&Find path","CTRL+F",":/icons/prm.xpm",mainWindow,SLOT(showPlannerToolBar()));
    mainWindow->addSeparator(TOOLBAR);
    mainWindow->addSeparator(ACTIONMENU);

    mainWindow->addSeparator(TOOLBAR);
    mainWindow->setAction(ACTIONTOOL,"Chan&ge Colour","CTRL+G",
                          ":/icons/determ.xpm", mainWindow, SLOT(changeActiveBackground()));

#if  defined(KAUTHAM_USE_ARMADILLO)
    if (settings->value("use_BBOX","false").toBool()) {
        mainWindow->setAction(ACTIONTOOL,"Disable BBOX","",":/icons/BBOXdisabled.xpm", mainWindow, SLOT(toogleBBOXflag()));
    } else {
        mainWindow->setAction(ACTIONTOOL,"Enable BBOX","",":/icons/BBOXenabled.xpm", mainWindow, SLOT(toogleBBOXflag()));
    }
#endif

    mainWindow->setAction(ACTIONTOOL,"Default Path","",":/icons/search.xpm", mainWindow, SLOT(setModelsDefaultPath()));

    mainWindow->setAction(FILETOOL,"&Close","CTRL+Q",":/icons/close.xpm",this,SLOT(closeProblem()));
}

void Application::openFile(){
    QString last_path, path, dir;
    QDir workDir;
    last_path = settings->value("last_path",workDir.absolutePath()).toString();
    mainWindow->setCursor(QCursor(Qt::WaitCursor));
    path = QFileDialog::getOpenFileName(
                mainWindow,
                "Choose a file to open",
                last_path,
                "All configuration files (*.xml)");
    if(!path.isEmpty()){
        if (appState == PROBLEMLOADED) {
            closeProblem();
            appState = INITIAL;
        }
        mainWindow->setText("Kautham is opening a problem file...");
        dir = path;
        dir.truncate(dir.lastIndexOf("/"));
        if (problemSetup(path.toUtf8().constData())) {
            mainWindow->showProblemAppearance();
            settings->setValue("last_path",dir);

            stringstream tmp;
            tmp << "Kautham ";
            tmp << MAJOR_VERSION;
            tmp << ".";
            tmp << MINOR_VERSION;
            tmp << " - ";
            tmp << path.toUtf8().constData();
            mainWindow->setWindowTitle( tmp.str().c_str() );
            mainWindow->setText(QString("File: ").append(path).toUtf8().constData() );
            mainWindow->setText("opened successfully.");
        } else {
            mainWindow->setText("Kautham couldn't open the problem file...");
        }
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}

void Application::saveFile(){
    mainWindow->setCursor(QCursor(Qt::WaitCursor));
    if( appState == PROBLEMLOADED ){
        if( _problem->saveToFile() )
            mainWindow->setText( "File saved successfully" );
        else
            mainWindow->setText( "Sorry but the file is not saved" );
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}

void Application::saveAsFile(){
    QString path,dir;
    QDir workDir;
    mainWindow->setCursor(QCursor(Qt::WaitCursor));
    switch(appState){
    case PROBLEMLOADED:
        path = QFileDialog::getSaveFileName(
                    mainWindow,
                    "Save as ...",
                    workDir.absolutePath(),
                    "All configuration files (*.xml)");
        if(!path.isEmpty()){
            mainWindow->setText( "Kautham is saving a problem file: " );
            mainWindow->setText( path.toUtf8().constData() );
            dir = path;
            dir.truncate(dir.lastIndexOf("/"));
            if( _problem->saveToFile( path.toUtf8().constData() ) )
                mainWindow->setText( "File saved successfully" );
            else
                mainWindow->setText( "Sorry but the file is not saved" );
        }
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}


void Application::closeProblem(){
    mainWindow->setCursor(QCursor(Qt::WaitCursor));
    switch(appState){
    case INITIAL:
        mainWindow->setText("First open a problem");
        break;
    case PROBLEMLOADED:
        if(mainWindow->getPlannerWidget()->ismoving())
            mainWindow->getPlannerWidget()->simulatePath();//stops simulation
        saveTabColors();
        mainWindow->restart();
        delete _problem;
        appState = INITIAL;

        break;
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}


void Application::saveTabColors() {
    vector <string> viewers;
    viewers.push_back("WSpace");
    viewers.push_back("CollisionWSpace");
    viewers.push_back("CSpace");
    SbColor color;
    settings->beginGroup("mainWindow");
    for (int i = 0; i < viewers.size(); i++) {
        if (mainWindow->getViewerTab(viewers.at(i)) != NULL) {
            settings->beginGroup(viewers.at(i).c_str());
            color = mainWindow->getViewerTab(viewers.at(i))->getBackgroundColor();
            settings->setValue("color",QColor((int)(255.0*color.getValue()[0]),
                                              (int)(255.0*color.getValue()[1]),
                                              (int)(255.0*color.getValue()[2])));
            settings->endGroup();
        }
    }
    settings->endGroup();
}


bool Application::problemSetup(string problemFile){
    mainWindow->setCursor(QCursor(Qt::WaitCursor));

    string dir = problemFile.substr(problemFile.find_last_of("/")+1);
    string models_def_path = settings->value("default_path/models",QString(string(dir+string("/../../models")).c_str())).
            toString().toStdString()+"/";
    bool useBBOX = settings->value("use_BBOX","false").toBool();

    _problem = new Problem();
    if (!_problem->setupFromFile(problemFile,models_def_path,useBBOX)) {
        appState = INITIAL;
        delete _problem;
        return false;
    }

    mainWindow->addToProblemTree(problemFile);

    mainWindow->addViewerTab("WSpace", ((IVWorkSpace*)_problem->wSpace())->getIvScene());
    QColor color = settings->value("mainWindow/WSpace/color",QColor("black")).value<QColor>();
    mainWindow->getViewerTab("WSpace")->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));

    mainWindow->addViewerTab("CollisionWSpace", ((IVWorkSpace*)_problem->wSpace())->getCollisionIvScene());
    color = settings->value("mainWindow/CollisionWSpace/color",QColor("black")).value<QColor>();
    mainWindow->getViewerTab("CollisionWSpace")->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));

    //  Used to show the IV models reconstructed from the PQP triangular meshes.
    //mainWindow->addViewerTab("PQP", ((IVWorkSpace*)_problem->wSpace())->getIvFromPQPScene());

    vector <DOFWidget*> robDOFWidgets;
    robDOFWidgets.resize(_problem->wSpace()->getNumRobots());
    for(uint i = 0; i < _problem->wSpace()->getNumRobots(); i++) {

        if(_problem->wSpace()->getRobot(i)->getCkine() != NULL)
            mainWindow->addConstrainedControlWidget(_problem->wSpace()->getRobot(i), _problem);

        robDOFWidgets[i] = mainWindow->addDOFWidget(_problem->wSpace()->getRobot(i));

        //Add widget for external applications
        //widget 1 used for virtual bronchoscopy apllication
        mainWindow->addExternalWidget1(_problem->wSpace()->getRobot(i), _problem, mainWindow);
        //widget 2 not used
        mainWindow->addExternalWidget2(_problem->wSpace()->getRobot(i), _problem, mainWindow);
        //widget 3 not used
        mainWindow->addExternalWidget3(_problem->wSpace()->getRobot(i), _problem, mainWindow);


        if(_problem->wSpace()->getRobot(i)->getIkine() != NULL)
            mainWindow->addInverseKinematic(_problem->wSpace()->getRobot(i)->getIkine());
    }
    mainWindow->addRobControlWidget(_problem,robDOFWidgets);

    if (_problem->wSpace()->getNumObsControls() > 0) {
        vector <DOFWidget*> obsDOFWidgets;
        obsDOFWidgets.resize(_problem->wSpace()->getNumObstacles());
        for(uint i = 0; i < _problem->wSpace()->getNumObstacles(); i++) {
            obsDOFWidgets[i] = mainWindow->addDOFWidget(_problem->wSpace()->getObstacle(i));
        }

        mainWindow->addObsControlWidget(_problem,obsDOFWidgets);
    }

    mainWindow->setSampleWidget(_problem->getSampleSet(), _problem->getSampler(), _problem);

    if( _problem->getPlanner() != NULL ){
        mainWindow->addPlanner(_problem->getPlanner(), _problem->getSampleSet(), mainWindow);
        color = settings->value("mainWindow/CSpace/color",QColor("black")).value<QColor>();
        if (mainWindow->getViewerTab("CSpace") != NULL) {
            mainWindow->getViewerTab("CSpace")->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));
        }
    }
    appState = PROBLEMLOADED;

    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
    return true;
}

int main(int argc, char* argv[]){       

    try{
        QWidget *app = SoQt::init(argv[0]);//argc, argv,argv[0]);
        app->setVisible(false);
        Application kauthApp;
        SoQt::mainLoop();
        return 0;
    }
    catch(...){
            std::cout <<"Unexpected error in Kautham initialization"<<endl;
        }
}


