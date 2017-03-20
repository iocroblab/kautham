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


#include <sstream>
  
#include <QFile>
#include <QSettings>
#include <QString>
#include <QMessageBox>
#include <QApplication>
#include <QWidget>

#include <Inventor/Qt/SoQt.h>

#include <kautham/problem/ivworkspace.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/util/kthutil/kauthamexception.h>
#include "kauthamgui.h"
#include "dofwidget.h"

//The following config header file contains the version number of kautham and is automatically generated from the
//CMakeList.txt file using the configure file option
#include <kautham/kauthamConfig.h>


#define MAXRECENTFILES 5

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
    mainWindow->showInitialAppearance();
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
    QIcon fileopen;
    fileopen.addFile(":/icons/fileopen_16x16.png");
    fileopen.addFile(":/icons/fileopen_22x22.png");
    fileopen.addFile(":/icons/fileopen_32x32.png");
    fileopen.addFile(":/icons/fileopen_48x48.png");
    fileopen.addFile(":/icons/fileopen_64x64.png");
    mainWindow->setAction(FILETOOL,"&Open","CTRL+O",fileopen,this,SLOT(openFile()));
    
    QIcon filesave;
    filesave.addFile(":/icons/filesave_16x16.png");
    filesave.addFile(":/icons/filesave_22x22.png");
    filesave.addFile(":/icons/filesave_32x32.png");
    filesave.addFile(":/icons/filesave_48x48.png");
    filesave.addFile(":/icons/filesave_64x64.png");
    mainWindow->setAction(FILETOOL,"&Save","CTRL+S",filesave,this,SLOT(saveFile()));
    
    QIcon filesaveas;
    filesaveas.addFile(":/icons/filesaveas_16x16.png");
    filesaveas.addFile(":/icons/filesaveas_22x22.png");
    filesaveas.addFile(":/icons/filesaveas_32x32.png");
    filesaveas.addFile(":/icons/filesaveas_48x48.png");
    filesaveas.addFile(":/icons/filesaveas_64x64.png");
    mainWindow->setAction(FILETOOL,"Save &as","CTRL+A",filesaveas,this,SLOT(saveAsFile()));
    
    mainWindow->addSeparator(FILETOOL);
    
    setRecentFilesAction();
    
    mainWindow->addSeparator(FILEMENU);
    
    QIcon fileclose;
    fileclose.addFile(":/icons/fileclose_16x16.png");
    fileclose.addFile(":/icons/fileclose_22x22.png");
    fileclose.addFile(":/icons/fileclose_32x32.png");
    fileclose.addFile(":/icons/fileclose_48x48.png");
    fileclose.addFile(":/icons/fileclose_64x64.png");
    mainWindow->setAction(FILETOOL,"&Close","CTRL+Q",fileclose,this,SLOT(closeProblem()));

    mainWindow->addSeparator(TOOLBAR);

    QIcon colors;
    colors.addFile(":/icons/colors_16x16.png");
    colors.addFile(":/icons/colors_22x22.png");
    colors.addFile(":/icons/colors_32x32.png");
    colors.addFile(":/icons/colors_48x48.png");
    colors.addFile(":/icons/colors_64x64.png");
    mainWindow->setAction(ACTIONTOOL,"Chan&ge Colour","CTRL+G",colors,mainWindow,SLOT(changeActiveBackground()));

    QIcon magnet;
    magnet.addFile(":/icons/magnet_16x16.png");
    magnet.addFile(":/icons/magnet_22x22.png");
    magnet.addFile(":/icons/magnet_32x32.png");
    magnet.addFile(":/icons/magnet_48x48.png");
    magnet.addFile(":/icons/magnet_64x64.png");
    mainWindow->setAction(ACTIONTOOL,"At/Detach Object","",magnet,mainWindow,SLOT(attachObject()));
    
    QIcon image;
    image.addFile(":/icons/image_16x16.png");
    image.addFile(":/icons/image_22x22.png");
    image.addFile(":/icons/image_32x32.png");
    image.addFile(":/icons/image_48x48.png");
    image.addFile(":/icons/image_64x64.png");
    mainWindow->setAction(ACTIONTOOL,"Export scene","",image,mainWindow,SLOT(scene2VRML()));

#if  defined(KAUTHAM_USE_ARMADILLO)
    if (settings->value("use_BBOX","false").toBool()) {
        QIcon box;
        box.addFile(":/icons/box_16x16.png");
        box.addFile(":/icons/box_22x22.png");
        box.addFile(":/icons/box_32x32.png");
        box.addFile(":/icons/box_48x48.png");
        box.addFile(":/icons/box_64x64.png");
        mainWindow->setAction(ACTIONTOOL,"BBOX enabled","",box,mainWindow,SLOT(toogleBBOXflag()));
    } else {
        QIcon greybox;
        greybox.addFile(":/icons/greybox_16x16.png");
        greybox.addFile(":/icons/greybox_22x22.png");
        greybox.addFile(":/icons/greybox_32x32.png");
        greybox.addFile(":/icons/greybox_48x48.png");
        greybox.addFile(":/icons/greybox_64x64.png");
        mainWindow->setAction(ACTIONTOOL,"BBOX disabled","",greybox, mainWindow, SLOT(toogleBBOXflag()));
    }
#endif

    QIcon filefind;
    filefind.addFile(":/icons/filefind_16x16.png");
    filefind.addFile(":/icons/filefind_22x22.png");
    filefind.addFile(":/icons/filefind_32x32.png");
    filefind.addFile(":/icons/filefind_48x48.png");
    filefind.addFile(":/icons/filefind_64x64.png");
    mainWindow->setAction(ACTIONTOOL,"Default Path","",filefind, mainWindow, SLOT(setModelsDefaultPath()));

    mainWindow->addSeparator(TOOLBAR);

    QIcon exit;
    exit.addFile(":/icons/exit_16x16.png");
    exit.addFile(":/icons/exit_22x22.png");
    exit.addFile(":/icons/exit_32x32.png");
    exit.addFile(":/icons/exit_48x48.png");
    exit.addFile(":/icons/exit_64x64.png");
    mainWindow->setAction(FILETOOL,"E&xit","CTRL+X",exit,this,SLOT(quit()));
}

void Application::setRecentFilesAction() {
    mainWindow->setRecentFilesMenu();
    QStringList recentFiles = settings->value("recent_files",QStringList()).toStringList();
    if (recentFiles.size() > 0) {
        mainWindow->showRecentFiles(true);
        QAction *ac;
        QSignalMapper* signalMapper = new QSignalMapper (this) ;
        for (int i = 0; i < recentFiles.size(); i++) {
            string problemFile = recentFiles.at(i).toStdString();
            stringstream problemName;
            problemName << "&" << i+1 << " " << problemFile.substr(problemFile.find_last_of("/")+1);
            ac = new QAction(problemName.str().c_str(),this);
            connect(ac,SIGNAL(triggered()),signalMapper,SLOT(map()));
            mainWindow->setAction(RECENTFILESMENU,ac);
            signalMapper->setMapping(ac,problemFile.c_str());
        }
        connect(signalMapper,SIGNAL(mapped(QString)),this,SLOT(openFile(QString)));
        mainWindow->addSeparator(RECENTFILESMENU);
        mainWindow->setAction(RECENTFILESMENU,"Clear Menu","",QIcon(),this,SLOT(clearRecentFiles()));
    } else {
        mainWindow->showRecentFiles(false);
    }
}

void Application::updateRecentFiles(string problemFile) {
    QStringList oldrecentFiles = settings->value("recent_files",QStringList()).toStringList();
    for (QStringList::Iterator i = oldrecentFiles.begin(); i != oldrecentFiles.end(); i++) {
        if (i->toStdString() == problemFile) {
            oldrecentFiles.erase(i);
            break;
        }
    }
    QStringList newrecentFiles;
    newrecentFiles.push_back(problemFile.c_str());
    for (int i = 0; i < oldrecentFiles.size() && i < MAXRECENTFILES - 1; i++) {
        newrecentFiles.push_back(oldrecentFiles.at(i));
    }
    settings->setValue("recent_files",newrecentFiles);
    setRecentFilesAction();
}

void Application::clearRecentFiles() {
    settings->remove("recent_files");
    setRecentFilesAction();
}

void Application::openFile(QString problemFile) {
    if (problemFile.isEmpty()) {
        QString last_path, path;
        QDir workDir;
        last_path = settings->value("last_path",workDir.absolutePath()).toString();
        mainWindow->setCursor(QCursor(Qt::WaitCursor));
        path = QFileDialog::getOpenFileName(
                    mainWindow,
                    "Choose a file to open",
                    last_path,
                    "All configuration files (*.xml)");
        if (!path.isEmpty()) {
            problemFile = path.toUtf8().constData();
        }
    }
    if (!problemFile.isEmpty()) {
        if (appState == PROBLEMLOADED) {
            closeProblem();
            appState = INITIAL;
        }
        mainWindow->setText("Opening a problem file...");
        QString dir = problemFile;
        dir.truncate(dir.lastIndexOf("/"));
        if (problemSetup(problemFile.toStdString())) {
            mainWindow->showProblemAppearance();
            settings->setValue("last_path",dir);
            updateRecentFiles(problemFile.toStdString());

            stringstream tmp;
            tmp << "Kautham ";
            tmp << MAJOR_VERSION;
            tmp << ".";
            tmp << MINOR_VERSION;
            tmp << " - ";
            tmp << PATCH_VERSION;
            tmp << " - ";
            tmp << problemFile.toStdString();
            mainWindow->setWindowTitle( tmp.str().c_str() );
            mainWindow->setText("File: " + problemFile.toStdString() + " was opened successfully.");
        } else {
            if (abort) {
                mainWindow->setText("Problem setup was aborted.");
            } else {
                mainWindow->setText("The problem couldn't be opened.");
            }
        }
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}

void Application::saveFile(){
    if( appState == PROBLEMLOADED ){
        mainWindow->setCursor(QCursor(Qt::WaitCursor));
        if( _problem->saveToFile() ) {
            mainWindow->setText( "File was saved successfully" );
        } else {
            mainWindow->setText( "Sorry but the file was not saved" );
        }
        mainWindow->setCursor(QCursor(Qt::ArrowCursor));
    }
}

void Application::saveAsFile(){
    if (appState == PROBLEMLOADED) {
        mainWindow->setCursor(QCursor(Qt::WaitCursor));
        QDir workDir;
        QString last_path = settings->value("last_path",workDir.absolutePath()).toString();
        QString path = QFileDialog::getSaveFileName(
                    mainWindow,
                    "Save as ...",
                    last_path,
                    "All configuration files (*.xml)");        
        if (!path.isEmpty()) {
            uint pointIndex = path.lastIndexOf(".");
            uint slashIndex = path.lastIndexOf("/");
            if (pointIndex > slashIndex) path.truncate(pointIndex);
            path.append(".xml");

            mainWindow->setText( "Kautham is saving the problem to the file: " );
            mainWindow->setText( path.toUtf8().constData() );
            if( _problem->saveToFile( path.toUtf8().constData() ) ) {
                mainWindow->setText( "File was saved successfully" );
                last_path = path;
                last_path.truncate(last_path.lastIndexOf("/"));
                settings->setValue("last_path",last_path);
            } else
                mainWindow->setText( "Sorry but the file couldn't be saved" );
        }
        mainWindow->setCursor(QCursor(Qt::ArrowCursor));
    }
}


void Application::closeProblem(){
    mainWindow->setCursor(QCursor(Qt::WaitCursor));
    switch(appState){
    case INITIAL:
        mainWindow->setText("First open a problem");
        break;
    case PROBLEMLOADED:
        mainWindow->stopPathSimulation();
        saveTabColors();
        mainWindow->restart();
        delete _problem;
        appState = INITIAL;
        break;
    }
    mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}


void Application::quit() {
    QApplication::quit();
}


void Application::saveTabColors() {
    vector <string> viewers;
    viewers.push_back("WSpace");
    viewers.push_back("CollisionWSpace");
    viewers.push_back("CSpace");
    SbColor color;
    settings->beginGroup("mainWindow");
    for (unsigned i = 0; i < viewers.size(); i++) {
        if (mainWindow->getViewerTab(viewers.at(i))) {
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


struct arg_struct {
    Problem *problem;
    xml_document *doc;
    bool useBBOX;
    progress_struct *progress;
    int links2Load;
    bool succeed;
    KthExcp *excp;
};


void *loadProblem(void *arguments) {
    struct arg_struct *args = (struct arg_struct *)arguments;

    try {
        args->succeed = args->problem->setupFromFile(args->doc,args->useBBOX,
                                                args->progress);
    } catch (const KthExcp& excp) {
        args->succeed = false;
        args->excp = new KthExcp(excp.what(),excp.more());
    } catch (const exception& excp) {
        args->succeed = false;
        args->excp = new KthExcp(excp.what(),"");
    } catch (...) {
        args->succeed = false;
        args->excp = new KthExcp("Unknown error","");
    }

    pthread_mutex_lock(args->progress->mutex);
    *(args->progress->linksLoaded) = args->links2Load+1;
    pthread_mutex_unlock(args->progress->mutex);

    pthread_exit(NULL);
}


bool Application::problemSetup(string problemFile){
    mainWindow->setCursor(QCursor(Qt::WaitCursor));

    string dir = problemFile.substr(0,problemFile.find_last_of("/")+1);
    QStringList pathList = settings->value("models_directories",QStringList()).toStringList();
    std::vector <string> def_path;
    def_path.push_back(dir);
    if (pathList.size() > 0) {
        for (int i = 0; i < pathList.size(); i++) {
            def_path.push_back(pathList.at(i).toStdString()+"/");
        }
    } else {
        def_path.push_back(dir+string("../../models/"));
    }
    bool useBBOX = settings->value("use_BBOX","false").toBool();

    pthread_t *loadProblem_thread = NULL;
    pthread_mutexattr_t *mutexattr = NULL;
    struct arg_struct *arguments = NULL;
    _problem = new Problem();
    bool succeed = false;
    abort = false;
    QProgressDialog *progressDialog = NULL;
    xml_document *doc = NULL;
    try {
        //Parse the problem file and count the number of links to load
        int links2Load = 0;
        doc = _problem->parseProblemFile(problemFile,def_path,&links2Load);

        if (links2Load > 0 && doc != NULL) {
            //Load the problem and show the progress dialog

            arguments = new struct arg_struct;
            arguments->problem = _problem;
            arguments->doc = doc;
            arguments->useBBOX = useBBOX;
            arguments->progress = new struct progress_struct;
            arguments->succeed = false;
            arguments->links2Load = links2Load;
            arguments->excp = NULL;

            mutexattr = new pthread_mutexattr_t;
            pthread_mutexattr_init(mutexattr);
            pthread_mutexattr_settype(mutexattr,PTHREAD_MUTEX_NORMAL);
            pthread_mutexattr_setprotocol(mutexattr,PTHREAD_PRIO_PROTECT);
            pthread_mutexattr_setprioceiling(mutexattr,1);

            arguments->progress->mutex = new pthread_mutex_t;
            pthread_mutex_init(arguments->progress->mutex,mutexattr);

            arguments->progress->linksLoaded = new int;
            *(arguments->progress->linksLoaded) = 0;

            arguments->progress->abort = false;

            progressDialog = new QProgressDialog(mainWindow);
            progressDialog->setLabelText("Loading problem ...");
            progressDialog->setWindowModality(Qt::WindowModal);
            progressDialog->setMinimumDuration(0);
            progressDialog->setVisible(true);
            progressDialog->setMinimum(0);
            progressDialog->setMaximum(links2Load);
            progressDialog->setValue(0);

            loadProblem_thread = new pthread_t;
            pthread_create(loadProblem_thread,NULL,loadProblem,(void*)arguments);

            int linksLoaded = 0;
            while (linksLoaded <= links2Load && !abort) {
                QApplication::processEvents(QEventLoop::AllEvents);
                abort = progressDialog->wasCanceled();

                pthread_mutex_lock(arguments->progress->mutex);
                arguments->progress->abort = abort;
                linksLoaded = *(arguments->progress->linksLoaded);
                pthread_mutex_unlock(arguments->progress->mutex);

                progressDialog->setValue(linksLoaded);
            }
        }
    } catch (const KthExcp& excp) {
        qDebug() << excp.what() << endl << excp.more() << endl;

        mainWindow->setDisabled(true);
        QMessageBox msgBox(mainWindow);
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setWindowTitle("Error encountered");
        msgBox.setText("The problem couldn't be loaded");
        msgBox.setInformativeText(excp.what());
        msgBox.setDetailedText(excp.more());
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
        mainWindow->setDisabled(false);
    } catch (const exception& excp) {
        qDebug() << excp.what() << endl;

        mainWindow->setDisabled(true);
        QMessageBox msgBox(mainWindow);
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setWindowTitle("Error encountered");
        msgBox.setText("The problem couldn't be loaded");
        msgBox.setInformativeText(excp.what());
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
        mainWindow->setDisabled(false);
    } catch (...) {
        qDebug() << "Unknown error" << endl;

        mainWindow->setDisabled(true);
        QMessageBox msgBox(mainWindow);
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setWindowTitle("Error encountered");
        msgBox.setText("The problem couldn't be loaded");
        msgBox.setInformativeText("Unknown error");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
        mainWindow->setDisabled(false);
    }

    if (doc != NULL) {
        if (!arguments || !loadProblem_thread) return false;

        if (abort) {
            pthread_mutex_lock(arguments->progress->mutex);
            arguments->progress->abort = abort;
            pthread_mutex_unlock(arguments->progress->mutex);
        }
        pthread_join(*loadProblem_thread,NULL);
        succeed = arguments->succeed;
        if (arguments->excp != NULL) {
            qDebug() << arguments->excp->what() << endl;

            mainWindow->setDisabled(true);
            QMessageBox msgBox(mainWindow);
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.setWindowTitle("Error encountered");
            msgBox.setText("The problem couldn't be loaded");
            msgBox.setInformativeText(arguments->excp->what());
            if (strcmp(arguments->excp->more(),"") != 0) {
                msgBox.setDetailedText(arguments->excp->more());
            }
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.setDefaultButton(QMessageBox::Ok);
            msgBox.exec();
            mainWindow->setDisabled(false);

            delete arguments->excp;
        }

        delete loadProblem_thread;
        delete mutexattr;
        delete progressDialog;
        delete arguments->doc;
        delete arguments->progress->linksLoaded;
        delete arguments->progress->mutex;
        delete arguments->progress;
        delete arguments;
    }

    if (!succeed) {
        appState = INITIAL;
        delete _problem;
        return false;
    }

    mainWindow->addToProblemTree(_problem->wSpace());

    mainWindow->addViewerTab("WSpace", ((IVWorkSpace*)_problem->wSpace())->getIvScene());
    QColor color = settings->value("mainWindow/WSpace/color",QColor("black")).value<QColor>();
    mainWindow->getViewerTab("WSpace")->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));

    mainWindow->addViewerTab("CollisionWSpace", ((IVWorkSpace*)_problem->wSpace())->getCollisionIvScene());
    color = settings->value("mainWindow/CollisionWSpace/color",QColor("black")).value<QColor>();
    mainWindow->getViewerTab("CollisionWSpace")->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));

    //Used to show the IV models reconstructed from the Collision Checking model.
    //mainWindow->addViewerTab("SceneFromColl", ((IVWorkSpace*)_problem->wSpace())->getSceneFromColl());

    vector <DOFWidget*> robDOFWidgets;
    robDOFWidgets.resize(_problem->wSpace()->getNumRobots());
    for(uint i = 0; i < _problem->wSpace()->getNumRobots(); i++) {

        if(_problem->wSpace()->getRobot(i)->getCkine() != NULL)
            mainWindow->addConstrainedControlWidget(_problem->wSpace()->getRobot(i), _problem);

        robDOFWidgets[i] = mainWindow->addDOFWidget(_problem->wSpace()->getRobot(i));

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

    mainWindow->setSampleWidget(_problem);

    if( _problem->getPlanner() != NULL ){
        mainWindow->addPlanner(_problem->getPlanner(), _problem->getSampleSet());
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

    (void)argc;//unused
    try {
        QWidget *app = SoQt::init(argv[0]);//argc, argv,argv[0]);
        app->setVisible(false);
        Application kauthApp;
        SoQt::mainLoop();
    } catch(...) {
        qDebug() << "Unexpected error in Kautham execution" << endl;
    }

    return 0;
}



