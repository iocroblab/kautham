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

#include <Inventor/nodes/SoSeparator.h>
#include "gui.h"
#include "aboutwidget.h"
#include "plannerparameters.h"
#include "dofwidget.h"
#include "sampleswidget.h"
#include "plannerwidget.h"
#include "controlwidget.h"
#include "constrainedcontrolwidget.h"
#include "invkinwidget.h"
#include <QtGui>
#include <kautham/sampling/sampling.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLNodes.h>
#include <pugixml.hpp>

//The following config header file contains the version number of kautham and is automatically generated from the
//CMakeList.txt file using the configure file option
#include <kautham/kauthamConfig.h>


//Definitions to specify scene2VRML function behaviour
//#define VRML
//#define VRML2


using namespace pugi;

namespace Kautham {
    //GUI::GUI(QWidget *p) {
    GUI::GUI() {
        setupUi(this);
        problemTree->setEnabled(true);
        attachObjectDialog = new AttachObjectDialog(this);
        connect(attachObjectDialog,SIGNAL(sendText(string)),this,SLOT(setText(string)));
        QString f(":/kautham.html");
        if (QFile::exists(f)){
            QFile file(f);
            if (file.open(QFile::ReadOnly)){
                QByteArray data = file.readAll();
                QTextCodec *codec = Qt::codecForHtml(data);
                QString str = codec->toUnicode(data);
                textBrowser->setHtml(str);
            }
        }
        connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
        connect(actionPlannersParameters, SIGNAL(triggered()), this, SLOT(plannersparameters()));
        connect(outputWindow, SIGNAL(dockLocationChanged (Qt::DockWidgetArea)), this, SLOT(changeDockAreaForOutput(Qt::DockWidgetArea)));
        boolPlanVis = false;
        planToolBar = NULL;
        menuRecentFiles = NULL;
        restart();
    }

    void GUI::about(){
        setDisabled(true);
        AboutWidget tmp(this);
        tmp.setModal(true);
        tmp.setVisible(true);
        tmp.exec();
        setDisabled(false);
    }


    void GUI::plannersparameters(){
        setDisabled(true);
        PlannerParametersWidget tmp(this);
        tmp.setModal(true);
        tmp.setVisible(true);
        tmp.exec();
        setDisabled(false);
    }

    void GUI::changeDockAreaForOutput(Qt::DockWidgetArea area){
        if(area == Qt::LeftDockWidgetArea || area == Qt::RightDockWidgetArea)
            outputWindow->setFeatures(QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable);
        else
            outputWindow->setFeatures(QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable |
                                      QDockWidget::DockWidgetVerticalTitleBar);

    }

    void GUI::changeActiveBackground(){
        try{
            SoQtExaminerViewer* window = ((Viewer)viewers.at(viewsTab->currentIndex())).window;
            SbColor initial = window->getBackgroundColor();
            QColor color = QColorDialog::getColor(QColor((int)(255.0*initial.getValue()[0]),
                                                         (int)(255.0*initial.getValue()[1]),
                                                         (int)(255.0*initial.getValue()[2])), this);
            if (color.isValid()) {
                window->setBackgroundColor(SbColor(color.redF(),color.greenF(),color.blueF()));
            }
        }catch(...){
        }
    }

    QString GUI::getFilePath() {
        QSettings settings("IOC","Kautham");
        QDir workDir;
        QString last_path = settings.value("last_path",workDir.absolutePath()).toString();
        QString filePath = QFileDialog::getSaveFileName(this,
                                                        "Save scene as ...", last_path,
                                                        "Inventor file (*.iv)");
        if (!filePath.isEmpty()) {
            uint pointIndex = filePath.lastIndexOf(".");
            uint slashIndex = filePath.lastIndexOf("/");
            if (pointIndex > slashIndex) filePath.truncate(pointIndex);
            filePath.append(".iv");
        }

        return filePath;
    }

    static char * buffer;
    static size_t buffer_size = 0;

    static void *buffer_realloc(void * bufptr, size_t size) {
        buffer = (char *)realloc(bufptr, size);
        buffer_size = size;
        return buffer;
    }

    static SbString buffer_writeaction(SoNode * root) {
        SoOutput out;
        buffer = (char *)malloc(1024);
        buffer_size = 1024;
        out.setBuffer(buffer, buffer_size, buffer_realloc);

        SoWriteAction wa(&out);
        wa.apply(root);

        SbString s(buffer);
        free(buffer);
        return s;
    }

    void GUI::scene2VRML(){
        try {
            setCursor(QCursor(Qt::WaitCursor));
            QString filePath = getFilePath();
            if (!filePath.isEmpty()) {
                try {
                    SoSeparator* root = ((Viewer)viewers.at(viewsTab->currentIndex())).root;
                    root->ref();
#ifdef VRML
                    SoToVRMLAction wrlAction;
                    wrlAction.apply(root);
                    SoNode *wrl2 = wrlAction.getVRMLSceneGraph();
                    wrl2->ref();
                    SoOutput out;
                    if (out.openFile(filePath.toStdString().c_str())) {
                        out.setBinary(FALSE);
                        SoWriteAction writeAction(&out);
                        writeAction.apply(wrl2);
                        out.closeFile();
                        cout << "Saved scene to " << filePath.toStdString() <<  endl;
                    } else {
                        cout << "File " << filePath.toStdString() << " couldn't be opened" << endl;
                    }
                    wrl2->unref();
#else
#ifdef VRML2
                    SoToVRML2Action wrlAction;
                    wrlAction.apply(root);
                    SoVRMLGroup *wrl2 = wrlAction.getVRML2SceneGraph();
                    wrl2->ref();

                    SoOutput out;
                    if (out.openFile(filePath.toStdString().c_str())) {
                        out.setBinary(FALSE);
                        SoWriteAction writeAction(&out);
                        writeAction.apply(wrl2);
                        out.closeFile();
                        cout << "Saved scene to " << filePath.toStdString() <<  endl;
                    } else {
                        cout << "File " << filePath.toStdString() << " couldn't be opened" << endl;
                    }
                    wrl2->unref();
#else
                    QFile file(filePath);
                    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                        SbString s = buffer_writeaction(root);
                        QTextStream out(&file);
                        out << s.getString();
                        setText("File was saved successfully");
                    } else {
                        setText("Sorry but the file couldn't be saved");
                    }
                    file.close();
#endif
#endif
                    root->unref();
                } catch (const exception& excp) {
                    cout << "Error: " << excp.what() << endl;
                }
            }
            setCursor(QCursor(Qt::ArrowCursor));
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
        }
    }

    void GUI::attachObject() {
        attachObjectDialog->exec();
    }

    void GUI::toogleBBOXflag() {
        QSettings settings("IOC","Kautham");
        bool use_BBOX = settings.value("use_BBOX","false").toBool();
        settings.setValue("use_BBOX",!use_BBOX);
        QList<QAction*> actions = menuActions->actions();
        int i = 0;
        if (use_BBOX) {
            setText("Bounding boxes computation was disabled");

            while (actions.at(i)->text() != "BBOX enabled" && i < actions.size()) {
                i++;
            }
            if (i < actions.size()) {
                actions.at(i)->setText("BBOX disabled");
                QIcon greybox;
                greybox.addFile(":/icons/greybox_16x16.png");
                greybox.addFile(":/icons/greybox_22x22.png");
                greybox.addFile(":/icons/greybox_32x32.png");
                greybox.addFile(":/icons/greybox_48x48.png");
                greybox.addFile(":/icons/greybox_64x64.png");
                actions.at(i)->setIcon(greybox);
            }
        } else {
            setText("Bounding boxes computation was enabled");

            while (actions.at(i)->text() != "BBOX disabled" && i < actions.size()) {
                i++;
            }
            if (i < actions.size()) {
                actions.at(i)->setText("BBOX enabled");
                QIcon box;
                box.addFile(":/icons/box_16x16.png");
                box.addFile(":/icons/box_22x22.png");
                box.addFile(":/icons/box_32x32.png");
                box.addFile(":/icons/box_48x48.png");
                box.addFile(":/icons/box_64x64.png");
                actions.at(i)->setIcon(box);
            }
        }

    }


    void GUI::setModelsDefaultPath() {
        QSettings settings("IOC","Kautham");
        DefaultPathDialog *defaultPathDialog = new DefaultPathDialog(settings.value
                                                                     ("models_directories",
                                                                      QStringList()).
                                                                     toStringList(),this);
        QStringList *pathList = defaultPathDialog->getList();
        if (pathList != NULL) {
            settings.setValue("models_directories",*pathList);
            delete pathList;
        }
    }


    void GUI::clearText(){
        textEdit->clear();
    }

    void GUI::setText(string s){
        if(s.size()!=0){
            textEdit->append(QString(s.c_str()));
        }
    }

    bool GUI::setSampleWidget(Problem* problem){
        SamplesWidget* tmpSam = new SamplesWidget(problem);
        propertiesTab->addTab(tmpSam, "Samplers");
        connect(tmpSam, SIGNAL(sendText(string)), this, SLOT(setText(string)) );
        return true;
    }

    bool GUI::addRobControlWidget(Problem* prob, vector<DOFWidget*> robDOFWidgets){
        if( prob != NULL){
            ControlWidget* tmpControl = new ControlWidget(prob,robDOFWidgets,true);
            connect(tmpControl,SIGNAL(sendText(string)),this,SLOT(setText(string)));
            propertiesTab->addTab(tmpControl, "RobContr");
            //JAN
            indexRobControlsTab = propertiesTab->indexOf(tmpControl);
            return true;
        }else{
            ControlWidget* tmpControl = new ControlWidget(NULL,robDOFWidgets,true);
            connect(tmpControl,SIGNAL(sendText(string)),this,SLOT(setText(string)));
            propertiesTab->addTab(tmpControl, "RobContr-Test");
            indexRobControlsTab = propertiesTab->indexOf(tmpControl);
            return true;
        }
        return false;
    }

    bool GUI::addObsControlWidget(Problem* prob, vector<DOFWidget*> obsDOFWidgets){
        if( prob != NULL){
            ControlWidget* tmpControl = new ControlWidget(prob,obsDOFWidgets,false);
            connect(tmpControl,SIGNAL(sendText(string)),this,SLOT(setText(string)));
            propertiesTab->addTab(tmpControl, "ObsContr");
            //JAN
            indexObsControlsTab = propertiesTab->indexOf(tmpControl);
            return true;
        }else{
            ControlWidget* tmpControl = new ControlWidget(NULL,obsDOFWidgets,false);
            connect(tmpControl,SIGNAL(sendText(string)),this,SLOT(setText(string)));
            propertiesTab->addTab(tmpControl, "ObsContr-Test");
            indexObsControlsTab = propertiesTab->indexOf(tmpControl);
            return true;
        }
        return false;
    }

    ControlWidget* GUI::getRobControlWidget()
    {
        if (indexRobControlsTab < 0) {
            return (NULL);
        } else {
            return (ControlWidget*)propertiesTab->widget(indexRobControlsTab);
        }
    }

    ControlWidget* GUI::getObsControlWidget()
    {
        if (indexObsControlsTab < 0) {
            return (NULL);
        } else {
            return (ControlWidget*)propertiesTab->widget(indexObsControlsTab);
        }
    }

    PlannerWidget* GUI::getPlannerWidget()
    {
        return (PlannerWidget*)propertiesTab->widget(indexPlannerTab);
    }

    bool GUI::addConstrainedControlWidget( Robot* rob, Problem* prob){
        if( rob != NULL){
            ConstrainedControlWidget* tmpControl = new ConstrainedControlWidget( rob, prob);
            propertiesTab->addTab(tmpControl, "ConstrainedCtrl-" + QString((rob->getName()).c_str()));
            return true;
        }else{
            ConstrainedControlWidget* tmpControl = new ConstrainedControlWidget( NULL, NULL);
            propertiesTab->addTab(tmpControl, "ConstrainedControlTest");
            return true;
        }
        return false;
    }

    DOFWidget* GUI::addDOFWidget(Robot* rob) {
        if( rob != NULL){
            DOFWidget* tmpDOF = new DOFWidget(rob);
            string rob_name = rob->getName();
            tmpDOF->setToolTip(string(rob_name+"-DOF").c_str());
            if (rob_name.length() > 10) {
                rob_name = rob_name.substr(0,7)+"...";
            }
            DOFsTab->addTab(tmpDOF, string(rob_name+"-DOF").c_str());
            connect(tmpDOF, SIGNAL(sendText(string)), this, SLOT(setText(string)));
            return tmpDOF;
        }
        return NULL;
    }

    bool GUI::addInverseKinematic(InverseKinematic* ikine){
        InvKinWidget* tmpIkine = new InvKinWidget(ikine);
        string rob_name = ikine->getRobot().getName();
        if (rob_name.length() > 10) {
            rob_name = rob_name.substr(0,7)+"...";
        }
        propertiesTab->addTab(tmpIkine,string(rob_name+"-IK").c_str());
        connect(tmpIkine, SIGNAL(sendText(string)), this, SLOT(setText(string)));
        return true;
    }


    bool GUI::addWidgetToTab(QWidget* widget, QString name){
        if( widget != NULL){
            propertiesTab->addTab(widget, name);
            return true;
        }
        return false;
    }



    bool GUI::addPlanner(Planner *plan, SampleSet* samp){
        if (plan) {
            PlannerWidget* tmpPlan = new PlannerWidget( plan, samp, plan->hasCameraMovements());
            propertiesTab->addTab(tmpPlan, QString((plan->getGuiName()).c_str()));
            connect(tmpPlan,SIGNAL(sendText(string)),this,SLOT(setText(string)));
            connect(tmpPlan,SIGNAL(changeCursor(bool)),this,SLOT(changeCursor(bool)));
            connect(this,SIGNAL(stopSimulation()),tmpPlan,SLOT(stopSimulation()));
            //JAN
            indexPlannerTab = propertiesTab->indexOf(tmpPlan);
            SoSeparator *cspace(plan->getIvCspaceScene());
            if (cspace) {
                addViewerTab("CSpace",cspace);
            }
            SoSeparator *path(plan->getIvPathScene());
            if (path) {
                for (unsigned int i(0); i < viewers.size(); ++i) {
                    if (viewers.at(i).title == "WSpace" ||
                            viewers.at(i).title == "CollisionWSpace") {
                        viewers.at(i).root->addChild(path);
                    }
                }
            }
            return true;
        } else {
            PlannerWidget* tmpPlan = new PlannerWidget( NULL, NULL);
            propertiesTab->addTab(tmpPlan, "Planner");
            //JAN
            indexPlannerTab = propertiesTab->indexOf(tmpPlan);
            return true;
        }
        return false;
    }

    void GUI::changeCursor(bool waiting) {
        setCursor(QCursor(waiting?Qt::WaitCursor:Qt::ArrowCursor));
    }


    bool GUI::addViewerTab(string title, SoSeparator *root){
        viewsTab->setEnabled(true);
        viewsTab->setCurrentIndex(0);
        if (root && title.size()!=0) {
            Viewer v;
            v.tab = new QWidget(viewsTab);
            v.root= root;
            v.title =title;
            v.window= new SoQtExaminerViewer(v.tab);
            if (title != "CSpace") {
                v.window->setFeedbackVisibility(true);
            }
            viewsTab->addTab(v.tab,QString(title.c_str()));
            v.window->setViewing(FALSE);
            v.window->setSceneGraph(root);
            v.window->setBackgroundColor(SbColor(0.0f,0.0f,0.0f));
            //v.window->setPopupMenuEnabled(FALSE);
            v.window->show();
            viewers.push_back(v);
            viewsTab->setCurrentIndex(viewsTab->count()-1);
            return true;
        }
        return false;
    }

    //removes the viewer tab. The Introduction tab viewer cannot be eliminated.
    void GUI::removeViewerTab(string title){
        vector<Viewer>::iterator it_v;
        if( title != "Introduction")
            for(it_v = viewers.begin(); it_v != viewers.end(); it_v++)
                if((*it_v).title == title){
                    viewsTab->removeTab(viewsTab->indexOf( it_v->tab ));
                    delete it_v->window;
                    it_v->root->unref();
                    viewers.erase(it_v);
                    break;
                }
    }

    std::string GUI::getActiveViewTitle(){
        return viewsTab->tabText(viewsTab->currentIndex()).toUtf8().constData();
    }


    void GUI::removePropTab(string title){
        if( title.compare("Problem") == -1 )
            for(int i =0; i < propertiesTab->count(); i++)
                if(propertiesTab->tabText(i).compare(QString(title.c_str())) == 0){
                    QWidget* tmp = propertiesTab->widget( i );
                    delete tmp;
                    propertiesTab->removeTab( i );
                    break;
                }
    }

    /*!	This function return the root scene SoSeparator that show in tab which this title.
    */
    SoSeparator* GUI::getRootTab(string title){
        if(title.size()!=0){
            for(unsigned int i=0; i < viewers.size();i++){
                if(viewers[i].title == title)
                    return viewers[i].root ;
            }
        }
        return NULL;
    }

    const mt::Transform GUI::getActiveCameraTransfom(){
        //    if( typeid(*(viewsTab->currentWidget())) == typeid(SoQtExaminerViewer)){
        try{
            SoQtExaminerViewer* tmp = ((Viewer)viewers.at(viewsTab->currentIndex()-1)).window;
            SoCamera* tmpCam = tmp->getCamera();
            SbVec3f pos, axis;
            mt::Unit3 mtaxis;
            mt::Point3 mtpos;
            float   angle;

            //  Get Transformation matrix for the camera

            tmpCam->orientation.getValue(axis, angle);
            pos = tmpCam->position.getValue();
            pos.getValue(mtpos.at(0), mtpos.at(1), mtpos.at(2));
            axis.getValue(mtaxis.at(0), mtaxis.at(1), mtaxis.at(2));

            const mt::Rotation rot(mtaxis, angle);

            const mt::Transform ci2w(rot, mtpos);
            return ci2w;
        }catch(...){
            const mt::Transform ci2w;
            return ci2w;
        }
    }

    bool GUI::setActiveCameraRotation(float qx, float qy, float qz, float qw){
        try{
            SoQtExaminerViewer* tmp = ((Viewer)viewers.at(viewsTab->currentIndex()-1)).window;
            SoCamera* tmpCam = tmp->getCamera();
            tmpCam->orientation.setValue(qx, qy, qz, qw);
            tmp->viewAll();
            tmp->setFeedbackVisibility(true) ;
            return true;
        }catch(...){
            return false;
        }
    }

    bool GUI::setActiveCameraPosition(float x, float y, float z ){
        try{
            SoQtViewer* tmp = ((Viewer)viewers.at(viewsTab->currentIndex()-1)).window;
            SoCamera* tmpCam = tmp->getCamera();
            tmpCam->position.setValue(x, y, z);
            tmp->viewAll();
            return true;
        }catch(...){
            return false;
        }
    }

    bool GUI::setActiveCameraPointAt(float x, float y, float z ){
        try{
            SoQtViewer* tmp = ((Viewer)viewers.at(viewsTab->currentIndex()-1)).window;
            SoCamera* tmpCam = tmp->getCamera();
            SbVec3f pos;
            pos.setValue(x, y, z);
            tmpCam->pointAt(pos);
            tmp->viewAll();
            return true;
        }catch(...){
            return false;
        }
    }

    bool GUI::setActiveCameraTransform(mt::Transform tra){
        try{
            SoQtViewer* tmp = ((Viewer)viewers.at(viewsTab->currentIndex()-1)).window;
            SoCamera* tmpCam = tmp->getCamera();
            SbVec3f pos, axis;

            mt::Unit3 mtaxis;
            float   angle;

            const mt::Point3 mtpos = tra.getTranslation();
            const mt::Rotation rot = tra.getRotation();

            rot.getAxisAngle(mtaxis, angle);

            axis.setValue(mtaxis.at(0), mtaxis.at(1), mtaxis.at(2));

            tmpCam->position.setValue(mtpos.at(0), mtpos.at(1), mtpos.at(2));
            tmpCam->orientation.setValue(axis, angle);
            //tmp->viewAll();
            return true;
        }catch(...){
            return false;
        }
    }

    SoQtExaminerViewer* GUI::getViewerTab(string title){
        if(title.size()!=0){
            for(unsigned int i=0; i < viewers.size();i++){
                if(viewers[i].title == title)
                    return &(*(viewers.at(i).window) );
            }
        }
        return NULL;
    }

    bool GUI::addSeparator(WHERETYPE typ){
        switch(typ){
        case TOOLBAR:
            toolBar->addSeparator();
            break;
        case FILEMENU:
            menuFile->addSeparator();
            break;
        case ACTIONMENU:
            menuActions->addSeparator();
            break;
        case FILETOOL:
            menuFile->addSeparator();
            //toolBar->addSeparator();
            break;
        case ACTIONTOOL:
            menuActions->addSeparator();
            //toolBar->addSeparator();
            break;
        case RECENTFILESMENU:
            if (menuRecentFiles == NULL) {
                return false;
            } else {
                menuRecentFiles->addSeparator();
            }
            break;
        default:
            return false;
        }
        return true;
    }
    /*! This function adds an action where do you like in GUI and asociates it with the QObject
            and the Slot passed throw the parameters
    */
    bool GUI::setAction(WHERETYPE typ, string name, string shortcut, QIcon icon,
                        QObject *receiver, const char *member){
        if (name.size()!= 0 ) {
            QAction *ac;

            if (icon.isNull()) {
                ac = new QAction(tr(name.c_str()), this);
            } else {
                ac = new QAction(icon, tr(name.c_str()), this);
            }

            if (shortcut.size() != 0) ac->setShortcut(tr(shortcut.c_str()));

            connect(ac, SIGNAL(triggered()), receiver, member);

            return setAction(typ,ac);
        }

        return false;
    }

    /*! This function adds an action where do you like in GUI
    */
    bool GUI::setAction(WHERETYPE typ, QAction *ac){
        if(ac != NULL){
            switch(typ){
            case TOOLBAR:
                toolBar->addAction(ac);
                break;
            case FILEMENU:
                menuFile->addAction(ac);
                break;
            case ACTIONMENU:
                menuActions->addAction(ac);
                break;
            case FILETOOL:
                menuFile->addAction(ac);
                toolBar->addAction(ac);
                break;
            case ACTIONTOOL:
                menuActions->addAction(ac);
                toolBar->addAction(ac);
                break;
            case RECENTFILESMENU:
                if (menuRecentFiles == NULL) {
                    return false;
                } else {
                    menuRecentFiles->addAction(ac);
                }
            default:
                return false;
            }
            return true;
        }
        return false;
    }

    void GUI::setRecentFilesMenu() {
        if (menuRecentFiles == NULL) {
            menuRecentFiles = menuFile->addMenu("Recent &Files");
            menuRecentFiles->setObjectName("menuRecentFiles");
        } else {
            menuRecentFiles->clear();
        }
    }

    void GUI::showRecentFiles(bool visible) {
        menuRecentFiles->setEnabled(visible);
    }

    bool GUI::setToogleAction(WHERETYPE typ, string name, string shortcut, string iconame,
                              QObject *receiver, const char *member){
        QAction *ac;

        if(iconame.size()!=0 && name.size()!=0 ){
            ac = new QAction(QIcon(iconame.c_str()), tr(name.c_str()), this);
            ac->setCheckable(true);
            if(shortcut.size()!=0)
                ac->setShortcut(tr(shortcut.c_str()));

            connect(ac, SIGNAL(changed()), receiver, member);

            switch(typ){
            case TOOLBAR:
                toolBar->addAction(ac);
                break;
            case FILEMENU:
                menuFile->addAction(ac);
                break;
            case ACTIONMENU:
                menuActions->addAction(ac);
                break;
            case FILETOOL:
                menuFile->addAction(ac);
                toolBar->addAction(ac);
                break;
            case ACTIONTOOL:
                menuActions->addAction(ac);
                toolBar->addAction(ac);
                break;
            default:
                return false;
            }
            return true;
        }
        return false;

    }

    bool GUI::restart(){
        for(int i=viewers.size(); i>0 ;i--)
            removeViewerTab(viewers[0].title);

        for(int i=viewsTab->count(); i>0 ;i--)
            viewsTab->removeTab(0);

        for(int i=propertiesTab->count();i>0;i--)
            propertiesTab->removeTab(1);//tab 0 is never removed

        for(int i=DOFsTab->count();i>0;i--)
            DOFsTab->removeTab(0);

        showInitialAppearance();

        viewers.clear();
        textEdit->clear();
        stringstream tmp;
        tmp << "Kautham ";
        tmp << MAJOR_VERSION;
        tmp << ".";
        tmp << MINOR_VERSION;
        tmp << ".";
        tmp << PATCH_VERSION;
        tmp << " - Institute of Industrial and Control Engineering";
        tmp << " - Technical University of Catalonia";
        setWindowTitle( tmp.str().c_str() );

        indexRobControlsTab = -1;
        indexObsControlsTab = -1;

        return true;
    }

    bool GUI::addToProblemTree(WorkSpace *workSpace){
        attachObjectDialog->set(workSpace);
        return problemTree->setTree(workSpace);
    }

    void GUI::showProblemAppearance() {
        viewsTab->removeTab(viewsTab->indexOf(introTab));
        DOFsTab->show();
        propertiesTab->show();
        QList <QAction*> actions = toolBar->actions();
        QAction *ac;
        for (int i = 0; i < actions.size(); i++) {
            ac = actions.at(i);
            if (ac->text() == "Chan&ge Colour") {
                ac->setEnabled(true);
                QIcon colors;
                colors.addFile(":/icons/colors_16x16.png");
                colors.addFile(":/icons/colors_22x22.png");
                colors.addFile(":/icons/colors_32x32.png");
                colors.addFile(":/icons/colors_48x48.png");
                colors.addFile(":/icons/colors_64x64.png");
                ac->setIcon(colors);
            } else if (ac->text() == "At/Detach Object") {
                ac->setEnabled(true);
                QIcon magnet;
                magnet.addFile(":/icons/magnet_16x16.png");
                magnet.addFile(":/icons/magnet_22x22.png");
                magnet.addFile(":/icons/magnet_32x32.png");
                magnet.addFile(":/icons/magnet_48x48.png");
                magnet.addFile(":/icons/magnet_64x64.png");
                ac->setIcon(magnet);
            } else if (ac->text() == "Export scene") {
                ac->setEnabled(true);
                QIcon image;
                image.addFile(":/icons/image_16x16.png");
                image.addFile(":/icons/image_22x22.png");
                image.addFile(":/icons/image_32x32.png");
                image.addFile(":/icons/image_48x48.png");
                image.addFile(":/icons/image_64x64.png");
                ac->setIcon(image);
            }
        }
    }

    void GUI::showInitialAppearance() {
        viewsTab->addTab(introTab, QApplication::translate
                         ("kauthamMain","Introduction",0,QApplication::UnicodeUTF8));
        DOFsTab->hide();
        propertiesTab->hide();
        QList <QAction*> actions = toolBar->actions();
        QAction *ac;
        for (int i = 0; i < actions.size(); i++) {
            ac = actions.at(i);
            if (ac->text() == "Chan&ge Colour") {
                ac->setDisabled(true);
                QIcon greycolors;
                greycolors.addFile(":/icons/greycolors_16x16.png");
                greycolors.addFile(":/icons/greycolors_22x22.png");
                greycolors.addFile(":/icons/greycolors_32x32.png");
                greycolors.addFile(":/icons/greycolors_48x48.png");
                greycolors.addFile(":/icons/greycolors_64x64.png");
                ac->setIcon(greycolors);
            } else if (ac->text() == "At/Detach Object") {
                ac->setDisabled(true);
                QIcon greymagnet;
                greymagnet.addFile(":/icons/greymagnet_16x16.png");
                greymagnet.addFile(":/icons/greymagnet_22x22.png");
                greymagnet.addFile(":/icons/greymagnet_32x32.png");
                greymagnet.addFile(":/icons/greymagnet_48x48.png");
                greymagnet.addFile(":/icons/greymagnet_64x64.png");
                ac->setIcon(greymagnet);
            }  else if (ac->text() == "Export scene") {
                ac->setDisabled(true);
                QIcon image;
                image.addFile(":/icons/image_16x16.png");
                image.addFile(":/icons/image_22x22.png");
                image.addFile(":/icons/image_32x32.png");
                image.addFile(":/icons/image_48x48.png");
                image.addFile(":/icons/image_64x64.png");
                ac->setIcon(image);
            }

        }
    }

}
