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


#ifndef UI_ROBOTSIM_H
#define UI_ROBOTSIM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>

#include "problemtreewidget.h"
#include "attachobjectdialog.h"

QT_BEGIN_NAMESPACE

class Ui_kauthamMain
{
public:
    QAction         *actionNew;
    QAction         *actionAbout;
    QAction         *actionPlannersParameters;
    QWidget         *centralwidget;
    QWidget         *probTab;
    QWidget         *introTab;
    QWidget         *dockWidgetContents;
    QGridLayout     *gridLayout;
    QGridLayout     *gridLayout1;
    QGridLayout     *gridLayout2;
    QGridLayout     *gridLayout_2;
    QSplitter       *splitter;
    QTabWidget      *propertiesTab;
    QTabWidget      *DOFsTab;
    QTabWidget      *viewsTab;
    ProblemTreeWidget     *problemTree;
    QTextBrowser    *textBrowser;
    QMenuBar        *menubar;
    QMenu           *menuActions;
    QMenu           *menuFile;
    QMenu           *menuRecentFiles;
    QMenu           *menuHelp;
    QStatusBar      *statusbar;
    QToolBar        *toolBar;
    QToolBar        *planToolBar;
    QDockWidget     *outputWindow;
    QTextEdit       *textEdit;
    AttachObjectDialog *attachObjectDialog;

    void setupUi(QMainWindow *kauthamMain)
    {
        if (kauthamMain->objectName().isEmpty())
            kauthamMain->setObjectName(QString::fromUtf8("kauthamMain"));
        kauthamMain->setWindowModality(Qt::ApplicationModal);
        kauthamMain->resize(850, 759);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(kauthamMain->sizePolicy().hasHeightForWidth());
        kauthamMain->setSizePolicy(sizePolicy);
        //kauthamMain->setMinimumSize(QSize(850, 600));
        QIcon icon(QString::fromUtf8(":/icons/logo.svg"));
        kauthamMain->setWindowIcon(icon);
        kauthamMain->setToolButtonStyle(Qt::ToolButtonIconOnly);
        kauthamMain->setDockNestingEnabled(false);
        actionNew = new QAction(kauthamMain);
        actionNew->setObjectName(QString::fromUtf8("actionNew"));
        actionAbout = new QAction(kauthamMain);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionPlannersParameters = new QAction(kauthamMain);
        actionPlannersParameters->setObjectName(QString::fromUtf8("actionPlannersParameters"));
        centralwidget = new QWidget(kauthamMain);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        splitter = new QSplitter(centralwidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        //splitter->setMinimumSize(QSize(600, 500));
        splitter->setOrientation(Qt::Horizontal);
        propertiesTab = new QTabWidget(splitter);
        propertiesTab->setObjectName(QString::fromUtf8("propertiesTab"));
        propertiesTab->setMaximumSize(QSize(350, 16777215));
        propertiesTab->setTabPosition(QTabWidget::West);
        propertiesTab->setTabShape(QTabWidget::Rounded);
        probTab = new QWidget();
        probTab->setObjectName(QString::fromUtf8("probTab"));
        gridLayout = new QGridLayout(probTab);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        problemTree = new ProblemTreeWidget(Qt::Vertical);
        problemTree->setObjectName(QString::fromUtf8("problemTree"));
        gridLayout->addWidget(problemTree, 0, 0, 1, 1);
        propertiesTab->addTab(probTab, QString());
        propertiesTab->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Expanding);
        splitter->addWidget(propertiesTab);
        viewsTab = new QTabWidget(splitter);
        viewsTab->setObjectName(QString::fromUtf8("viewsTab"));
        viewsTab->setEnabled(true);
        //viewsTab->setMinimumSize(QSize(600, 4));
        introTab = new QWidget();
        introTab->setObjectName(QString::fromUtf8("introTab"));
        introTab->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(introTab->sizePolicy().hasHeightForWidth());
        introTab->setSizePolicy(sizePolicy1);
        gridLayout1 = new QGridLayout(introTab);
        gridLayout1->setObjectName(QString::fromUtf8("gridLayout1"));
        textBrowser = new QTextBrowser(introTab);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setSource(QUrl(""));
        textBrowser->setOpenExternalLinks(true);
        gridLayout1->addWidget(textBrowser, 0, 0, 1, 1);
        viewsTab->addTab(introTab, QString());
        splitter->addWidget(viewsTab);
        gridLayout_2->addWidget(splitter, 0, 0, 1, 1);
        DOFsTab = new QTabWidget(splitter);
        DOFsTab->setObjectName(QString::fromUtf8("DOFsTab"));
        DOFsTab->setMaximumSize(QSize(240, 16777215));
        //DOFsTab->setMinimumSize(QSize(200, 69));
        DOFsTab->setTabPosition(QTabWidget::East);
        DOFsTab->setTabShape(QTabWidget::Rounded);
        DOFsTab->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Expanding);
        splitter->addWidget(DOFsTab);
        kauthamMain->setCentralWidget(centralwidget);
        menubar = new QMenuBar(kauthamMain);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 850, 21));
        menuActions = new QMenu(menubar);
        menuActions->setObjectName(QString::fromUtf8("menuActions"));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        kauthamMain->setMenuBar(menubar);
        statusbar = new QStatusBar(kauthamMain);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        kauthamMain->setStatusBar(statusbar);
        toolBar = new QToolBar(kauthamMain);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setOrientation(Qt::Horizontal);
        toolBar->setIconSize(QSize(22,22));
        kauthamMain->addToolBar(Qt::TopToolBarArea, toolBar);
        outputWindow = new QDockWidget(kauthamMain);
        outputWindow->setObjectName(QString::fromUtf8("outputWindow"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        //sizePolicy2.setHeightForWidth(outputWindow->sizePolicy().hasHeightForWidth());
        //outputWindow->setSizePolicy(sizePolicy2);
        outputWindow->setMinimumSize(QSize(0, 0));
        outputWindow->setMaximumSize(QSize(16777215,16777215));
        //outputWindow->setBaseSize(QSize(122, 69));
        outputWindow->setAutoFillBackground(false);
        outputWindow->setFloating(false);
        outputWindow->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable|QDockWidget::DockWidgetVerticalTitleBar);
        outputWindow->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setMinimumSize(QSize(0,0));
        dockWidgetContents->setMaximumSize(QSize(16777215,16777215));
        gridLayout2 = new QGridLayout(dockWidgetContents);
        gridLayout2->setObjectName(QString::fromUtf8("gridLayout2"));
        textEdit = new QTextEdit(dockWidgetContents);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        sizePolicy2.setHeightForWidth(textEdit->sizePolicy().hasHeightForWidth());
        //textEdit->setSizePolicy(sizePolicy2);
        textEdit->setMinimumSize(QSize(0,0));
        textEdit->setMaximumSize(QSize(16777215,16777215));
        gridLayout2->addWidget(textEdit, 0, 0, 1, 1);
        outputWindow->setWidget(dockWidgetContents);
        kauthamMain->addDockWidget(static_cast<Qt::DockWidgetArea>(8), outputWindow);
        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuActions->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuHelp->addSeparator();
        menuHelp->addAction(actionAbout);
        menuHelp->addAction(actionPlannersParameters);
        retranslateUi(kauthamMain);
        propertiesTab->setCurrentIndex(0);
        viewsTab->setCurrentIndex(0);
        DOFsTab->setCurrentIndex(0);
        QMetaObject::connectSlotsByName(kauthamMain);
    } // setupUi

    void retranslateUi(QMainWindow *kauthamMain)
    {
        kauthamMain->setWindowTitle(QApplication::translate("kauthamMain", "Kautham 2.0 - Institute of Industrial and Control Engineering - Technical University of Catalonia", 0));
        actionNew->setText(QApplication::translate("kauthamMain", "New...", 0));
        actionAbout->setText(QApplication::translate("kauthamMain", "About...", 0));
        actionPlannersParameters->setText(QApplication::translate("kauthamMain", "Planner Parameters...", 0));
#ifndef QT_NO_ACCESSIBILITY
        propertiesTab->setAccessibleName(QApplication::translate("kauthamMain", "l", 0));
#endif // QT_NO_ACCESSIBILITY
        propertiesTab->setTabText(propertiesTab->indexOf(probTab), QApplication::translate("kauthamMain", "Problem", 0));
        textBrowser->setHtml(QApplication::translate("kauthamMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"></p></body></html>", 0));
        viewsTab->setTabText(viewsTab->indexOf(introTab), QApplication::translate("kauthamMain", "Introduction", 0));
        menuActions->setTitle(QApplication::translate("kauthamMain", "&Actions", 0));
        menuFile->setTitle(QApplication::translate("kauthamMain", "&File", 0));
        menuHelp->setTitle(QApplication::translate("kauthamMain", "&Help", 0));
#ifndef QT_NO_TOOLTIP
        outputWindow->setToolTip(QApplication::translate("kauthamMain", "Output Window", 0));
#endif // QT_NO_TOOLTIP
        outputWindow->setWindowTitle(QApplication::translate("kauthamMain", "Output", 0));
    } // retranslateUi

};

namespace Ui {
    class kauthamMain: public Ui_kauthamMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTSIM_H
