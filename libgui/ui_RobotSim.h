/********************************************************************************
** Form generated from reading ui file 'RobotSim.ui'
**
** Created: Fri 12. Jun 01:05:19 2009
**      by: Qt User Interface Compiler version 4.3.0
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_ROBOTSIM_H
#define UI_ROBOTSIM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QTreeWidget>
#include <QtGui/QWidget>
#include <QMessageBox>

class Ui_kauthamMain
{
public:
    QAction*      actionNew;
    QAction*      actionHelp;
    QAction*      actionAbout;
    QWidget*      centralwidget;
    QGridLayout*  gridLayout;
    QSplitter*    splitter_2;
    QSplitter*    splitter;
    QTabWidget*   propertiesTab;
    QWidget*      probTab;
    QGridLayout*  gridLayout1;
    QTreeWidget*  problemTree;
    QTabWidget*   viewsTab;
    QWidget*      introTab;
    QGridLayout*  gridLayout2;
    QTextBrowser* textBrowser;
    QTextEdit*    textEdit;
    QMenuBar*     menubar;
    QMenu*        menuActions;
    QMenu*        menuFile;
    QMenu*        menuHelp;
    QStatusBar*   statusbar;
    QToolBar*     toolBar;
    QToolBar*     planToolBar;

    void setupUi(QMainWindow *kauthamMain)
    {
    if (kauthamMain->objectName().isEmpty())
        kauthamMain->setObjectName(QString::fromUtf8("kauthamMain"));
    kauthamMain->setWindowModality(Qt::ApplicationModal);
    QRect available_geom = QDesktopWidget().availableGeometry();
    QSize size(available_geom.width(),available_geom.height());//850, 615);
    size = size.expandedTo(kauthamMain->minimumSizeHint());
    kauthamMain->resize(size);
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(kauthamMain->sizePolicy().hasHeightForWidth());
    kauthamMain->setSizePolicy(sizePolicy);
    kauthamMain->setMinimumSize(QSize(1000,700));//850, 600));
    kauthamMain->setWindowIcon(QIcon(QString::fromUtf8(":/icons/kautham.xpm")));
    kauthamMain->setToolButtonStyle(Qt::ToolButtonIconOnly);
    kauthamMain->setDockNestingEnabled(false);
    actionNew = new QAction(kauthamMain);
    actionNew->setObjectName(QString::fromUtf8("actionNew"));
    actionHelp = new QAction(kauthamMain);
    actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
    actionAbout = new QAction(kauthamMain);
    actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
    centralwidget = new QWidget(kauthamMain);
    centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
    gridLayout = new QGridLayout(centralwidget);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    splitter_2 = new QSplitter(centralwidget);
    splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
    splitter_2->setOrientation(Qt::Vertical);
    splitter = new QSplitter(splitter_2);
    splitter->setObjectName(QString::fromUtf8("splitter"));
    splitter->setOrientation(Qt::Horizontal);
    propertiesTab = new QTabWidget(splitter);
    propertiesTab->setObjectName(QString::fromUtf8("propertiesTab"));
    propertiesTab->setMaximumSize(QSize(250, 16777215));
    propertiesTab->setTabPosition(QTabWidget::West);
    propertiesTab->setTabShape(QTabWidget::Rounded);
    probTab = new QWidget();
    probTab->setObjectName(QString::fromUtf8("probTab"));
    gridLayout1 = new QGridLayout(probTab);
    gridLayout1->setObjectName(QString::fromUtf8("gridLayout1"));
    problemTree = new QTreeWidget(probTab);
    problemTree->setObjectName(QString::fromUtf8("problemTree"));

    gridLayout1->addWidget(problemTree, 0, 0, 1, 1);

    propertiesTab->addTab(probTab, QString());
    splitter->addWidget(propertiesTab);
    viewsTab = new QTabWidget(splitter);
    viewsTab->setObjectName(QString::fromUtf8("viewsTab"));
    viewsTab->setEnabled(true);
    viewsTab->setMinimumSize(QSize(600, 4));
    introTab = new QWidget();
    introTab->setObjectName(QString::fromUtf8("introTab"));
    introTab->setEnabled(true);
    QSizePolicy sizePolicy1(QSizePolicy::Maximum, QSizePolicy::Maximum);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(introTab->sizePolicy().hasHeightForWidth());
    introTab->setSizePolicy(sizePolicy1);
    gridLayout2 = new QGridLayout(introTab);
    gridLayout2->setObjectName(QString::fromUtf8("gridLayout2"));
    textBrowser = new QTextBrowser(introTab);
    textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
    textBrowser->setSource(QUrl(""));
    textBrowser->setOpenExternalLinks(true);

    gridLayout2->addWidget(textBrowser, 0, 0, 1, 1);

    viewsTab->addTab(introTab, QString());
    splitter->addWidget(viewsTab);
    splitter_2->addWidget(splitter);
    textEdit = new QTextEdit(splitter_2);
    textEdit->setObjectName(QString::fromUtf8("textEdit"));
    QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
    sizePolicy2.setHorizontalStretch(0);
    sizePolicy2.setVerticalStretch(0);
    sizePolicy2.setHeightForWidth(textEdit->sizePolicy().hasHeightForWidth());
    textEdit->setSizePolicy(sizePolicy2);
    textEdit->setMinimumSize(QSize(0, 60));
    textEdit->setMaximumSize(QSize(16777215, 100));
    splitter_2->addWidget(textEdit);

    gridLayout->addWidget(splitter_2, 0, 0, 1, 1);

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
    kauthamMain->addToolBar(Qt::TopToolBarArea, toolBar);
    QWidget::setTabOrder(viewsTab, textEdit);

    menubar->addAction(menuFile->menuAction());
    menubar->addAction(menuActions->menuAction());
    menubar->addAction(menuHelp->menuAction());
    menuHelp->addAction(actionHelp);
    menuHelp->addSeparator();
    menuHelp->addAction(actionAbout);

    retranslateUi(kauthamMain);

    propertiesTab->setCurrentIndex(0);
    viewsTab->setCurrentIndex(0);

    planToolBar = NULL;


    QMetaObject::connectSlotsByName(kauthamMain);
    } // setupUi

    void retranslateUi(QMainWindow *kauthamMain)
    {
    kauthamMain->setWindowTitle(QApplication::translate("kauthamMain", "Kautham 2.0 - Institute of Industrial and Control Engineering - Technical University of Catalonia", 0, QApplication::UnicodeUTF8));
    actionNew->setText(QApplication::translate("kauthamMain", "New...", 0, QApplication::UnicodeUTF8));
    actionHelp->setText(QApplication::translate("kauthamMain", "Kautham API Reference ", 0, QApplication::UnicodeUTF8));
    actionAbout->setText(QApplication::translate("kauthamMain", "About Kautham", 0, QApplication::UnicodeUTF8));
    propertiesTab->setAccessibleName(QApplication::translate("kauthamMain", "l", 0, QApplication::UnicodeUTF8));
    problemTree->headerItem()->setText(0, QApplication::translate("kauthamMain", "Attributes", 0, QApplication::UnicodeUTF8));
    problemTree->headerItem()->setText(1, QApplication::translate("kauthamMain", "Values", 0, QApplication::UnicodeUTF8));
    propertiesTab->setTabText(propertiesTab->indexOf(probTab), QApplication::translate("kauthamMain", "Problem", 0, QApplication::UnicodeUTF8));
    textBrowser->setHtml(QApplication::translate("kauthamMain", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
    viewsTab->setTabText(viewsTab->indexOf(introTab), QApplication::translate("kauthamMain", "Introduction", 0, QApplication::UnicodeUTF8));
    menuActions->setTitle(QApplication::translate("kauthamMain", "Actions", 0, QApplication::UnicodeUTF8));
    menuFile->setTitle(QApplication::translate("kauthamMain", "File", 0, QApplication::UnicodeUTF8));
    menuHelp->setTitle(QApplication::translate("kauthamMain", "Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class kauthamMain: public Ui_kauthamMain {};
} // namespace Ui

#endif // UI_ROBOTSIM_H
