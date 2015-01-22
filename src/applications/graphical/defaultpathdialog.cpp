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


#include "defaultpathdialog.h"


namespace Kautham {
    DefaultPathDialog::DefaultPathDialog(QStringList pathList, QWidget *parent,
                                         Qt::WindowFlags f):QDialog(parent, f) {
        setWindowTitle("Choose the directories of the models");
        setModal(true);
        setObjectName(QString::fromUtf8("defaultPathDialog"));

        QVBoxLayout *mainLayout = new QVBoxLayout();
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        setLayout(mainLayout);

        QHBoxLayout *topLayout = new QHBoxLayout();
        topLayout->setObjectName(QString::fromUtf8("topLayout"));
        mainLayout->addLayout(topLayout);

        pathListWidget = new QListWidget();
        pathListWidget->setObjectName(QString::fromUtf8("pathListWidget"));
        pathListWidget->addItems(pathList);
        topLayout->addWidget(pathListWidget);

        QFrame *buttonFrame = new QFrame();
        buttonFrame->setObjectName(QString::fromUtf8("buttonFrame"));
        buttonFrame->setMaximumWidth(100);
        buttonFrame->setMinimumWidth(100);
        buttonFrame->setContentsMargins(QMargins(-1,-1,-1,-1));
        buttonFrame->setFrameStyle(QFrame::NoFrame);
        topLayout->addWidget(buttonFrame);

        QVBoxLayout *buttonLayout = new QVBoxLayout(buttonFrame);
        buttonLayout->setObjectName(QString::fromUtf8("buttonLayout"));
        buttonLayout->setMargin(0);


        QIcon addIcon;
        addIcon.addFile(":/icons/add_16x16.png");
        addIcon.addFile(":/icons/add_22x22.png");
        QPushButton *button = new QPushButton(addIcon,tr("&Add"));
        button->setObjectName(QString::fromUtf8("addButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(addDirectory()));
        buttonLayout->addWidget(button);

        QIcon removeIcon;
        removeIcon.addFile(":/icons/remove_16x16.png");
        removeIcon.addFile(":/icons/remove_22x22.png");
        button = new QPushButton(removeIcon,tr("&Remove"));
        button->setObjectName(QString::fromUtf8("removeButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(removeDirectory()));
        buttonLayout->addWidget(button);

        QIcon clearIcon;
        clearIcon.addFile(":/icons/trashcan_16x16.png");
        clearIcon.addFile(":/icons/trashcan_22x22.png");
        clearIcon.addFile(":/icons/trashcan_32x32.png");
        clearIcon.addFile(":/icons/trashcan_48x48.png");
        clearIcon.addFile(":/icons/trashcan_64x64.png");
        button = new QPushButton(clearIcon,tr("C&lear"));
        button->setObjectName(QString::fromUtf8("clearButton"));
        connect(button,SIGNAL(clicked()),pathListWidget,SLOT(clear()));
        buttonLayout->addWidget(button);

        QSpacerItem *vSpacer = new QSpacerItem(20,40,QSizePolicy::Expanding,
                                        QSizePolicy::Expanding);
        buttonLayout->addSpacerItem(vSpacer);

        QIcon upIcon;
        upIcon.addFile(":/icons/up_16x16.png");
        upIcon.addFile(":/icons/up_22x22.png");
        button = new QPushButton(upIcon,tr("&Up"));
        button->setObjectName(QString::fromUtf8("upButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(upDirectory()));
        buttonLayout->addWidget(button);

        QIcon downIcon;
        downIcon.addFile(":/icons/down_16x16.png");
        downIcon.addFile(":/icons/down_22x22.png");
        button = new QPushButton(downIcon,tr("&Down"));
        button->setObjectName(QString::fromUtf8("downButton"));
        connect(button,SIGNAL(clicked()),this,SLOT(downDirectory()));
        buttonLayout->addWidget(button);

        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok |
                                                           QDialogButtonBox::Cancel,
                                                           Qt::Horizontal);
        connect(buttonBox,SIGNAL(accepted()),this,SLOT(accept()));
        connect(buttonBox,SIGNAL(rejected()),this,SLOT(reject()));
        mainLayout->addWidget(buttonBox);
    }


    QStringList *DefaultPathDialog::getList() {
        if (exec()) {
            QStringList *pathList = new QStringList();
            for (int i = 0; i < pathListWidget->count(); i ++) {
                pathList->push_back(pathListWidget->item(i)->text());
            }
            return pathList;
        } else {
            return NULL;
        }
    }


    void DefaultPathDialog::addDirectory() {
        QDir workDir;
        QString dir;
        dir = QFileDialog::getExistingDirectory(this,"Choose the default models directory",
                                                workDir.absolutePath(),QFileDialog::ShowDirsOnly
                                                | QFileDialog::DontResolveSymlinks);
        if (!dir.isEmpty()) {
            if (pathListWidget->findItems(dir,Qt::MatchExactly).size() == 0) {
                pathListWidget->addItem(dir);
                pathListWidget->setCurrentRow(pathListWidget->count()-1);
            }
        }
    }


    void DefaultPathDialog::removeDirectory() {
        QListWidgetItem *pathWidget = pathListWidget->takeItem(pathListWidget->currentRow());
        if (pathWidget != NULL) {
            delete pathWidget;
        }
    }


    void DefaultPathDialog::upDirectory() {
        int row = pathListWidget->currentRow();
        if (row > 0) {
            pathListWidget->insertItem(row-1, pathListWidget->takeItem(row));
            pathListWidget->setCurrentRow(row-1);
        }
    }


    void DefaultPathDialog::downDirectory() {
        int row = pathListWidget->currentRow();
        if (row < pathListWidget->count()-1) {
            pathListWidget->insertItem(row+1, pathListWidget->takeItem(row));
            pathListWidget->setCurrentRow(row+1);
        }
    }
}
