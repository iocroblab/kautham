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
    DefaultPathDialog::DefaultPathDialog(QStringList pathList, QWidget *parent) {
        defaultPathDialog = new QDialog(parent);
        defaultPathDialog->setWindowTitle("Choose the directories of the models");
        defaultPathDialog->setModal(true);
        defaultPathDialog->setObjectName(QString::fromUtf8("defaultPathDialog"));

        mainLayout = new QVBoxLayout(defaultPathDialog);
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));

        topLayout = new QHBoxLayout();
        topLayout->setObjectName(QString::fromUtf8("topLayout"));
        mainLayout->addLayout(topLayout);

        pathListWidget = new QListWidget();
        pathListWidget->setObjectName(QString::fromUtf8("pathListWidget"));
        pathListWidget->addItems(pathList);
        topLayout->addWidget(pathListWidget);

        buttonFrame = new QFrame();
        buttonFrame->setObjectName(QString::fromUtf8("buttonFrame"));
        buttonFrame->setMaximumWidth(100);
        buttonFrame->setMinimumWidth(100);
        buttonFrame->setContentsMargins(QMargins(-1,-1,-1,-1));
        buttonFrame->setFrameStyle(QFrame::NoFrame);
        topLayout->addWidget(buttonFrame);

        buttonLayout = new QVBoxLayout(buttonFrame);
        buttonLayout->setObjectName(QString::fromUtf8("buttonLayout"));
        buttonLayout->setMargin(0);

        addButton = new QPushButton(QIcon(":/icons/add.png"),tr("&Add"));
        addButton->setObjectName(QString::fromUtf8("addButton"));
        connect(addButton,SIGNAL(clicked()),this,SLOT(addDirectory()));
        buttonLayout->addWidget(addButton);

        removeButton = new QPushButton(QIcon(":/icons/remove.png"),tr("&Remove"));
        removeButton->setObjectName(QString::fromUtf8("removeButton"));
        connect(removeButton,SIGNAL(clicked()),this,SLOT(removeDirectory()));
        buttonLayout->addWidget(removeButton);

        clearButton = new QPushButton(QIcon(":/icons/clear.png"),tr("C&lear"));
        clearButton->setObjectName(QString::fromUtf8("clearButton"));
        connect(clearButton,SIGNAL(clicked()),pathListWidget,SLOT(clear()));
        buttonLayout->addWidget(clearButton);

        VSpacer = new QSpacerItem(20,40,QSizePolicy::Expanding,
                                        QSizePolicy::Expanding);
        buttonLayout->addSpacerItem(VSpacer);

        upButton = new QPushButton(QIcon(":/icons/up.png"),tr("&Up"));
        addButton->setObjectName(QString::fromUtf8("upButton"));
        connect(upButton,SIGNAL(clicked()),this,SLOT(upDirectory()));
        buttonLayout->addWidget(upButton);

        downButton = new QPushButton(QIcon(":/icons/down.png"),tr("&Down"));
        downButton->setObjectName(QString::fromUtf8("downButton"));
        connect(downButton,SIGNAL(clicked()),this,SLOT(downDirectory()));
        buttonLayout->addWidget(downButton);

        buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok  | QDialogButtonBox::Cancel, Qt::Horizontal);
        connect(buttonBox, SIGNAL(accepted()), defaultPathDialog, SLOT(accept()));
        connect(buttonBox, SIGNAL(rejected()), defaultPathDialog, SLOT(reject()));
        mainLayout->addWidget(buttonBox);
    }

    bool DefaultPathDialog::exec(QStringList *pathList) {
        if (defaultPathDialog->exec()) {
            pathList->clear();
            for (uint i = 0; i < pathListWidget->count(); i ++) {
                pathList->push_back(pathListWidget->item(i)->text());
            }
            return true;
        } else {
            return false;
        }
    }

    void DefaultPathDialog::addDirectory() {
        QDir workDir;
        QString dir;
        dir = QFileDialog::getExistingDirectory(defaultPathDialog,"Choose the default models directory",
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
