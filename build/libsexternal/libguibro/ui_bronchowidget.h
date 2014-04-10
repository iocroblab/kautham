/********************************************************************************
** Form generated from reading UI file 'bronchowidget.ui'
**
** Created: Thu Apr 10 11:49:55 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BRONCHOWIDGET_H
#define UI_BRONCHOWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bronchoWidget
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QSlider *alphaSlider;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_2;
    QSlider *XiSlider;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_3;
    QSlider *DzSlider;
    QCheckBox *RateCheckBox;
    QCheckBox *CameraCheckBox;
    QPushButton *advanceButton;
    QPushButton *collisionCheckButton;

    void setupUi(QWidget *bronchoWidget)
    {
        if (bronchoWidget->objectName().isEmpty())
            bronchoWidget->setObjectName(QString::fromUtf8("bronchoWidget"));
        bronchoWidget->resize(407, 457);
        verticalLayout = new QVBoxLayout(bronchoWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(bronchoWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        verticalLayout_5 = new QVBoxLayout(groupBox);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        alphaSlider = new QSlider(groupBox);
        alphaSlider->setObjectName(QString::fromUtf8("alphaSlider"));
        alphaSlider->setMinimum(-1000);
        alphaSlider->setMaximum(1000);
        alphaSlider->setValue(0);
        alphaSlider->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(alphaSlider);


        verticalLayout_5->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_3->addWidget(label_2);

        XiSlider = new QSlider(groupBox);
        XiSlider->setObjectName(QString::fromUtf8("XiSlider"));
        XiSlider->setMinimum(-1000);
        XiSlider->setMaximum(1000);
        XiSlider->setValue(0);
        XiSlider->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(XiSlider);


        verticalLayout_5->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_4->addWidget(label_3);

        DzSlider = new QSlider(groupBox);
        DzSlider->setObjectName(QString::fromUtf8("DzSlider"));
        DzSlider->setMinimum(-100);
        DzSlider->setMaximum(100);
        DzSlider->setValue(0);
        DzSlider->setOrientation(Qt::Horizontal);

        verticalLayout_4->addWidget(DzSlider);


        verticalLayout_5->addLayout(verticalLayout_4);

        RateCheckBox = new QCheckBox(groupBox);
        RateCheckBox->setObjectName(QString::fromUtf8("RateCheckBox"));

        verticalLayout_5->addWidget(RateCheckBox);

        CameraCheckBox = new QCheckBox(groupBox);
        CameraCheckBox->setObjectName(QString::fromUtf8("CameraCheckBox"));

        verticalLayout_5->addWidget(CameraCheckBox);

        advanceButton = new QPushButton(groupBox);
        advanceButton->setObjectName(QString::fromUtf8("advanceButton"));
        advanceButton->setMaximumSize(QSize(160, 16777215));

        verticalLayout_5->addWidget(advanceButton);

        collisionCheckButton = new QPushButton(groupBox);
        collisionCheckButton->setObjectName(QString::fromUtf8("collisionCheckButton"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(collisionCheckButton->sizePolicy().hasHeightForWidth());
        collisionCheckButton->setSizePolicy(sizePolicy);
        collisionCheckButton->setMaximumSize(QSize(160, 16777215));

        verticalLayout_5->addWidget(collisionCheckButton);


        verticalLayout->addWidget(groupBox);


        retranslateUi(bronchoWidget);

        QMetaObject::connectSlotsByName(bronchoWidget);
    } // setupUi

    void retranslateUi(QWidget *bronchoWidget)
    {
        bronchoWidget->setWindowTitle(QApplication::translate("bronchoWidget", "bronchoWidget", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("bronchoWidget", "BronchoControls", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("bronchoWidget", "Alpha_Z", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("bronchoWidget", "Xi_Z", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("bronchoWidget", "DeltaZ", 0, QApplication::UnicodeUTF8));
        RateCheckBox->setText(QApplication::translate("bronchoWidget", "Rate Control", 0, QApplication::UnicodeUTF8));
        CameraCheckBox->setText(QApplication::translate("bronchoWidget", "Camera View", 0, QApplication::UnicodeUTF8));
        advanceButton->setText(QApplication::translate("bronchoWidget", "Advance To Best Configuration", 0, QApplication::UnicodeUTF8));
        collisionCheckButton->setText(QApplication::translate("bronchoWidget", "Grid-based Collision Check", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class bronchoWidget: public Ui_bronchoWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BRONCHOWIDGET_H
