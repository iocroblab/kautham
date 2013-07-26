#include "properties_gui.hpp"

namespace libGUI{
Properties_GUI::Properties_GUI( TeleoperationWidget* gui ){
  setupUi(this);
  _gui = gui;

  _txtName->setText( QString(gui->getNodeName().c_str()) );
  txt_IpMaster->setText( QString(gui->getIPMaster().c_str()) );
  txt_IpNode->setText( QString(gui->getIPLocal().c_str()));
  txt_PubPeriod->setText( QString().setNum(gui->getFreqPubli()));
  

  QObject::connect(buttonBox, SIGNAL(accepted()), this, SLOT(accepted()));
  QObject::connect(buttonBox, SIGNAL(rejected()), this, SLOT(rejected())); 
}

void Properties_GUI::accepted(){
  _gui->setNodeName( _txtName->text().toUtf8().constData() );
  _gui->setFreqPubli( txt_PubPeriod->text().toInt() );
  _gui->setIPLocal( txt_IpNode->text().toUtf8().constData()  );
  _gui->setIPMaster( txt_IpMaster->text().toUtf8().constData()  );
  this->done(QDialog::Accepted);
}

void Properties_GUI::rejected(){
  this->done(QDialog::Rejected);
}

void Properties_GUI::setupUi(QDialog *win_properties){
  if (win_properties->objectName().isEmpty())
        win_properties->setObjectName(QString::fromUtf8("win_properties"));
    win_properties->resize(217, 196);
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(win_properties->sizePolicy().hasHeightForWidth());
    win_properties->setSizePolicy(sizePolicy);
    win_properties->setMinimumSize(QSize(0, 0));
    win_properties->setMaximumSize(QSize(1000, 1000));
    gridLayout = new QGridLayout(win_properties);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    verticalLayout = new QVBoxLayout();
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    groupBox_2 = new QGroupBox(win_properties);
    groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
    gridLayout_2 = new QGridLayout(groupBox_2);
    gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
    verticalLayout_2 = new QVBoxLayout();
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    horizontalLayout_3 = new QHBoxLayout();
    horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
    label_2 = new QLabel(groupBox_2);
    label_2->setObjectName(QString::fromUtf8("label_2"));

    horizontalLayout_3->addWidget(label_2);

    _txtName = new QLineEdit(groupBox_2);
    _txtName->setObjectName(QString::fromUtf8("_txtName"));

    horizontalLayout_3->addWidget(_txtName);


    verticalLayout_2->addLayout(horizontalLayout_3);

    horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
    label_7 = new QLabel(groupBox_2);
    label_7->setObjectName(QString::fromUtf8("label_7"));

    horizontalLayout_5->addWidget(label_7);

    txt_PubPeriod = new QLineEdit(groupBox_2);
    txt_PubPeriod->setObjectName(QString::fromUtf8("txt_PubPeriod"));
    QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(txt_PubPeriod->sizePolicy().hasHeightForWidth());
    txt_PubPeriod->setSizePolicy(sizePolicy1);
    txt_PubPeriod->setMinimumSize(QSize(45, 20));
    txt_PubPeriod->setMaximumSize(QSize(45, 20));

    horizontalLayout_5->addWidget(txt_PubPeriod);

    label_8 = new QLabel(groupBox_2);
    label_8->setObjectName(QString::fromUtf8("label_8"));

    horizontalLayout_5->addWidget(label_8);


    verticalLayout_2->addLayout(horizontalLayout_5);

    horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    label = new QLabel(groupBox_2);
    label->setObjectName(QString::fromUtf8("label"));

    horizontalLayout->addWidget(label);

    txt_IpNode = new QLineEdit(groupBox_2);
    txt_IpNode->setObjectName(QString::fromUtf8("txt_IpNode"));
    txt_IpNode->setMinimumSize(QSize(100, 20));
    txt_IpNode->setMaximumSize(QSize(100, 20));

    horizontalLayout->addWidget(txt_IpNode);


    verticalLayout_2->addLayout(horizontalLayout);

    horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
    label_4 = new QLabel(groupBox_2);
    label_4->setObjectName(QString::fromUtf8("label_4"));

    horizontalLayout_2->addWidget(label_4);

    txt_IpMaster = new QLineEdit(groupBox_2);
    txt_IpMaster->setObjectName(QString::fromUtf8("txt_IpMaster"));
    sizePolicy.setHeightForWidth(txt_IpMaster->sizePolicy().hasHeightForWidth());
    txt_IpMaster->setSizePolicy(sizePolicy);
    txt_IpMaster->setMinimumSize(QSize(0, 0));
    txt_IpMaster->setMaximumSize(QSize(100000, 100000));

    horizontalLayout_2->addWidget(txt_IpMaster);


    verticalLayout_2->addLayout(horizontalLayout_2);


    gridLayout_2->addLayout(verticalLayout_2, 0, 0, 1, 1);


    verticalLayout->addWidget(groupBox_2);


    gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

    buttonBox = new QDialogButtonBox(win_properties);
    buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
    buttonBox->setOrientation(Qt::Horizontal);
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

    gridLayout->addWidget(buttonBox, 1, 0, 1, 1);


    retranslateUi(win_properties);
    QObject::connect(buttonBox, SIGNAL(accepted()), win_properties, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()), win_properties, SLOT(reject())); 

} // setupUi

void Properties_GUI::retranslateUi(QDialog *win_properties){
    win_properties->setWindowTitle(QApplication::translate("win_properties", "Properties", 0, QApplication::UnicodeUTF8));
    groupBox_2->setTitle(QApplication::translate("win_properties", "ROS Node - Local Client", 0, QApplication::UnicodeUTF8));
    label_2->setText(QApplication::translate("win_properties", "Node Name", 0, QApplication::UnicodeUTF8));
    label_7->setText(QApplication::translate("win_properties", "Publi. Frecuency", 0, QApplication::UnicodeUTF8));
    txt_PubPeriod->setText(QApplication::translate("win_properties", "250", 0, QApplication::UnicodeUTF8));
    label_8->setText(QApplication::translate("win_properties", "Hz", 0, QApplication::UnicodeUTF8));
    label->setText(QApplication::translate("win_properties", "IP Node", 0, QApplication::UnicodeUTF8));
    txt_IpNode->setText(QApplication::translate("win_properties", "147.83.37.56", 0, QApplication::UnicodeUTF8));
    label_4->setText(QApplication::translate("win_properties", "URL Master", 0, QApplication::UnicodeUTF8));
    txt_IpMaster->setText(QApplication::translate("win_properties", "http://147.83.37.26:11311", 0, QApplication::UnicodeUTF8));

} // retranslate
}