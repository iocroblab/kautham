
#include <QtGui>
#include "constrainedcontrolwidget.h"
#include <QString>


namespace Kautham {


	ConstrainedControlWidget::ConstrainedControlWidget( Robot* rob, Problem* prob, int offset) {
    _robot = rob;
    _globalOffset = offset;
    _ptProblem = prob;
    string names = "This|is|a|test";
    if(rob != NULL) names= rob->getControlsName();
		gridLayout = new QGridLayout(this);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    vboxLayout = new QVBoxLayout();
    vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
    QLabel* tempLab;
    QSlider* tempSli;
    QString content(names.c_str());
		QStringList cont = content.split("|");
    QStringList::const_iterator iterator;
    for (iterator = cont.constBegin(); iterator != cont.constEnd()-4;++iterator){
      tempLab = new QLabel(this);
      tempLab->setObjectName(/*QString("lbl")  +*/ (*iterator).toUtf8().constData());
      content = (*iterator).toUtf8().constData();
      tempLab->setText(content.append(" = 0.5"));
      this->vboxLayout->addWidget(tempLab);
      labels.push_back(tempLab);

      tempSli = new QSlider(this);
      tempSli->setObjectName(/*"sld" + */(*iterator).toUtf8().constData());
      tempSli->setOrientation(Qt::Horizontal);
      tempSli->setMinimum(0);
      tempSli->setMaximum(1000);
      tempSli->setSingleStep(1);
      tempSli->setValue(500);
      vboxLayout->addWidget(tempSli);
      sliders.push_back(tempSli);
      QObject::connect(tempSli,SIGNAL(valueChanged(int)),SLOT(sliderChanged(int)));
    }

    values.resize(cont.size());
    for(int i=0; i<cont.size(); i++)
      values[i]=0.5;

    gridLayout->addLayout(vboxLayout,0,1,1,1);
		
	}

  ConstrainedControlWidget::~ConstrainedControlWidget(){
    for(unsigned int i=0; i<sliders.size(); i++){
      delete (QSlider*)sliders[i];
      delete (QLabel*)labels[i];
    }
    values.clear();
  }

	void ConstrainedControlWidget::sliderChanged(int value){
		QString tmp;
		for(unsigned int i=0; i<sliders.size(); i++){
		  values[i]=(KthReal)((QSlider*)sliders[i])->value()/1000.0;
		  tmp = labels[i]->text().left(labels[i]->text().indexOf("=") + 2);
		  labels[i]->setText( tmp.append( QString().setNum(values[i],'g',5)));
		}
	    
		_ptProblem->setCurrentControls(values,_globalOffset);
		if(_robot != NULL){
			_robot->ConstrainedKinematics(values);
		}
	}

  void ConstrainedControlWidget::setValues(vector<KthReal> &val){
    if(val.size() == sliders.size()){
      for(unsigned int i = 0; i < val.size(); i++)
        ((QSlider*)sliders[i])->setValue((int)(val[i]*1000.0));
    }
  }


}

