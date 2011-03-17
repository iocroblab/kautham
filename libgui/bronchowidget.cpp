#include "bronchowidget.h"
#include "build/libgui/ui_bronchowidget.h"

bronchoWidget::bronchoWidget(Robot* rob, Problem* prob, int offset) : //QWidget *parent
    ui(new Ui::bronchoWidget)
{
    ui->setupUi(this);

    _robot = rob;
    _globalOffset = offset;
    _ptProblem = prob;

    values.resize(3);

    timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(zetaSliderChanged1()));
    //timer->start();
    
    QObject::connect(ui->alphaSlider,SIGNAL(valueChanged(int)),SLOT(alphaSliderChanged(int)));
    QObject::connect(ui->XiSlider,SIGNAL(valueChanged(int)),SLOT(xiSliderChanged(int)));
    QObject::connect(ui->DzSlider,SIGNAL(valueChanged(int)),SLOT(zetaSliderChanged(int)));
    QObject::connect(ui->DzSlider,SIGNAL(sliderReleased()),SLOT(zetaSliderReleased()));
    QObject::connect(ui->RateCheckBox,SIGNAL(stateChanged(int)),SLOT(setNavMode(int)));
}

bronchoWidget::~bronchoWidget()
{
    delete ui;
}

void bronchoWidget::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void bronchoWidget::alphaSliderChanged(int val){
  /*QString tmp;c
  for(unsigned int i=0; i<sliders.size(); i++){
    values[i]=(KthReal)((QSlider*)sliders[i])->value()/1000.0;
    tmp = labels[i]->text().left(labels[i]->text().indexOf("=") + 2);
    labels[i]->setText( tmp.append( QString().setNum(values[i],'g',5)));
  }
    
  _ptProblem->setCurrentControls(values,_globalOffset);
  if(_robot != NULL) _robot->control2Pose(values);*/
  values[0]=(KthReal) val/1000; // /1000 because the slider only accept int values, in this case from -3141 to 3141
  _robot->ConstrainedKinematics(values);
}


void bronchoWidget::xiSliderChanged(int val){
  values[1]=(KthReal) val/1000;
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
  }

void bronchoWidget::zetaSliderChanged(int val){
  values[2]=(KthReal)val/10;  // slider goes from -100 to 100, every step is 0.1
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
}

void bronchoWidget::zetaSliderChanged1(){
  values[2]=(KthReal) ui->DzSlider->value()/20;  // slider goes from -100 to 100, every step is of 0.05
    if (values[2]!=0){
      _ptProblem->setCurrentControls(values,_globalOffset);
      _robot->ConstrainedKinematics(values);
    }

}

void bronchoWidget::zetaSliderReleased(){
  ui->DzSlider->setValue(0);
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);

}

void bronchoWidget::setNavMode(int state){
  if (state==Qt::Unchecked){
    disconnect(timer, SIGNAL(timeout()), 0, 0);
    timer->stop();
    QObject::connect(ui->DzSlider,SIGNAL(valueChanged(int)),SLOT(zetaSliderChanged(int)));
     }
  else if (state==Qt::Checked)
  {
    QObject::disconnect(ui->DzSlider,SIGNAL(valueChanged(int)),0,0);
    connect(timer, SIGNAL(timeout()), this, SLOT(zetaSliderChanged1()));
    timer->start();
  }

}