#include "bronchowidget.h"
#include "build/libgui/ui_bronchowidget.h"
#include <libproblem/ConsBronchoscopyKin.h>
#include <libplanner/guibrogridplanner.h>
#include "gui.h"

using namespace  libGUI;

 


bronchoWidget::bronchoWidget(Robot* rob, Problem* prob, int offset, GUI* gui) : //QWidget *parent
    ui(new Ui::bronchoWidget)
{
    ui->setupUi(this);

    _gui = gui;
    _robot = rob;
    _globalOffset = offset;
    _ptProblem = prob;
	_cameraView = false;
	_homeView = _gui->getActiveCameraTransfom();
	//_stepAdvance  =10;

    values.resize(3);
    lastZsliderPos=0;

    timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(zetaSliderChanged1()));
    //timer->start();
    
    QObject::connect(ui->alphaSlider,SIGNAL(valueChanged(int)),SLOT(alphaSliderChanged(int)));
    QObject::connect(ui->XiSlider,SIGNAL(valueChanged(int)),SLOT(xiSliderChanged(int)));
    QObject::connect(ui->DzSlider,SIGNAL(valueChanged(int)),SLOT(zetaSliderChanged(int)));
    QObject::connect(ui->DzSlider,SIGNAL(sliderReleased()),SLOT(zetaSliderReleased()));
    QObject::connect(ui->RateCheckBox,SIGNAL(stateChanged(int)),SLOT(setNavMode(int)));
    QObject::connect(ui->InverseCheckBox,SIGNAL(stateChanged(int)),SLOT(setAdvanceMode(int)));
    QObject::connect(ui->CameraCheckBox,SIGNAL(stateChanged(int)),SLOT(setCameraMode(int)));
    QObject::connect(ui->collisionCheckButton, SIGNAL( clicked() ), this, SLOT( collisionCheck() ) ); 
    QObject::connect(ui->advanceButton, SIGNAL( clicked() ), this, SLOT( advanceBronchoscope() ) ); 
    //QObject::connect(ui->stepAdvance, SIGNAL( valueChanged( int ) ), this, SLOT( stepAdvanceBronchoscope(int) ) ); 

	
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



/*
void bronchoWidget::stepAdvanceBronchoscope(int val)
{
	_stepAdvance = val;
	if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
	{
		((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->setAdvanceStep(_stepAdvance);
	}
  updateLookAt();
}
*/

void bronchoWidget::advanceBronchoscope()
{
	//advanceButton
	if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
	{
		KthReal s = ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getAdvanceStep();
		((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->advanceToBest(s);
		updateView();
	}	
	else
		cout<<"Sorry: This option only works for the planner named 'GUIBRO Grid Planner'"<<endl;
}


void bronchoWidget::collisionCheck()
{
	//collisionCheckButton
	if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
	{
		int dcost,NF1cost;
		bool c =((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->collisionCheck(&dcost,&NF1cost);
		if(c==true)
		{
			cout<<"The bronchoscope is in collision or out of bounds"<<endl;
			cout<<"The distance cost of current configuration is "<<dcost<<endl;
			cout<<"The NF1 value of the tip is "<<NF1cost<<endl;
		}
		else
		{
			cout<<"The bronchoscope is in a free configuration"<<endl;
			cout<<"The distance cost of current configuration is "<<dcost<<endl;
			cout<<"The NF1 value of the tip is "<<NF1cost<<endl;
		}
	}	
	else
		cout<<"Sorry: This option only works for the planner named 'GUIBRO Grid Planner'"<<endl;
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
  KthReal readVal=(KthReal) val;
  values[0]=(KthReal) readVal/1000; // /1000 because the slider only accept int values, in this case from -3141 to 3141
  _robot->ConstrainedKinematics(values);
  updateView();
  updateLookAt();
}


void bronchoWidget::xiSliderChanged(int val){
  KthReal readVal=(KthReal) val;
  values[1]=(KthReal) readVal/1000;
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
  updateView();
  updateLookAt();
  }

void bronchoWidget::zetaSliderChanged(int val){
  KthReal readVal=(KthReal) val;
  values[2]=(KthReal)(readVal-lastZsliderPos)/5;  // slider goes from -100 to 100, every delta is 10
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
  lastZsliderPos=readVal;
  updateView();
  updateLookAt();
}

void bronchoWidget::zetaSliderChanged1(){
  KthReal readVal=(KthReal) ui->DzSlider->value();
  values[2]=(KthReal) (readVal/1000);  // slider goes from -100 to 100, every step is of 0.05
    if (values[2]!=0){
      _ptProblem->setCurrentControls(values,_globalOffset);
      _robot->ConstrainedKinematics(values);
	  updateView();
    }
  updateLookAt();
}


void bronchoWidget::updateLookAt()
{
  if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
  {
	  KthReal bestalpha, bestbeta;
		KthReal s = ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getAdvanceStep();
	  ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->look(s, &bestalpha, &bestbeta);
	  //cout<<"Best Alpha = "<<bestalpha<<" Best Beta = "<<bestbeta<<endl;
  }	
}

void bronchoWidget::zetaSliderReleased(){
  QObject::disconnect(ui->DzSlider,SIGNAL(valueChanged(int)),0,0);
  ui->DzSlider->setValue(0);
  lastZsliderPos=0;
  values[2]=0;
  QObject::connect(ui->DzSlider,SIGNAL(valueChanged(int)),SLOT(zetaSliderChanged(int)));

  /*_ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);*/
  
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


void bronchoWidget::setAdvanceMode(int state){

	
	ConstrainedKinematic* ck = _robot->getCkine();
	if (state==Qt::Unchecked) ((ConsBronchoscopyKin*)ck)->setInverseAdvanceMode(false);
	else ((ConsBronchoscopyKin*)ck)->setInverseAdvanceMode(true);
	
}


void bronchoWidget::setCameraMode(int state){
	if (state==Qt::Unchecked) {
		_cameraView = false;
		_gui->setActiveCameraTransform(_homeView);
	}
	else{
		_cameraView = true;
	}
}

void bronchoWidget::updateView()
{
	if(_cameraView)
	{
		mt::Transform T_Ry;
		mt::Transform T_tz;
		T_Ry.setRotation( mt::Rotation(mt::Vector3(0,1,0),-M_PI/2) );
		
		T_tz.setTranslation( mt::Vector3(0,0,-(_robot->getLink(_robot->getNumLinks()-1)->getA()+1.1)) );
		mt::Transform camTrsf = _robot->getLastLinkTransform()*T_Ry*T_tz;
		_gui->setActiveCameraTransform(camTrsf);
	}
}

