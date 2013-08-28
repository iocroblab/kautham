#include "bronchowidget.h"
//#include "ui_bronchowidget.h"
#include <ui_bronchowidget.h>
#include "consbronchoscopykin.h"
#include "guibrogridplanner.h"
#include <libgui/gui.h>

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
	_updateValues = true;
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
    QObject::connect(ui->CameraCheckBox,SIGNAL(stateChanged(int)),SLOT(setCameraMode(int)));
    QObject::connect(ui->collisionCheckButton, SIGNAL( clicked() ), this, SLOT( collisionCheck() ) ); 
    QObject::connect(ui->advanceButton, SIGNAL( clicked() ), this, SLOT( advanceBronchoscope() ) ); 

	
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



void bronchoWidget::advanceBronchoscope()
{
	//advanceButton
	if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
	{
		//restore alpha0, beta0
		((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->registerValues();
		//get advance step
		KthReal s = ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getAdvanceStep();
		//advance
		KthReal a,b;//dummy. not used
		((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->advanceToBest(s, &a, &b);
		updateView();
		//update alpha0, beta0
		((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->registerValues();
		KthReal alpha = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getvalues(0);
		KthReal beta = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getvalues(1);
		
		//the flag _updateValues is set to false in order that the setSliderPosition function changes the porition of the slider but
		//does not execute the code of the alphaSliderChanged function, if not the bronchoscpoe moves when it should not move
		//(surely it can be done in another better way....)
		_updateValues = false;
		int aa,bb;
		if(alpha>0) aa=(int)(alpha*ui->alphaSlider->maximum());
		else aa=(int)(-alpha*ui->alphaSlider->minimum());
		ui->alphaSlider->setSliderPosition(aa);
		
		if(beta>0) bb=(int)(beta*ui->XiSlider->maximum());
		else bb=(int)(-beta*ui->XiSlider->minimum());
		ui->XiSlider->setSliderPosition(bb);
		_updateValues = true;
	}	
	else
		cout<<"Sorry: This option only works for the planner named 'GUIBRO Grid Planner'"<<endl;
}


void bronchoWidget::collisionCheck()
{
	//collisionCheckButton
	if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
	{
		KthReal dcost;
		KthReal NF1cost;
		//bool c =((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->collisionCheck(&dcost,&NF1cost);
		bool c =((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->comply(&dcost,&NF1cost);
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

	if(_updateValues==false) return;

  KthReal readVal=(KthReal) val;
  KthReal maxAlpha = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getMaxAlpha();
  KthReal minAlpha = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getMinAlpha();
  
  if(readVal>0){
	  //values goes in the range -1..1
	values[0]=(KthReal)readVal/ui->alphaSlider->maximum(); // readVal/1000
  }
  else
  {
	  //values goes in the range -1..1
	values[0]=(KthReal)-readVal/ui->alphaSlider->minimum(); // readVal/1000
  }

  _robot->ConstrainedKinematics(values);
  updateView();
  updateLookAt();
//print inf0
  KthReal a;
  if(values[0]>0) a=values[0]*maxAlpha*180/M_PI;
  else a=-values[0]*minAlpha*180/M_PI;
  //cout<<"Alpha = "<<a<<endl;
  //cout<<"val = "<<val<<" alpha = "<<values[0]<<" beta = "<<values[1]<<endl;
}


void bronchoWidget::xiSliderChanged(int val){
	
	if(_updateValues==false) return;

  KthReal readVal=(KthReal) val;
  KthReal maxBending = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getMaxBending();
  KthReal minBending = ((ConsBronchoscopyKin*)_ptProblem->getPlanner()->wkSpace()->getRobot(0)->getCkine())->getMinBending();

  if(readVal>0)
  {
	  //values goes in the range -1..1
	values[1]=(KthReal) readVal/ui->XiSlider->maximum();
  }
  else
  {
	  //values goes in the range -1..1
	values[1]=(KthReal) -readVal/ui->XiSlider->minimum();
  }
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
  updateView();
  updateLookAt();
//print inf0
  KthReal b;
  if(values[1]>0) b=values[1]*maxBending*180/M_PI;
  else b=-values[1]*minBending*180/M_PI;
  //cout<<"Beta = "<<b<<endl;
  //cout<<"val = "<<val<<" alpha = "<<values[0]<<" beta = "<<values[1]<<endl;
  }

void bronchoWidget::zetaSliderChanged(int val){
  KthReal readVal=(KthReal) val;
  values[2]=(KthReal)(readVal-lastZsliderPos)/5;  // slider goes from -100 to 100, every delta is 10 ¿?
  _ptProblem->setCurrentControls(values,_globalOffset);
  _robot->ConstrainedKinematics(values);
  lastZsliderPos=readVal;
  updateView();
  //updateLookAt();
}

void bronchoWidget::zetaSliderChanged1(){
  KthReal readVal=(KthReal) ui->DzSlider->value();
  values[2]=(KthReal) (readVal/1000);  // slider goes from -100 to 100, every step is of 0.05 ¿?
    if (values[2]!=0){
      _ptProblem->setCurrentControls(values,_globalOffset);
      _robot->ConstrainedKinematics(values);
	  updateView();
    }
  //updateLookAt();
}


void bronchoWidget::updateLookAt()
{
  if(_ptProblem->getPlanner()->getIDName()=="GUIBRO Grid Planner")
  {
	  if( ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getShowPoints() != 0)
	  {
		KthReal bestalpha, bestbeta;
		KthReal s = ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getAdvanceStep();
		((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->look(s, &bestalpha, &bestbeta);
		//cout<<"Best Alpha = "<<bestalpha<<" Best Beta = "<<bestbeta<<endl;
	  }
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

void bronchoWidget::setCameraMode(int state){
	if (state==Qt::Unchecked) {
		_cameraView = false;
		_gui->setActiveCameraTransform(_homeView);
    _robot->setPathVisibility( false );
	}
	else{
		_cameraView = true;
    _robot->setPathVisibility( true );
	}
}

void bronchoWidget::updateView()
{
	if(_cameraView)
	{
		mt::Transform T_Ry;
		mt::Transform T_tz;
		T_Ry.setRotation( mt::Rotation(mt::Vector3(0,1,0),-M_PI/2) );
		
		KthReal offset = ((libPlanner::GUIBROGRID::GUIBROgridPlanner*)_ptProblem->getPlanner())->getBronchoscopeRadius() * 1.1;

		T_tz.setTranslation( mt::Vector3(0,0,-(_robot->getLink(_robot->getNumLinks()-1)->getA()+offset)) );
		mt::Transform camTrsf = _robot->getLastLinkTransform()*T_Ry*T_tz;
		_gui->setActiveCameraTransform(camTrsf);
	}
}

