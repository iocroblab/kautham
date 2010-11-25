/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include "teleoperationwidget.h"
#include "gui.h"
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoTransparencyType.h>

namespace libGUI{
  TeleoperationWidget::TeleoperationWidget(Problem* prob, Device* hap, GUI* gui){

    setupUI();

    // Core Initialization
    _synchronized = false;
    _problem = prob;
    _haptic = hap;
    _gui = gui;
    _inAction = _lastAction = _callNewH = false;
    settingRX->setText("0.0");
    settingRY->setText("0.0");
    settingRZ->setText("0.0");
    settingX->setText("0.0");
    settingY->setText("0.0");
    settingZ->setText("0.0");
    _rotScale = _tranScale = 1.0;
    _hapticBoxSize[0] = (KthReal)(_haptic->deviceLimits(0)[1] - _haptic->deviceLimits(0)[0]);
    _hapticBoxSize[1] = (KthReal)(_haptic->deviceLimits(1)[1] - _haptic->deviceLimits(1)[0]);
    _hapticBoxSize[2] = (KthReal)(_haptic->deviceLimits(2)[1] - _haptic->deviceLimits(2)[0]);
    _hapticBoxSize[3] = (KthReal)(_haptic->deviceLimits(3)[1] - _haptic->deviceLimits(3)[0]);
    _hapticBoxSize[4] = (KthReal)(_haptic->deviceLimits(4)[1] - _haptic->deviceLimits(4)[0]);
    _hapticBoxSize[5] = (KthReal)(_haptic->deviceLimits(5)[1] - _haptic->deviceLimits(5)[0]);
    _guidingPath[0].clear();
    _guidingPath[1].clear();
    _threshold = _hapticBoxSize[0] / 100.;

    _maxForces[0]= 4.; _maxForces[1]= 4.; _maxForces[2]= 4.;
    _maxForces[3]= 2.; _maxForces[4]= 2.; _maxForces[5]= 2.;

    _hapticBox = NULL;
    _posVec = NULL;
    _rotVec = NULL;
    _scaVec = NULL;


    if(_problem->wSpace()->robotsCount() < 2)
      _radBttRobot1->setEnabled(false);

    updateGuidingPath();

    connect(_cameraTop, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_cameraBottom, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_cameraLeft, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_cameraRight, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_cameraFront, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_cameraRear, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_setCamera, SIGNAL(clicked()), this, SLOT(changeCamera()));
    connect(_getCamera, SIGNAL(clicked()), this, SLOT(getCamera()));

    connect(_startOperation, SIGNAL(clicked()), this, SLOT(startTeleoperation()));
    connect(_stopOperation, SIGNAL(clicked()), this, SLOT(stopTeleoperation()));
    connect(_haptic, SIGNAL(updated()), this, SLOT(acts()));
    connect(_sliderRotScale, SIGNAL(valueChanged(int)), this, SLOT(changeRotScale(int)));
    connect(_sliderTransScale, SIGNAL(valueChanged(int)), this, SLOT(changeTranScale(int)));
    connect(_radBttRobot0, SIGNAL(clicked()), this, SLOT(changeRobot()));
    connect(_radBttRobot1, SIGNAL(clicked()), this, SLOT(changeRobot()));
    connect(_radCamera, SIGNAL(clicked()), this, SLOT(changeRobot()));
    connect(_radWorld, SIGNAL(clicked()), this, SLOT(changeRobot()));
    
    _stopOperation->setEnabled(false);
  }

  void TeleoperationWidget::updateGuidingPath(){
    // Creating the guiding path for each robot from their proposed solution.
    GuiNode* temp;
    vector<RobConf>::iterator it;
    float ya,pi,roll;
    ya=pi=roll = (float)0.0;
    for(unsigned int i = 0; i < 2; i++ ){
      for(unsigned int j = 0; j < _guidingPath[i].size(); j++ )
        delete[] _guidingPath[i].at(j);
      _guidingPath[i].clear();
    }

    // Filling the points of the guiding path
    KthReal radC,radS;
    radC = radS = 0.;
    for(unsigned i = 0; i < 3; i++)
      radS += _hapticBoxSize[i] * _hapticBoxSize[i];
    radS = sqrt(radS)/10.;

    for(unsigned int i = 0; i < _problem->wSpace()->robotsCount(); i++ ){
      vector<RobConf>& path = _problem->wSpace()->getRobot(i)->getProposedSolution();
      if(path.size() != 0)
        for(it = path.begin(); it != path.end(); ++it){
          
          _problem->wSpace()->getRobot(i)->Kinematics(*it);
          mt::Transform& trans = _problem->wSpace()->getRobot(i)->getLastLinkTransform();
          mt::Point3& pos = trans.getTranslationRef();
          mt::Rotation& rot =trans.getRotationRef();
          temp = new GuiNode;
          temp->point[0] = pos.at(0); temp->point[1] = pos.at(1); temp->point[2] = pos.at(2);
          rot.getYpr(ya, pi, roll);
          temp->point[3] = ya; temp->point[4] = pi; temp->point[5] = roll;
          _guidingPath[i].push_back(temp);
        }
    }

    // Calculate the unit vector from point i to point i+1 until point n-1
    for(unsigned int i = 0; i < _problem->wSpace()->robotsCount(); i++ ){
      if(_guidingPath[i].size() != 0)
        for(unsigned int j = 0; j < _guidingPath[i].size()-1; j++){
          double norm = 0.0;
          for(unsigned int k = 0; k < 6; k++){
            _guidingPath[i].at(j)->unitVec[k] = _guidingPath[i].at(j+1)->point[k]
                                                - _guidingPath[i].at(j)->point[k];
            norm += _guidingPath[i].at(j)->unitVec[k] * _guidingPath[i].at(j)->unitVec[k];
          }
          norm = sqrt(norm);
          for(unsigned int k = 0; k < 6; k++)
            if( norm != 0. )
              _guidingPath[i].at(j)->unitVec[k] /= norm;
        }
    }
  }

  unsigned int TeleoperationWidget::findNearNode(vector<GuiNode*>& path, pathPoint& tcploc, KthReal& dist){
    if(path.size() > 0 ){
      // First find the closer translational point
      KthReal* dis = new KthReal[path.size()];
      for(unsigned int i = 0; i < path.size(); i++){
        dis[i] = 0.;
        for(unsigned int j = 0; j<3; j++)
          dis[i] += (path.at(i)->point[j] - tcploc[j] ) * (path.at(i)->point[j] - tcploc[j] );
      }
      unsigned int index = 0;
      for(unsigned int i = 1; i < path.size(); i++){
        if(dis[i] < dis[index])
          index = i;
      }
      index = index==path.size() - 1 ? --index : index;
      dist = sqrt(dis[index]);
      delete[] dis;
      return index;
    }
    return 0;
  }

  void TeleoperationWidget::findGuideVector(vector<GuiNode*>& path, pathPoint& tcploc, pathPoint& vecTo){
    if(path.size() > 0 ){
      KthReal dist = 0.;
      unsigned int index = findNearNode(path, tcploc, dist);
      // Now index is the closer point but the returned one will have positive projection.
      // inner product will be positive
      
      GuiNode& candidate = *(path.at(index));
      KthReal inn = 0.;
      for(unsigned int j = 0; j<3; j++){
        vecTo[j] = tcploc[j] - candidate.point[j];
        inn += vecTo[j] * candidate.unitVec[j];
      }
      
      if(inn >= 0)
        for(unsigned int j = 0; j<6; j++)
          vecTo[j] = candidate.unitVec[j];

      // if projection is negative, send the bisector unit vector with node before.
      else{
        if(index == 0 )++index;
        if(dist > _threshold)
          for(unsigned int j = 0; j<6; j++)
            vecTo[j] = path.at(index - 1)->unitVec[j];
        else{
          inn = 0.;
          _threshold = _hapticBoxSize[0] / 100.;
          KthReal ratio = dist / _threshold;
          KthReal diffe = 1.0 - ratio;
          for(unsigned int j = 0; j<6; j++){
            vecTo[j] = ratio * path.at(index - 1)->unitVec[j] + diffe * candidate.unitVec[j];
            inn += vecTo[j] * vecTo[j];
          }
          inn=sqrt(inn);
          // Normalizing.
          for(unsigned int j = 0; j<6; j++)
            vecTo[j] /= inn;
        }
      }
    }
  }

  mt::Transform TeleoperationWidget::getNewH2HipVec(vector<GuiNode*>& path, mt::Transform& tcp){
    mt::Transform vecH;
    if(path.size() > 0 ){
      // This prodedure uses the unit vector to next node to find the new Haptic frame.
      pathPoint tcpPoint, vecTo;
      KthReal ya, pi, roll;
      mt::Point3& pos = tcp.getTranslationRef();
      mt::Rotation& rot =tcp.getRotationRef();
      tcpPoint[0] = pos.at(0); tcpPoint[1] = pos.at(1); tcpPoint[2] = pos.at(2);
      rot.getYpr(ya, pi, roll);
      tcpPoint[3] = ya; tcpPoint[4] = pi; tcpPoint[5] = roll;

      findGuideVector(path, tcpPoint,vecTo); // returns the vecTo in W frame

      mt::Transform vecW;
      vecW.setTranslation( Point3(vecTo[0], vecTo[1], vecTo[2]));
      vecW.setRotation( Rotation(vecTo[3], vecTo[4], vecTo[5]));

      // Transform the vector to H frame.
      vecH=_w2h.inverse();
      vecH.setTranslation(mt::Point3(0., 0., 0.));
      vecH = vecH * vecW;
      mt::Point3& pos2 = vecH.getTranslationRef();
      vecTo[0] = pos2.at(0); vecTo[1] = pos2.at(1); vecTo[2] = pos2.at(2);
      vecH.getRotation().getYpr(vecTo[3], vecTo[4], vecTo[5]);

      KthReal times[6];
      unsigned int index=0;
      for(unsigned int i = 0; i < 6; i++){
        if(i<3)
          times[i] = _hapticBoxSize[i] * _tranScale / vecTo[i];
        else
          times[i] = _hapticBoxSize[i] * _rotScale / vecTo[i];
        index = abs(times[index]) > abs(times[i]) ? i : index;
      }

      for(unsigned int i = 0; i < 6; i++)
        vecTo[i] *= times[index];

      //vecH.setTranslation( Point3(vecTo[0], vecTo[1], vecTo[2]));
      //vecH.setRotation( Rotation(vecTo[3], vecTo[4], vecTo[5]));

      //vecW = _h2w * vecH;

      //vecH = vecW.inverse() * tcp;

      //// Now it is divided by the respective scale
      //mt::Point3& pos3 = vecH.getTranslationRef();
      //vecTo[0] = pos3.at(0); vecTo[1] = pos3.at(1); vecTo[2] = pos3.at(2);
      //vecH.getRotation().getYpr(vecTo[3], vecTo[4], vecTo[5]);

      for(unsigned int i = 0; i < 6; i++)
        if(i<3)
          vecTo[i] /= _tranScale;
        else
          vecTo[i] /= _rotScale;

      vecH.setTranslation(Point3(vecTo[0], vecTo[1], vecTo[2]));
      vecH.setRotation(Rotation(vecTo[3], vecTo[4], vecTo[5]));
    }
    // Now the function returns the attraction point in the H frame.
    return vecH;
  }

  mt::Transform TeleoperationWidget::getNewH2HipBB(vector<GuiNode*>& path, mt::Transform& tcp){
    mt::Transform newHip2H;
    if(path.size() > 0 ){
      pathPoint tcpPoint;
      KthReal ya, pi, roll;
      mt::Transform inverse = _w2h.inverse();
      mt::Transform tcpLoc = inverse * tcp; // Tcp in H frame
      mt::Point3& pos = tcpLoc.getTranslationRef();
      mt::Rotation& rot =tcpLoc.getRotationRef();
      tcpPoint[0] = pos.at(0); tcpPoint[1] = pos.at(1); tcpPoint[2] = pos.at(2);
      rot.getYpr(ya, pi, roll);
      tcpPoint[3] = ya; tcpPoint[4] = pi; tcpPoint[5] = roll;

      vector<mt::Point3> tmpPoints;
      tmpPoints.push_back(pos);
      KthReal lims[3][2], dist[3];

      for(unsigned int i = 0; i < 3; i++)
        lims[i][0] = lims[0][1] = pos.at(i);

      dist[0] = dist[1] = dist[2] = 0.;

      unsigned int index = findNearNode(path, tcpPoint, ya);
      mt::Point3 tmpPoint;
      unsigned int ind=0;
      do{
        ind = index++;
        tmpPoint.setValue(path.at(ind)->point[0], path.at(ind)->point[1], path.at(ind)->point[2]);
        tmpPoint = inverse * tmpPoint;
        tmpPoints.push_back(tmpPoint);
        for(unsigned int i = 0; i < tmpPoints.size(); i++)
          for(unsigned j = 0; j < 3; j++){
            lims[j][0] = lims[j][0] > tmpPoints.at(i)[j] ? tmpPoints.at(i)[j]: lims[j][0];
            lims[j][1] = lims[j][1] < tmpPoints.at(i)[j] ? tmpPoints.at(i)[j]: lims[j][1];
          }
        for(unsigned int i = 0; i < 3; i++)
          dist[i] = abs(lims[i][1] - lims[i][0]);

      }while(index < path.size() && (dist[0] < _hapticBoxSize[0] * _tranScale)
              && (dist[1] < _hapticBoxSize[1] * _tranScale)
              && (dist[2] < _hapticBoxSize[2] * _tranScale));

      // Calculate the new Hip position in H frame
      for(unsigned int i = 0; i < 3; i++){
        tcpPoint[i] = lims[i][0] - (lims[i][1] - dist[i])/2.;
        tcpPoint[i] /= _tranScale;
      }

     newHip2H.setTranslation(Point3(tcpPoint[0], tcpPoint[1], tcpPoint[2]));
    }
    return newHip2H;
  }

  // This method returns the Force to be applied in the vecDir direction
  void TeleoperationWidget::calculateForce(pathPoint delta, KthReal dist, KthReal forces[]){
    KthReal signs[3];
    for(unsigned int i = 0; i < 3; i++)
      signs[0] = delta[i+3] > M_PI ? delta[i+3]-2*M_PI : delta[i+3];

    if( dist > 2 * _threshold){
      //Apply the maximum force in the dif direction
      for(unsigned int i = 0; i < 3; i++)
        forces[i] = _maxForces[i] * delta[i] / dist;
      for(unsigned int i = 0; i < 3; i++)
        forces[i+3] = _maxForces[i+3] * signs[i]/M_PI;
    }else{
      //Apply a proporcional force f= fmax * dist / (4*th^2)
      for(unsigned int i = 0; i < 3; i++)
        forces[i] = _maxForces[i] * delta[i] / (4 * dist * _threshold * _threshold) ;
      for(unsigned int i = 0; i < 3; i++)
        forces[i+3] = _maxForces[i+3] * signs[i] /(4*M_PI*M_PI);
    }
  }

  void TeleoperationWidget::setGuideForce(mt::Transform& tcp){
    KthReal forces[6];

    for(unsigned int i = 0; i < 6; i++)
        forces[i] = 0.;

    _haptic->setSE3Force(forces);
  }

  void TeleoperationWidget::setJumpForce(mt::Transform& tcp){
    KthReal dis = 0.;
    KthReal ya, pi, roll;
    KthReal forces[6];

    KthReal delta[6];
    for(unsigned int i = 0; i < 3; i++)
      delta[i] = _h2newHip.getTranslation().at(i);

    _h2newHip.getRotation().getYpr(delta[3], delta[4], delta[5]);

    for(unsigned int i = 0; i < 3; i++)
      delta[i] -= tcp.getTranslation().at(i);

    tcp.getRotation().getYpr(ya, pi, roll);

    delta[3] -= ya;
    delta[4] -= pi;
    delta[5] -= roll;

    for(unsigned int i = 0; i < 6; i++)
      dis += delta[i] * delta[i];
    dis = sqrt(dis);

    calculateForce(delta,dis, forces);

    _haptic->setSE3Force(forces);
  }

  mt::Transform TeleoperationWidget::getNewH2Hip(){
    pathPoint tcpPoint;
    KthReal ya, pi, roll;
    vector<GuiNode*>* path = NULL;

    if( _radBttRobot0->isChecked() )
      path = &_guidingPath[0];
    else if( _radBttRobot1->isChecked() )
      path = &_guidingPath[1];
 
    _callNewH = false; // Next time does not call this method until a new button press event.

    if(path->size() > 0 ){
      //mt::Transform& tcp = _haptic->getHIP();
      // Retrieving the TCP absolute transformation
      mt::Transform* tcp = NULL;
      if( _radBttRobot0->isChecked() )
        tcp = &_problem->wSpace()->getRobot(0)->getLastLinkTransform();
      else if( _radBttRobot1->isChecked() )
        tcp = &_problem->wSpace()->getRobot(1)->getLastLinkTransform();

      mt::Point3& pos = tcp->getTranslationRef();
      mt::Rotation& rot =tcp->getRotationRef();
      tcpPoint[0] = pos.at(0); tcpPoint[1] = pos.at(1); tcpPoint[2] = pos.at(2);
      rot.getYpr(ya, pi, roll);
      tcpPoint[3] = ya; tcpPoint[4] = pi; tcpPoint[5] = roll;

      KthReal dist = 0.;

      unsigned int index = findNearNode(*path, tcpPoint, dist);
      
       // if distance to near node is greater than _hapticBoxSize use vector method.
      if(dist > _hapticBoxSize[0] / 2.)
        return getNewH2HipVec(*path,*tcp);
      else
        return getNewH2HipBB(*path,*tcp);
    }else{
      mt::Transform tmp;
      return tmp;
    }
  }


  void TeleoperationWidget::acts(){
    if(_inAction ){
      if(_lastAction == false || _haptic->getButtonState()){
        if(_callNewH)
          _h2newHip = getNewH2Hip();
        //else
          synchronization();
      }else
        teleoperation();
    }
    _lastAction = _inAction;
  }

  void TeleoperationWidget::changeRotScale(int value){
    _rotScale = value/10.;
    _lblRotScale->setText( "Rotational Scale: " + QString().setNum(_rotScale,'f',1));
    synchronization(false);
  }

  void TeleoperationWidget::changeTranScale(int value){
    _tranScale = value/10.;
    _lblTransScale->setText( "Translational Scale: " + QString().setNum(_tranScale,'f',1));
    if( _scaVec != NULL )
      _scaVec->setValue(_tranScale, _tranScale, _tranScale);
    synchronization(false);
  }

  void TeleoperationWidget::changeRobot(){
    _sliderRotScale->setValue(10);
    _sliderTransScale->setValue(10);
    synchronization(false);
  }

  void TeleoperationWidget::startTeleoperation(){
    _inAction = true;
    // Prevent to change the teleoperated robot and method
    _radBttRobot0->setEnabled(false);
    _radBttRobot1->setEnabled(false);
    //_radWorld->setEnabled(false);
    //_radCamera->setEnabled(false);

    _stopOperation->setEnabled(true);
    _startOperation->setEnabled(false);
    acts();

    if(_hapticBox == NULL){
      // Adding the haptic workspace box

      mt::Rotation tmpRot = _gui->getActiveCameraTransfom().getRotation();
      mt::Point3 tmpPos = _w2h.getTranslation();

      SoTranslation *trans;
      SoRotation    *rot;
      // SoMaterial    *color;
      SoScale       *sca;

      //====================
      trans= new SoTranslation;
		  rot = new SoRotation;
		  sca = new SoScale();

      _posVec = new SoSFVec3f;
      trans->translation.connectFrom(_posVec);
      _posVec->setValue(tmpPos.at(0), tmpPos.at(1), tmpPos.at(2));

      _rotVec = new SoSFRotation;
      _rotVec->setValue(tmpRot.at(0), tmpRot.at(1), tmpRot.at(2), tmpRot.at(3));
      rot->rotation.connectFrom(_rotVec);

		  _hapticBox = new SoSeparator;
		  _hapticBox->ref();
      _hapticBox->setName("HapticBox");
		  //_hapticBox->addChild(sca);
	    _hapticBox->addChild(trans);
	    _hapticBox->addChild(rot);

      // The material =======
      //SoTransparencyType
      SoTransparencyType *ttype = new SoTransparencyType;
			ttype->value = SoGLRenderAction::SORTED_OBJECT_BLEND ;
			_hapticBox->addChild(ttype);

      SoMaterial* tmpMat = new SoMaterial;
      tmpMat->ambientColor.setValue( 1.0, 0.6471, 0.0);
      tmpMat->diffuseColor.setValue( 1.0, 0.6471, 0.0);
      tmpMat->shininess.setValue( 0.7);
      tmpMat->transparency.setValue( 0.7);
      _hapticBox->addChild(tmpMat);
      
      SoCube* tmp = new SoCube();
      tmp->width.setValue((float)_hapticBoxSize[0]);
      tmp->height.setValue((float)_hapticBoxSize[1]);
      tmp->depth.setValue((float)_hapticBoxSize[2]);

      SoSeparator* tmp2 = new SoSeparator;
      _scaVec= new SoSFVec3f;
      _scaVec->setValue(_tranScale, _tranScale, _tranScale);
      sca->scaleFactor.connectFrom(_scaVec);
      tmp2->addChild(sca);
      SoTranslation *tmpTrans = new SoTranslation;
      tmpTrans->translation.setValue((float)((_haptic->deviceLimits(0)[1] + _haptic->deviceLimits(0)[0])/2.),
                                     (float)((_haptic->deviceLimits(1)[1] + _haptic->deviceLimits(1)[0])/2.),
                                     (float)((_haptic->deviceLimits(2)[1] + _haptic->deviceLimits(2)[0])/2.));
      tmp2->addChild(tmpTrans);
      tmp2->addChild(tmp);

      _hapticBox->addChild(tmp2);
		  _hapticBox->ref();
    }
    SoSeparator* tmpSep = _gui->getRootTab(_gui->getActiveViewTitle());
    tmpSep->addChild(_hapticBox);
    
  }

  void TeleoperationWidget::stopTeleoperation(){
    _inAction = false;
    // Allow to change the teleoperated robot
    _radBttRobot0->setEnabled(true);
    _radBttRobot1->setEnabled(true);
    //_radWorld->setEnabled(true);
    //_radCamera->setEnabled(true);
    _stopOperation->setEnabled(false);
    _startOperation->setEnabled(true);

    if(_problem->wSpace()->robotsCount() < 2)
      _radBttRobot1->setEnabled(false);

    SoSeparator* tmpSep = _gui->getRootTab(_gui->getActiveViewTitle());
    tmpSep->removeChild(_hapticBox);
  }

  void TeleoperationWidget::getCamera(){
    const mt::Transform tmp(_gui->getActiveCameraTransfom());
    mt::Point3 pos = tmp.getTranslation();
    currentX->setText(QString().setNum(pos.at(0),'f',2));
    currentY->setText(QString().setNum(pos.at(1),'f',2));
    currentZ->setText(QString().setNum(pos.at(2),'f',2));

    mt::Rotation rot = tmp.getRotation();
    mt::Scalar y,p,r;
    rot.getYpr(y,p,r);
    currentRX->setText(QString().setNum(r,'f',2));
    currentRY->setText(QString().setNum(p,'f',2));
    currentRZ->setText(QString().setNum(y,'f',2));
  }

  void TeleoperationWidget::changeCamera(){
    QObject* sender = QObject::sender ();
    float x, y, z, yw, pt, ro;
    float qx, qy, qz, qw;
    x = y = z = 0.;
    pt = yw = ro = 0.;
    qx = qy = qz =  0.;
    qw = 1.;

    if(sender == _cameraTop ){
      x = 0.0; y = 0.0; z = 1.0;
    }else if(sender == _cameraBottom ){
      x = 0.0; y = 0.0; z = -1.0; ro = M_PI;
    }else if(sender == _cameraLeft ){
      x = 0.0; y = -1.0; z = 0.0; yw = 1.5*M_PI; ro = M_PI/2.;
    }else if(sender == _cameraRight ){
      x = 0.0; y = 1.0; z = 0.0; yw = M_PI/2.; ro = M_PI/2.;
    }else if(sender == _cameraFront ){
      x = 1.0; y = 0.0; z = 0.0; ro = M_PI/2.;
    }else if(sender == _cameraRear ){
      x = -1.0; y = 0.0; z = 0.0; yw = M_PI; ro = M_PI/2.;
    }else if(sender == _setCamera){
      x = settingX->text().toFloat();
      y = settingY->text().toFloat();
      z = settingZ->text().toFloat();
      yw = settingRZ->text().toFloat();
      pt = settingRY->text().toFloat();
      ro = settingRX->text().toFloat();
    }
    //cout << yw << ", " << pt << ", " << ro << endl;
    mt::Rotation rot(yw, pt, ro);
    qx = rot.at(0);
    qy = rot.at(1);
    qz = rot.at(2);
    qw = rot.at(3);
    _gui->setActiveCameraPosition(x, y, z);
    _gui->setActiveCameraRotation(qx, qy, qz, qw);
    synchronization(false);
  }


  void TeleoperationWidget::synchronization(bool applyForces){
    // Retrieving the TCP absolute transformation
    if( _radBttRobot0->isChecked() )
      _w2tcp0 = _problem->wSpace()->getRobot(0)->getLastLinkTransform();
    else if( _radBttRobot1->isChecked() )
      _w2tcp0 = _problem->wSpace()->getRobot(1)->getLastLinkTransform();

    //Geting hip02h from haptic device
    mt::Transform hreal, aux;
    hreal = _haptic->getHIP();
    _h2hip0.setIdentity();
    _h2hip0.setTranslation(hreal.getTranslation()* _tranScale);
    aux = _h2hip0;
    //// Now scalling the rotation
    mt::Rotation auxrot = hreal.getRotation();
    mt::Unit3 axis;
    mt::Scalar angle;
    auxrot.getAxisAngle(axis, angle);

    if( angle > M_PI ) angle -= 2*M_PI;
    //if( abs(angle) < M_PI / _rotScale)
      auxrot.setAxisAngle(axis, angle*_rotScale);

    _h2hip0.setRotation(auxrot );

    if(_radCamera->isChecked()){
      const mt::Transform ci2w = _gui->getActiveCameraTransfom();
      // Calculating matrix transformation for the camera into TCP0
      mt::Transform ci2tcp0 = _w2tcp0.inverse() * ci2w;

      // Rotation of ci2tcp0 is equal to rotation of tcpci2tcp0
      _tcp2tcpci.setRotation(ci2tcp0.getRotation());

      // Calculating h2w
      _w2h = _w2tcp0 * _tcp2tcpci * aux.inverse();

    }else{
      // Calculating h2w
      //const mt::Rotation r1(mt::Unit3(0., 1., 0.),-M_PI/2.);
      //const mt::Rotation r2(mt::Unit3(1., 0., 0.),-M_PI/2.);
      //_h2w.setRotation(r1*r2);
      _w2h.setRotation(Rotation(0., 0., 0., 1.));
      //_h2w.setRotation(Rotation(-0.5, -0.5, -0.5, 0.5));
      _w2h.setTranslation(_w2tcp0.getTranslation());
      aux.setRotation(Rotation(-0.5, -0.5, -0.5, 0.5));
      _w2h = _w2h * aux.inverse(); // aux is only translational part rotated 

    }

    _whip2tcp0 = _h2hip0.inverse() * _w2h.inverse() *_w2tcp0;

    if(_posVec != NULL && _rotVec != NULL){
	    mt::Rotation tmpRot = _w2h.getRotation();
      mt::Point3 tmpPos = _w2h.getTranslation();
      _posVec->setValue(tmpPos.at(0), tmpPos.at(1), tmpPos.at(2));
      _rotVec->setValue(tmpRot.at(0), tmpRot.at(1), tmpRot.at(2), tmpRot.at(3));
    }
    if(applyForces ){
      vector<GuiNode*>* path = NULL;
      if( _radBttRobot0->isChecked() )
        path = &_guidingPath[0];
      else if( _radBttRobot1->isChecked() )
        path = &_guidingPath[1];

      if(path->size() != 0)
        setJumpForce(_w2tcp0);
    }
  }

  void TeleoperationWidget::teleoperation(){
    _callNewH = true; // To be sure to calculate newH the next time to sincronize();
	  mt::Transform aux(_haptic->getHIP());
    mt::Transform hip2h;
    hip2h.setTranslation(aux.getTranslation()*_tranScale);
    // Now scalling the rotation
    mt::Rotation auxrot = aux.getRotation();
    mt::Unit3 axis;
    mt::Scalar angle;
    auxrot.getAxisAngle(axis, angle);
    
    if( angle > M_PI ) angle -= 2*M_PI;
    if( abs(angle) < M_PI / _rotScale)
      auxrot.setAxisAngle(axis, angle*_rotScale);
    else
      auxrot.setAxisAngle(axis, M_PI);

    hip2h.setRotation(auxrot );

    mt::Transform tcp2w = _w2h * hip2h * _whip2tcp0;

    libSampling::SE3Conf se3conf;
    std::vector<KthReal> tmp(3);
    tmp.at(0)= tcp2w.getTranslation().at(0);
    tmp.at(1)= tcp2w.getTranslation().at(1);
    tmp.at(2)= tcp2w.getTranslation().at(2);
    se3conf.setPos(tmp);
    tmp.resize(4);
    tmp.at(0)= tcp2w.getRotation().at(0);
    tmp.at(1)= tcp2w.getRotation().at(1);
    tmp.at(2)= tcp2w.getRotation().at(2);
    tmp.at(3)= tcp2w.getRotation().at(3);
    se3conf.setOrient(tmp);

    unsigned int activeRob=0;
    if(_radBttRobot0->isChecked())
      activeRob = 0;
    else if(_radBttRobot1->isChecked())
      activeRob = 1;

    setGuideForce(tcp2w);

    if(_problem->wSpace()->getRobot(activeRob)->getIkine() == NULL)
      _problem->wSpace()->getRobot(activeRob)->Kinematics(se3conf);
    else{
      try{
        if(_problem->wSpace()->getRobot(activeRob)->getName() == "TX90"){
          vector<KthReal> target(se3conf.getCoordinates());
          RnConf& currConf = _problem->wSpace()->getRobot(activeRob)->getCurrentPos()->getRn();

          KthReal ifRig = 425*sin(currConf.getCoordinate(1))
                          + 425*sin(currConf.getCoordinate(1) + currConf.getCoordinate(2))
                          + 50;
          if(ifRig >= 0.) //Shoulder Lefty
            target.push_back(1);
          else
            target.push_back(0);

          if(currConf.getCoordinate(2) >= 0.) //Elbow Positive
            target.push_back(0);
          else
            target.push_back(1);

          if(currConf.getCoordinate(4) >= 0.) //Wrist Positive
            target.push_back(0);
          else
            target.push_back(1);

          RobConf& tmp =_problem->wSpace()->getRobot(activeRob)
                      ->InverseKinematics(target);

          _problem->wSpace()->getRobot(activeRob)->Kinematics(tmp);
        }else{
          RobConf& tmp =_problem->wSpace()->getRobot(activeRob)
                      ->InverseKinematics(se3conf.getCoordinates());

          _problem->wSpace()->getRobot(activeRob)->Kinematics(tmp);
        }
      }catch(InvKinEx &ex){
        std::cout << ex.what() << std::endl;
        
      }catch(...){
        std::cout << "Unexpected error" << std::endl;
      }
    }
  }

  void TeleoperationWidget::setupUI(){
    if (this->objectName().isEmpty())
        this->setObjectName(QString::fromUtf8("Form"));
    this->resize(200, 640);
    QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
    this->setSizePolicy(sizePolicy);
    this->setMinimumSize(QSize(200, 350));
    gridLayout_5 = new QGridLayout(this);
    gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
    verticalLayout_5 = new QVBoxLayout();
    verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
    groupBox = new QGroupBox(this);
    groupBox->setObjectName(QString::fromUtf8("groupBox"));
    gridLayout_7 = new QGridLayout(groupBox);
    gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
    verticalLayout_11 = new QVBoxLayout();
    verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
    gridLayout_4 = new QGridLayout();
    gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
    verticalLayout = new QVBoxLayout();
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    _cameraTop = new QPushButton(groupBox);
    _cameraTop->setObjectName(QString::fromUtf8("_cameraTop"));
    _cameraTop->setMinimumSize(QSize(25, 0));
    _cameraTop->setMaximumSize(QSize(50, 16777215));

    verticalLayout->addWidget(_cameraTop);

    _cameraBottom = new QPushButton(groupBox);
    _cameraBottom->setObjectName(QString::fromUtf8("_cameraBottom"));
    _cameraBottom->setMinimumSize(QSize(25, 0));
    _cameraBottom->setMaximumSize(QSize(50, 16777215));

    verticalLayout->addWidget(_cameraBottom);


    gridLayout_4->addLayout(verticalLayout, 0, 0, 1, 1);

    verticalLayout_4 = new QVBoxLayout();
    verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
    _cameraLeft = new QPushButton(groupBox);
    _cameraLeft->setObjectName(QString::fromUtf8("_cameraLeft"));
    _cameraLeft->setMinimumSize(QSize(25, 0));
    _cameraLeft->setMaximumSize(QSize(50, 16777215));

    verticalLayout_4->addWidget(_cameraLeft);

    _cameraRight = new QPushButton(groupBox);
    _cameraRight->setObjectName(QString::fromUtf8("_cameraRight"));
    _cameraRight->setMinimumSize(QSize(25, 0));
    _cameraRight->setMaximumSize(QSize(50, 16777215));

    verticalLayout_4->addWidget(_cameraRight);


    gridLayout_4->addLayout(verticalLayout_4, 0, 1, 1, 1);

    verticalLayout_10 = new QVBoxLayout();
    verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
    _cameraFront = new QPushButton(groupBox);
    _cameraFront->setObjectName(QString::fromUtf8("_cameraFront"));
    _cameraFront->setMinimumSize(QSize(25, 0));
    _cameraFront->setMaximumSize(QSize(50, 16777215));

    verticalLayout_10->addWidget(_cameraFront);

    _cameraRear = new QPushButton(groupBox);
    _cameraRear->setObjectName(QString::fromUtf8("_cameraRear"));
    _cameraRear->setMinimumSize(QSize(25, 0));
    _cameraRear->setMaximumSize(QSize(50, 16777215));

    verticalLayout_10->addWidget(_cameraRear);


    gridLayout_4->addLayout(verticalLayout_10, 0, 2, 1, 1);


    verticalLayout_11->addLayout(gridLayout_4);

    groupBox_3 = new QGroupBox(groupBox);
    groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
    gridLayout_3 = new QGridLayout(groupBox_3);
    gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
    verticalLayout_2 = new QVBoxLayout();
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
    label = new QLabel(groupBox_3);
    label->setObjectName(QString::fromUtf8("label"));
    label->setMinimumSize(QSize(10, 0));
    label->setMaximumSize(QSize(10, 20));

    horizontalLayout_5->addWidget(label);

    settingX = new QLineEdit(groupBox_3);
    settingX->setObjectName(QString::fromUtf8("settingX"));
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(settingX->sizePolicy().hasHeightForWidth());
    settingX->setSizePolicy(sizePolicy1);
    settingX->setMinimumSize(QSize(50, 20));
    settingX->setMaximumSize(QSize(50, 20));

    horizontalLayout_5->addWidget(settingX);

    label_7 = new QLabel(groupBox_3);
    label_7->setObjectName(QString::fromUtf8("label_7"));
    label_7->setMinimumSize(QSize(13, 0));
    label_7->setMaximumSize(QSize(13, 20));

    horizontalLayout_5->addWidget(label_7);

    settingRX = new QLineEdit(groupBox_3);
    settingRX->setObjectName(QString::fromUtf8("settingRX"));
    sizePolicy1.setHeightForWidth(settingRX->sizePolicy().hasHeightForWidth());
    settingRX->setSizePolicy(sizePolicy1);
    settingRX->setMinimumSize(QSize(50, 20));
    settingRX->setMaximumSize(QSize(50, 20));

    horizontalLayout_5->addWidget(settingRX);


    verticalLayout_2->addLayout(horizontalLayout_5);

    horizontalLayout_6 = new QHBoxLayout();
    horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
    label_2 = new QLabel(groupBox_3);
    label_2->setObjectName(QString::fromUtf8("label_2"));
    label_2->setMinimumSize(QSize(10, 0));
    label_2->setMaximumSize(QSize(10, 20));

    horizontalLayout_6->addWidget(label_2);

    settingY = new QLineEdit(groupBox_3);
    settingY->setObjectName(QString::fromUtf8("settingY"));
    sizePolicy1.setHeightForWidth(settingY->sizePolicy().hasHeightForWidth());
    settingY->setSizePolicy(sizePolicy1);
    settingY->setMinimumSize(QSize(50, 20));
    settingY->setMaximumSize(QSize(50, 20));

    horizontalLayout_6->addWidget(settingY);

    label_8 = new QLabel(groupBox_3);
    label_8->setObjectName(QString::fromUtf8("label_8"));
    label_8->setMinimumSize(QSize(13, 0));
    label_8->setMaximumSize(QSize(13, 20));

    horizontalLayout_6->addWidget(label_8);

    settingRY = new QLineEdit(groupBox_3);
    settingRY->setObjectName(QString::fromUtf8("settingRY"));
    sizePolicy1.setHeightForWidth(settingRY->sizePolicy().hasHeightForWidth());
    settingRY->setSizePolicy(sizePolicy1);
    settingRY->setMinimumSize(QSize(50, 20));
    settingRY->setMaximumSize(QSize(50, 20));

    horizontalLayout_6->addWidget(settingRY);


    verticalLayout_2->addLayout(horizontalLayout_6);

    horizontalLayout_7 = new QHBoxLayout();
    horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
    label_3 = new QLabel(groupBox_3);
    label_3->setObjectName(QString::fromUtf8("label_3"));
    label_3->setMinimumSize(QSize(10, 0));
    label_3->setMaximumSize(QSize(10, 20));

    horizontalLayout_7->addWidget(label_3);

    settingZ = new QLineEdit(groupBox_3);
    settingZ->setObjectName(QString::fromUtf8("settingZ"));
    sizePolicy1.setHeightForWidth(settingZ->sizePolicy().hasHeightForWidth());
    settingZ->setSizePolicy(sizePolicy1);
    settingZ->setMinimumSize(QSize(50, 20));
    settingZ->setMaximumSize(QSize(50, 20));

    horizontalLayout_7->addWidget(settingZ);

    label_9 = new QLabel(groupBox_3);
    label_9->setObjectName(QString::fromUtf8("label_9"));
    label_9->setMinimumSize(QSize(13, 0));
    label_9->setMaximumSize(QSize(13, 20));

    horizontalLayout_7->addWidget(label_9);

    settingRZ = new QLineEdit(groupBox_3);
    settingRZ->setObjectName(QString::fromUtf8("settingRZ"));
    sizePolicy1.setHeightForWidth(settingRZ->sizePolicy().hasHeightForWidth());
    settingRZ->setSizePolicy(sizePolicy1);
    settingRZ->setMinimumSize(QSize(50, 20));
    settingRZ->setMaximumSize(QSize(50, 20));

    horizontalLayout_7->addWidget(settingRZ);


    verticalLayout_2->addLayout(horizontalLayout_7);

    _setCamera = new QPushButton(groupBox_3);
    _setCamera->setObjectName(QString::fromUtf8("_setCamera"));

    verticalLayout_2->addWidget(_setCamera);


    gridLayout_3->addLayout(verticalLayout_2, 0, 0, 1, 1);


    verticalLayout_11->addWidget(groupBox_3);

    groupBox_4 = new QGroupBox(groupBox);
    groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
    gridLayout = new QGridLayout(groupBox_4);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    verticalLayout_3 = new QVBoxLayout();
    verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
    horizontalLayout_8 = new QHBoxLayout();
    horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
    label_4 = new QLabel(groupBox_4);
    label_4->setObjectName(QString::fromUtf8("label_4"));

    horizontalLayout_8->addWidget(label_4);

    currentX = new QLineEdit(groupBox_4);
    currentX->setObjectName(QString::fromUtf8("currentX"));
    currentX->setMinimumSize(QSize(50, 20));
    currentX->setMaximumSize(QSize(50, 20));

    horizontalLayout_8->addWidget(currentX);

    label_10 = new QLabel(groupBox_4);
    label_10->setObjectName(QString::fromUtf8("label_10"));
    label_10->setMinimumSize(QSize(16, 20));
    label_10->setMaximumSize(QSize(16, 20));

    horizontalLayout_8->addWidget(label_10);

    currentRX = new QLineEdit(groupBox_4);
    currentRX->setObjectName(QString::fromUtf8("currentRX"));
    currentRX->setMinimumSize(QSize(50, 20));
    currentRX->setMaximumSize(QSize(50, 20));

    horizontalLayout_8->addWidget(currentRX);


    verticalLayout_3->addLayout(horizontalLayout_8);

    horizontalLayout_9 = new QHBoxLayout();
    horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
    label_5 = new QLabel(groupBox_4);
    label_5->setObjectName(QString::fromUtf8("label_5"));

    horizontalLayout_9->addWidget(label_5);

    currentY = new QLineEdit(groupBox_4);
    currentY->setObjectName(QString::fromUtf8("currentY"));
    currentY->setMinimumSize(QSize(50, 20));
    currentY->setMaximumSize(QSize(50, 20));

    horizontalLayout_9->addWidget(currentY);

    label_12 = new QLabel(groupBox_4);
    label_12->setObjectName(QString::fromUtf8("label_12"));
    label_12->setMinimumSize(QSize(16, 20));
    label_12->setMaximumSize(QSize(16, 20));

    horizontalLayout_9->addWidget(label_12);

    currentRY = new QLineEdit(groupBox_4);
    currentRY->setObjectName(QString::fromUtf8("currentRY"));
    currentRY->setMinimumSize(QSize(50, 20));
    currentRY->setMaximumSize(QSize(50, 20));

    horizontalLayout_9->addWidget(currentRY);


    verticalLayout_3->addLayout(horizontalLayout_9);

    horizontalLayout_10 = new QHBoxLayout();
    horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
    label_6 = new QLabel(groupBox_4);
    label_6->setObjectName(QString::fromUtf8("label_6"));

    horizontalLayout_10->addWidget(label_6);

    currentZ = new QLineEdit(groupBox_4);
    currentZ->setObjectName(QString::fromUtf8("currentZ"));
    currentZ->setMinimumSize(QSize(50, 20));
    currentZ->setMaximumSize(QSize(50, 20));

    horizontalLayout_10->addWidget(currentZ);

    label_11 = new QLabel(groupBox_4);
    label_11->setObjectName(QString::fromUtf8("label_11"));
    label_11->setMinimumSize(QSize(16, 20));
    label_11->setMaximumSize(QSize(16, 20));

    horizontalLayout_10->addWidget(label_11);

    currentRZ = new QLineEdit(groupBox_4);
    currentRZ->setObjectName(QString::fromUtf8("currentRZ"));
    currentRZ->setMinimumSize(QSize(50, 20));
    currentRZ->setMaximumSize(QSize(50, 20));

    horizontalLayout_10->addWidget(currentRZ);


    verticalLayout_3->addLayout(horizontalLayout_10);

    _getCamera = new QPushButton(groupBox_4);
    _getCamera->setObjectName(QString::fromUtf8("_getCamera"));

    verticalLayout_3->addWidget(_getCamera);


    gridLayout->addLayout(verticalLayout_3, 0, 0, 1, 1);


    verticalLayout_11->addWidget(groupBox_4);


    gridLayout_7->addLayout(verticalLayout_11, 0, 0, 1, 1);


    verticalLayout_5->addWidget(groupBox);

    groupBox_2 = new QGroupBox(this);
    groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
    gridLayout_8 = new QGridLayout(groupBox_2);
    gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
    verticalLayout_8 = new QVBoxLayout();
    verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
    horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    _groupRobots = new QGroupBox(groupBox_2);
    _groupRobots->setObjectName(QString::fromUtf8("_groupRobots"));
    gridLayout_2 = new QGridLayout(_groupRobots);
    gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
    verticalLayout_12 = new QVBoxLayout();
    verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
    _radBttRobot0 = new QRadioButton(_groupRobots);
    _radBttRobot0->setObjectName(QString::fromUtf8("_radBttRobot0"));
    _radBttRobot0->setChecked(true);

    verticalLayout_12->addWidget(_radBttRobot0);

    _radBttRobot1 = new QRadioButton(_groupRobots);
    _radBttRobot1->setObjectName(QString::fromUtf8("_radBttRobot1"));

    verticalLayout_12->addWidget(_radBttRobot1);


    gridLayout_2->addLayout(verticalLayout_12, 0, 0, 1, 1);


    horizontalLayout->addWidget(_groupRobots);

    groupBox_5 = new QGroupBox(groupBox_2);
    groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
    gridLayout_6 = new QGridLayout(groupBox_5);
    gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
    verticalLayout_9 = new QVBoxLayout();
    verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
    _radWorld = new QRadioButton(groupBox_5);
    _radWorld->setObjectName(QString::fromUtf8("_radWorld"));

    verticalLayout_9->addWidget(_radWorld);

    _radCamera = new QRadioButton(groupBox_5);
    _radCamera->setObjectName(QString::fromUtf8("_radCamera"));
    _radCamera->setChecked(true);

    verticalLayout_9->addWidget(_radCamera);


    gridLayout_6->addLayout(verticalLayout_9, 0, 0, 1, 1);


    horizontalLayout->addWidget(groupBox_5);


    verticalLayout_8->addLayout(horizontalLayout);

    verticalLayout_6 = new QVBoxLayout();
    verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
    _lblTransScale = new QLabel(groupBox_2);
    _lblTransScale->setObjectName(QString::fromUtf8("_lblTransScale"));

    verticalLayout_6->addWidget(_lblTransScale);

    _sliderTransScale = new QSlider(groupBox_2);
    _sliderTransScale->setObjectName(QString::fromUtf8("_sliderTransScale"));
    _sliderTransScale->setMinimumSize(QSize(100, 20));
    _sliderTransScale->setMinimum(1);
    _sliderTransScale->setMaximum(100);
    _sliderTransScale->setValue(10);
    _sliderTransScale->setOrientation(Qt::Horizontal);

    verticalLayout_6->addWidget(_sliderTransScale);


    verticalLayout_8->addLayout(verticalLayout_6);

    verticalLayout_7 = new QVBoxLayout();
    verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
    _lblRotScale = new QLabel(groupBox_2);
    _lblRotScale->setObjectName(QString::fromUtf8("_lblRotScale"));

    verticalLayout_7->addWidget(_lblRotScale);

    _sliderRotScale = new QSlider(groupBox_2);
    _sliderRotScale->setObjectName(QString::fromUtf8("_sliderRotScale"));
    _sliderRotScale->setMinimum(1);
    _sliderRotScale->setMaximum(20);
    _sliderRotScale->setPageStep(5);
    _sliderRotScale->setValue(10);
    _sliderRotScale->setOrientation(Qt::Horizontal);

    verticalLayout_7->addWidget(_sliderRotScale);


    verticalLayout_8->addLayout(verticalLayout_7);

    horizontalLayout_4 = new QHBoxLayout();
    horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
    _startOperation = new QPushButton(groupBox_2);
    _startOperation->setObjectName(QString::fromUtf8("_startOperation"));

    horizontalLayout_4->addWidget(_startOperation);

    _stopOperation = new QPushButton(groupBox_2);
    _stopOperation->setObjectName(QString::fromUtf8("_stopOperation"));

    horizontalLayout_4->addWidget(_stopOperation);


    verticalLayout_8->addLayout(horizontalLayout_4);


    gridLayout_8->addLayout(verticalLayout_8, 0, 0, 1, 1);


    verticalLayout_5->addWidget(groupBox_2);


    gridLayout_5->addLayout(verticalLayout_5, 0, 0, 1, 1);

    this->setWindowTitle(QApplication::translate("Form", "Teleoperation", 0, QApplication::UnicodeUTF8));
    groupBox->setTitle(QApplication::translate("Form", "Cameras", 0, QApplication::UnicodeUTF8));
    _cameraTop->setText(QApplication::translate("Form", "Top", 0, QApplication::UnicodeUTF8));
    _cameraBottom->setText(QApplication::translate("Form", "Bottom", 0, QApplication::UnicodeUTF8));
    _cameraLeft->setText(QApplication::translate("Form", "Left", 0, QApplication::UnicodeUTF8));
    _cameraRight->setText(QApplication::translate("Form", "Right", 0, QApplication::UnicodeUTF8));
    _cameraFront->setText(QApplication::translate("Form", "Front", 0, QApplication::UnicodeUTF8));
    _cameraRear->setText(QApplication::translate("Form", "Rear", 0, QApplication::UnicodeUTF8));
    groupBox_3->setTitle(QApplication::translate("Form", "Free", 0, QApplication::UnicodeUTF8));
    label->setText(QApplication::translate("Form", "X", 0, QApplication::UnicodeUTF8));
    label_7->setText(QApplication::translate("Form", "Rx", 0, QApplication::UnicodeUTF8));
    label_2->setText(QApplication::translate("Form", "Y", 0, QApplication::UnicodeUTF8));
    label_8->setText(QApplication::translate("Form", "Ry", 0, QApplication::UnicodeUTF8));
    label_3->setText(QApplication::translate("Form", "Z", 0, QApplication::UnicodeUTF8));
    label_9->setText(QApplication::translate("Form", "Rz", 0, QApplication::UnicodeUTF8));
    _setCamera->setText(QApplication::translate("Form", "Set Camera", 0, QApplication::UnicodeUTF8));
    groupBox_4->setTitle(QApplication::translate("Form", "Current", 0, QApplication::UnicodeUTF8));
    label_4->setText(QApplication::translate("Form", "X", 0, QApplication::UnicodeUTF8));
    label_10->setText(QApplication::translate("Form", "Rx", 0, QApplication::UnicodeUTF8));
    label_5->setText(QApplication::translate("Form", "Y", 0, QApplication::UnicodeUTF8));
    label_12->setText(QApplication::translate("Form", "Ry", 0, QApplication::UnicodeUTF8));
    label_6->setText(QApplication::translate("Form", "Z", 0, QApplication::UnicodeUTF8));
    label_11->setText(QApplication::translate("Form", "Rz", 0, QApplication::UnicodeUTF8));
    _getCamera->setText(QApplication::translate("Form", "Get Camera", 0, QApplication::UnicodeUTF8));
    groupBox_2->setTitle(QApplication::translate("Form", "Teleoperation", 0, QApplication::UnicodeUTF8));
    _groupRobots->setTitle(QApplication::translate("Form", "Robots", 0, QApplication::UnicodeUTF8));
    _radBttRobot0->setText(QApplication::translate("Form", "0", 0, QApplication::UnicodeUTF8));
    _radBttRobot1->setText(QApplication::translate("Form", "1", 0, QApplication::UnicodeUTF8));
    groupBox_5->setTitle(QApplication::translate("Form", "Method", 0, QApplication::UnicodeUTF8));
    _radWorld->setText(QApplication::translate("Form", "World", 0, QApplication::UnicodeUTF8));
    _radCamera->setText(QApplication::translate("Form", "Camera", 0, QApplication::UnicodeUTF8));
    _lblTransScale->setText(QApplication::translate("Form", "Translational Scale: 1.0", 0, QApplication::UnicodeUTF8));
    _lblRotScale->setText(QApplication::translate("Form", "Rotational Scale: 1.0", 0, QApplication::UnicodeUTF8));
    _startOperation->setText(QApplication::translate("Form", "Start", 0, QApplication::UnicodeUTF8));
    _stopOperation->setText(QApplication::translate("Form", "Stop", 0, QApplication::UnicodeUTF8));
  }
}
