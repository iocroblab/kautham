
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

/* Author: Alexander Perez, Jan Rosell and Nestor Garcia Hidalgo */
 
 
#include "ivkintx90.h"
#include <problem/robot.h>


using namespace TXrobot;

  IvKinTx90::IvKinTx90(Robot* const rob) : Kautham::InverseKinematic(rob){

    TXtype tipus = TXrobot::TX90;
    _tx = new TXRobot(tipus);
    _target.resize(7);  // This contains the pos and quaternion as a vector
    _result = new Vect6(6);
    _eulPos.resize(6);
    _robConf.setRn(6);
    _robLay.resize(3);

    // Offset from the modeled home to the real home.
    mt::Point3 Offset(0., 0., 478.);
    _rHomeOffset.setTranslation(Offset);

    addParameter("Px", _eulPos.at(0));
    addParameter("Py", _eulPos.at(1));
    addParameter("Pz", _eulPos.at(2));
    addParameter("Rx", _eulPos.at(3));
    addParameter("Ry", _eulPos.at(4));
    addParameter("Rz", _eulPos.at(5));

    addParameter("Shoulder Lefty?", 1. );
    addParameter("Elbow Positive?", 1.);
    addParameter("Wrist Positive?", 1.);
  }

  IvKinTx90::~IvKinTx90(){
    delete _result;
    delete _tx;
  }

   void IvKinTx90::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
	   //loads the target: the tcp transform
    _target.clear();
    for(unsigned i =0; i< target.size(); i++)
      _target.push_back(target.at(i));  
	  
	  //completes the target with configuration parameters information
	KthReal ifRig = 425*sin(masterconf[1])
                          + 425*sin(masterconf[1] + masterconf[2])
                          + 50;
    if(ifRig >= 0.) //Shoulder Lefty
      _target.push_back(slefty);
    else
      _target.push_back(srighty);

    if(masterconf[2] >= 0.) //Elbow Positive
      _target.push_back(epositive);
    else
      _target.push_back(enegative);

	 
    if(masterconf[4] >= 0.) //Wrist Positive
	{
		if(maintainSameWrist) _target.push_back(wpositive);
		else _target.push_back(wnegative); //return the oposite
	}
    else
	{
      if(maintainSameWrist) _target.push_back(wnegative);
	  else _target.push_back(wpositive);//return the oposite
	}
	
 }

  RobLayout& IvKinTx90::getRobLayout(vector<KthReal> &target){
    //completes the target with configuration parameters information
	  KthReal ifRig = 425*sin(target[1])
                          + 425*sin(target[1] + target[2])
                          + 50;
    _robLay[0] = ifRig >= 0. ? false : true ;
 
    _robLay[1] = target[2] >= 0. ? true : false ;

    _robLay[2] = target[4] >= 0. ? true : false ;

    return _robLay;
  }


  bool IvKinTx90::solve(mt::Transform& tcp, const Vect6 &current){
    config conf;

    KthReal ifRig = 425*sin(current[1])
                          + 425*sin(current[1] + current[2])
                          + 50;
    if(ifRig >= 0.) //Shoulder Lefty
      conf.sh = slefty;
    else
      conf.sh = srighty;

    if(current[2] >= 0.) //Elbow Positive
      conf.el = epositive;
    else
      conf.el = enegative;

    if(current[4] >= 0.) //Wrist Positive
      conf.wr = wpositive;
    else
      conf.wr = wnegative;

    return solve(tcp, conf );
  }


  bool IvKinTx90::solve(mt::Transform& tcp, const config& conf, config* solution, Vect6* qNear ){
    _targetTrans = tcp;
    _txConf = conf;
    Vect6 results(6);

    // The model developed in the TXRobot class has the origin as the real robot
    // and it is located in the shoulder articulation. This frame corresponds to
    // the our Home Reference Frame translated in z the 478 mm. The Home
    // Reference Frame is located in the World frame as it is expresed in the
    // SE3 part of the robot's configuration RobConf.


    mt::Transform w2realHome = _robot->getHomeTransform() * _rHomeOffset ;

    mt::Transform targetInHome = w2realHome.inverse() * _targetTrans;

    TXerror tmpError;
    if(solution != NULL && qNear != NULL)
      tmpError = _tx->invKin(targetInHome, results ,_txConf, *solution, *qNear);
    else
      tmpError = _tx->invKin(targetInHome, results ,_txConf);
    //else
    //  tmpError = _tx->invKin(targetInHome, results ,_txConf, *solution, *qNear);

    if( tmpError == TXrobot::SUCCESS){
      std::vector<KthReal> tmp(6);
      for(int i = 0; i < 6; i++ )
        tmp.at(i) = results[i];

      _robConf = *(_robot->getCurrentPos());
      _robConf.setRn(tmp);
      return true;
    }
    return false;
  }

  bool IvKinTx90::solve(){
    // Setting the Scene target from the _taget vector. In this case, the _target vector
    // contains the se3 coordinates (pos , quaternion).
    if(_target.size() > 7 /*&& _target.size() <= 10*/){
      //This next 3 parameters define the configuration

      if(_target.at(7) == slefty)  // Shoulder Lefty //antes ==1, ok
        _txConf.sh = slefty;
      else
        _txConf.sh = srighty;

      if(_target.at(8) == epositive)  // Elbow Positive //antes ==0, ok
        _txConf.el = epositive;
      else
        _txConf.el = enegative;

      if(_target.at(9) == wpositive)  // Wrist Positive   //correccion antes ==1!! KO
        _txConf.wr = wpositive;
      else
        _txConf.wr = wnegative;
    }
    _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1),
                                       _target.at(2)));
    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4),
                                          _target.at(5), _target.at(6) ));

    return solve(_targetTrans, _txConf);
  }

  bool IvKinTx90::setParameters(){
    try{
        HASH_S_K::iterator it = _parameters.find("Px");
        if(it != _parameters.end())
          _eulPos.at(0) = it->second;
        else
          return false;

        it = _parameters.find("Py");
        if(it != _parameters.end())
          _eulPos.at(1) = it->second;
        else
          return false;

        it = _parameters.find("Pz");
        if(it != _parameters.end())
          _eulPos.at(2) = it->second;
        else
          return false;

        it = _parameters.find("Rx");
        if(it != _parameters.end())
          _eulPos.at(3) = it->second;
        else
          return false;

        it = _parameters.find("Ry");
        if(it != _parameters.end())
          _eulPos.at(4) = it->second;
        else
          return false;

        it = _parameters.find("Rz");
        if(it != _parameters.end())
          _eulPos.at(5) = it->second;
        else
          return false;

        it = _parameters.find("Shoulder Lefty?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _txConf.sh = slefty;
          else
            _txConf.sh = srighty;
        }else
          return false;

        it = _parameters.find("Elbow Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _txConf.el = epositive;
          else
            _txConf.el = enegative;
        }else
          return false;

        it = _parameters.find("Wrist Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _txConf.wr = wpositive;
          else
            _txConf.wr = wnegative;
        }else
          return false;

        _targetTrans.setTranslation(mt::Point3(_eulPos.at(0), _eulPos.at(1), _eulPos.at(2)));
        _targetTrans.setRotation(mt::Rotation(mt::degToRad(_eulPos.at(5)), 
                                 mt::degToRad(_eulPos.at(4)), 
                                 mt::degToRad(_eulPos.at(3))));

        for( int i = 0; i < 3; i++)
          _target.at(i) = _targetTrans.getTranslation().at(i);

        for( int i = 3; i < 7; i++)
          _target.at(i) = _targetTrans.getRotation().at(i-3);

		if(_target.size() > 7){
      _target.at(7) = _txConf.sh == srighty ? 1. : 0. ;
      _target.at(8) = _txConf.el == epositive ? 1. : 0. ;
      _target.at(9) = _txConf.wr == wpositive ? 1. : 0. ;
		}

      }catch(...){
        return false;
      }
      return true;
  }
