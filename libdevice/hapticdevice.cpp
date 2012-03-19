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

#include "hapticdevice.h"
#include <boost/bind.hpp>


namespace libDevice{

  HapticDevice::HapticDevice(string name, unsigned int upPeriod):Device(name){

    //// Set initial values to the force to be send
    //_force.time_stamp.assign(ioc_comm::cal_time_stamp());
    //_force._data.at(0) = 0.0;  _force._data.at(1) = 0.0;
    //_force._data.at(2) = 0.0;  _force._data.at(3) = 0.0;
    //_force._data.at(4) = 0.0;  _force._data.at(5) = 0.0;

    addIncomingParameter("X", (KthReal)0.0);
    addIncomingParameter("Y", (KthReal)0.0);
    addIncomingParameter("Z", (KthReal)0.0);
    addIncomingParameter("Alpha", (KthReal)0.0);
    addIncomingParameter("Beta", (KthReal)0.0);
    addIncomingParameter("Gamma", (KthReal)0.0);
    addIncomingParameter("Button", (KthReal)0.0);

    addParameter("Fx", (KthReal)0.0);
    addParameter("Fy", (KthReal)0.0);
    addParameter("Fz", (KthReal)0.0);
    addParameter("Tx", (KthReal)0.0);
    addParameter("Ty", (KthReal)0.0);
    addParameter("Tz", (KthReal)0.0);

    _updatingPeriod = upPeriod;
    //_client = NULL;
    _updateDeviceTimer.setInterval(_updatingPeriod);
    _url = "";
    _port = "";
    _tcp = true;
    _fMax = 4.; _tMax = 2.;
    _emmitUpdated = true;
#ifdef KAUTHAM_USE_OPENHAPTICS
    _hapticDevice = NULL;
    _directReading = true;
#else
    _directReading = false;
#endif
    _tcp = true;
    _deviceLimits[0][0] = -200.;  _deviceLimits[0][1] = 200.;
    _deviceLimits[1][0] = -30.;   _deviceLimits[1][1] = 370.;
    _deviceLimits[2][0] = -110.;  _deviceLimits[2][1] = 100.;
    _deviceLimits[3][0] = -90.;  _deviceLimits[3][1] = 90.;
    _deviceLimits[4][0] = -60.;  _deviceLimits[4][1] = 60.;
    _deviceLimits[5][0] = -60.;  _deviceLimits[5][1] = 60.;


    QObject::connect(&_updateDeviceTimer, SIGNAL(timeout()), this, SLOT(update()));
  }

  HapticDevice::~HapticDevice(){
    try{
#ifdef KAUTHAM_USE_OPENHAPTICS
      if( _hapticDevice != NULL ){
        _hapticDevice->stop();
        delete _hapticDevice;
      }
#endif
      //if(_client != NULL ){
      //  _client->close();
      //  delete _client;
      //}
    }catch(...){}

  }

  bool HapticDevice::connect(string url, string port, bool tcp){
    _url= url;
    _port = port;
    _tcp = tcp;
    try{
      if( _directReading ){
#ifdef KAUTHAM_USE_OPENHAPTICS
        bool init=false;
        _hapticDevice = new haptic::Haptic(init);

        if(init){
          _hapticDevice->calibrate();
          _hapticDevice->start();
		  _updateDeviceTimer.start();
		  return true;
        }
#endif
      }else{
      //  if(_client == NULL){
      //    _client = new ioc_comm::Client(_url, _port, ioc_comm::HAPTIC, 1.0, 6);
      //    _sendingData.push_back(_force);
      //  }
      //  _client->start();
      //  _client->setSendingData(_sendingData);
		    //_updateDeviceTimer.start();
		    //return true;
      }
      
    }catch(...){ }
	 
	return false;
  }

  bool HapticDevice::disconnect(){
    try{
#ifdef KAUTHAM_USE_OPENHAPTICS
      if(_directReading)
        _hapticDevice->stop();
      else
#endif
        //_client->close();

      _updateDeviceTimer.stop();
      return true;
    }catch(...){
      return false;
    }
  }

  void HapticDevice::update(){
    try{
      if( _directReading ){
#ifdef KAUTHAM_USE_OPENHAPTICS
        mt::Vector3 pos;
        mt::Vector3 rot;
        _hapticDevice->getPosition(_hipTransform);
        pos = _hipTransform.getTranslation();
        _hipTransform.getRotation().getYpr(rot.at(0), rot.at(1), rot.at(2));

        setIncomingParameter("X", pos.at(0));
        setIncomingParameter("Y", pos.at(1));
        setIncomingParameter("Z", pos.at(2));
        setIncomingParameter("Alpha", rot.at(0));
        setIncomingParameter("Beta", rot.at(1));
        setIncomingParameter("Gamma", rot.at(2));

        if(_hapticDevice->getButtom())
          _buttonPressed = true;
        else
          _buttonPressed = false;

        setIncomingParameter("Button", _buttonPressed ? 1. : 0.);

        Vect6 _forcemt(6,0.);
        for(unsigned int i = 0; i < 6; i++)
          _forcemt[i] = _force._data.at(i);

        for(unsigned int i = 0; i < 3; i++){
          if(_forcemt[i] > _fMax ) _forcemt[i] = _fMax ;
          if(_forcemt[i] < -_fMax ) _forcemt[i] = -_fMax ;
        }
        for(unsigned int i = 3; i < 6; i++){
          if(_forcemt[i] > _tMax ) _forcemt[i] = _tMax ;
          if(_forcemt[i] < -_tMax ) _forcemt[i] = -_tMax ;
        }

        _hapticDevice->setForce(_forcemt);
#endif
      }else{
        //_client->getServerData(_serverData);
        //if(_serverData.size() > 0 ){
        //  ioc_comm::baseData& tmp = _serverData[0];

        //  _position.time_stamp = tmp.time_stamp;
        //  for(unsigned int i = 0; i < 6; i++)
        //    _position._data.at(i) = tmp._data[i];

        //  mt::Point3 mtpoint(_position._data.at(0), _position._data.at(1), _position._data.at(2) );
        //  mt::Rotation mtrot(_position._data.at(3), _position._data.at(4), _position._data.at(5));
        //  _hipTransform.setRotation(mtrot);
        //  _hipTransform.setTranslation(mtpoint);

        //  if(tmp._data.at(6) > 0)
        //    _buttonPressed = true;
        //  else
        //    _buttonPressed = false;

        //  setIncomingParameter("X", _position._data.at(0));
        //  setIncomingParameter("Y", _position._data.at(1));
        //  setIncomingParameter("Z", _position._data.at(2));
        //  setIncomingParameter("Alpha", _position._data.at(3));
        //  setIncomingParameter("Beta", _position._data.at(4));
        //  setIncomingParameter("Gamma", _position._data.at(5));
        //  setIncomingParameter("Button", tmp._data.at(6));
        //}

        //// Copyng the _force variable to the sendingData vector.
        //ioc_comm::baseData& force = _sendingData.at(0);
        //for(unsigned int i = 0; i< 6; i++)
        //  force._data.at(i)= _force._data.at(i);
        //force.time_stamp.assign(_force.time_stamp);

        //_sendingData.at(0) = force;
        //_client->setSendingData(_sendingData);
      }
      _updateDeviceTimer.start();
      
    }catch(...){}

    if(_emmitUpdated)
      emit updated();
  }

  void HapticDevice::setSE3Force(KthReal forces[]){
    //try{
    //  for(unsigned int i = 0; i< 6; i++)
    //    _force._data.at(i)= forces[i];

    //  _force.time_stamp.assign(ioc_comm::cal_time_stamp());
    //}catch(...){ }
  }

  bool HapticDevice::setParameters(){
    try{
      //HASH_S_K::iterator it = _parameters.find("Fx");
      //if(it != _parameters.end())
      //  _force._data.at(0) = (int)it->second;
      //else
      //  return false;

      //it = _parameters.find("Fy");
      //if(it != _parameters.end())
      //  _force._data.at(1) = (int)it->second;
      //else
      //  return false;

      //it = _parameters.find("Fz");
      //if(it != _parameters.end())
      //  _force._data.at(2) = (int)it->second;
      //else
      //  return false;

      //it = _parameters.find("Tx");
      //if(it != _parameters.end())
      //  _force._data.at(3) = (int)it->second;
      //else
      //  return false;

      //it = _parameters.find("Ty");
      //if(it != _parameters.end())
      //  _force._data.at(4) = (int)it->second;
      //else
      //  return false;

      //it = _parameters.find("Tz");
      //if(it != _parameters.end())
      //  _force._data.at(5) = (int)it->second;
      //else
      //  return false;
    }catch(...){}
    return true;
  }

}
