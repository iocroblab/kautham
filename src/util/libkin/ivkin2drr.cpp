/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Alexander Perez, Jan Rosell */

 
 
#include <kautham/util/libkin/ivkin2drr.h>
#include <kautham/problem/robot.h>

  
IvKin2DRR::IvKin2DRR(Robot* const rob) : Kautham::InverseKinematic(rob){

    _target.resize(3);  // This contains the X, Y and Lefty parameters.
    _tcp[0] = 0.;
    _tcp[1] = 0.;
    _llong[0] = rob->getLink(2)->getA();
    _llong[1] = rob->getLink(3)->getA();
    addParameter("Px", _tcp[0]);
    addParameter("Py", _tcp[1]);
    addParameter("Shoulder Lefty?", 0. );
    _robLay.resize(1);
    _robLay[0] = false;
  }

  IvKin2DRR::~IvKin2DRR(){

  }


  bool IvKin2DRR::solve(){
    try{
      _tcp[0] = _target.at(0);
      _tcp[1] = _target.at(1);

      if( _target.size() > 7 ) _robLefty = _target[7] == 1. ? true : false ;
      
      _targetTrans.setTranslation(mt::Point3(_tcp[0],_tcp[1], 0.));
      _targetTrans.setRotation(mt::Rotation(0., 0., 0., 1. ));
      
      double b2, q1, l12, l22, q2, phi11, phi12, phi2, phi22, phi21;
      b2= q1= l12= l22= q2= phi11= phi12= phi2= phi22= phi21= 0.;

      b2= _tcp[0]*_tcp[0] + _tcp[1]*_tcp[1]; //  by the Pythagorean theorem
      if(b2 < (_llong[0]+_llong[1])*(_llong[0]+_llong[1])){
        q1 = atan2(_tcp[1], _tcp[0]);
        l12 = _llong[0]*_llong[0];
        l22 =  _llong[1]*_llong[1];
        double tmp = (l12 - l22 + b2)/(2.*_llong[0]*sqrt(b2));
        if (tmp < -1. || tmp > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
        q2 = acos(tmp); //(by the law of cosines)
        phi11 = q1 + q2;                                  //(I know you can handle addition)
        phi12 = q1 - q2;
        phi11 -= phi11 > M_PI ? 2. * M_PI : 0;
        phi12 -= phi12 > M_PI ? 2. *M_PI : 0;
        phi11 += phi11 < -M_PI ? 2. *M_PI : 0;
        phi12 += phi12 < -M_PI ? 2. *M_PI : 0;
        tmp = (l12 + l22 - b2)/(2.*_llong[0]*_llong[1]);
        if (tmp < -1. || tmp > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
        phi2 = acos(tmp); //(by the law of cosines)
        phi22 = M_PI - phi2;
        phi21 = -M_PI + phi2;
      }else{
        phi11 = phi12 = atan2(_tcp[1], _tcp[0]);
        phi21 = phi22 = 0.;
        //_robLefty = !_robLefty;
      }

      vector<double> q;
      if( _robLefty ){
        q.push_back( phi11 );
        q.push_back( phi21 );
      }else{  // robot in righty configuration
        q.push_back( phi12 );
        q.push_back( phi22 );
      }
      _robConf.setRn(q); 
      return true;
    }catch(...){}
    return false;
  }

  bool IvKin2DRR::setParameters(){
    try{
        HASH_S_K::iterator it = _parameters.find("Px");
        if(it != _parameters.end())
          _target.at(0) = it->second;
        else
          return false;

        it = _parameters.find("Py");
        if(it != _parameters.end())
          _target.at(1) = it->second;
        else
          return false;

        it = _parameters.find("Shoulder Lefty?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _robLefty = true;
          else
            _robLefty = false;
        }else
          return false;

      }catch(...){
        std::cout << "Some throuble with the name or amount of parameters.\n" ;
        return false;
      }
      return true;
  }

  RobLayout& IvKin2DRR::getRobLayout(vector<double> &target){
    if( target.size() > 0 && target[1] <= 0. )
      _robLay[0] = true;
    else
      _robLay[0] = false;

    return _robLay;
  }

