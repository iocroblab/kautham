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

/* Author: Nestor Garcia Hidalgo */


#include "ivkinUR5.h"
#include "UR5_kinematics.h"

IvKinUR5::IvKinUR5(Robot* const rob): Kautham::InverseKinematic(rob){
    _target.resize(7);  // This contains the pos and quaternion as a vector
    _eulPos.resize(6);
    _robConf.setRn(6);
    _robLay.resize(3);

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

IvKinUR5::~IvKinUR5(){

}

bool IvKinUR5::solve(){
    bool shoulder(true), elbow(true), wrist(true);
    if (_target.size() > 7){
      shoulder = _target.at(7) == shoulder_left;
      elbow = _target.at(8) == elbow_up;
      wrist = _target.at(9) == wrist_in;
    }

    _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1),
                                           _target.at(2)));
    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4),
                                          _target.at(5), _target.at(6) ));

    if (UR5_inv_kin(_targetTrans, shoulder, wrist, elbow, _result)) {
        double control [6];
        UR5_controls(control,_result);
        cout << "Joint values are:" << endl;
        for (int j = 0; j < 6; j++) {
            cout << "  theta" << j+1 << "=" << _result[j]
                 << " (" << control[j] << ")" << endl;
        }
        return true;
    } else {
        cout << "Inverse kinematics failed" << endl;
        return false;
    }
}

bool IvKinUR5::setParameters(){
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
            _UR5Conf.sh = shoulder_left;
          else
            _UR5Conf.sh = shoulder_right;
        }else
          return false;

        it = _parameters.find("Elbow Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _UR5Conf.el = elbow_up;
          else
            _UR5Conf.el = elbow_down;
        }else
          return false;

        it = _parameters.find("Wrist Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)
            _UR5Conf.wr = wrist_in;
          else
            _UR5Conf.wr = wrist_out;
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
      _target.at(7) = _UR5Conf.sh == shoulder_right ? 1. : 0. ;
      _target.at(8) = _UR5Conf.el == elbow_up ? 1. : 0. ;
      _target.at(9) = _UR5Conf.wr == wrist_in ? 1. : 0. ;
        }

      }catch(...){
        return false;
      }
      return true;
}

void IvKinUR5::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
    //loads the target: the tcp transform
    _target.clear();
    for(unsigned i =0; i< target.size(); i++)
        _target.push_back(target.at(i));
}
