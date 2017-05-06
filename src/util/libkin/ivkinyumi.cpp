
/*************************************************************************\
   Copyright 2017 Institute of Industrial and Control Engineering (IOC)
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

/* Author: Josep-Arnau Claret Robert */

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <kautham/util/libkin/ivkinyumi.h>
#include <kautham/problem/robot.h>

#include <kautham/util/libkin/YumiKinematics.h>


IvKinYumi::IvKinYumi(Robot* const rob): Kautham::InverseKinematic(rob){

  _target.resize(8);  // This contains the pose and quaternion as a vector
  _eulPos.resize(6);
  _robConf.setRn(7);

  _redundantJoint = 0.0;

  addParameter("Px", _eulPos.at(0));
  addParameter("Py", _eulPos.at(1));
  addParameter("Pz", _eulPos.at(2));
  addParameter("Rx", _eulPos.at(3));
  addParameter("Ry", _eulPos.at(4));
  addParameter("Rz", _eulPos.at(5));

  addParameter("Redundant joint", _redundantJoint);
}

IvKinYumi::~IvKinYumi(){

}

bool IvKinYumi::solve(){

  // Set target pose
  _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1), _target.at(2)));
  _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));

  // Set redundant joint
  double redundantJoint = 0.0;
  if (_target.size() > 7)   redundantJoint = _target.at(7);

//  std::cout << "Yumi IK values:";
//  std::cout << "_targetTrans p: " << _targetTrans.getTranslation().at(0) << " "
//                                  << _targetTrans.getTranslation().at(1) << " "
//                                  << _targetTrans.getTranslation().at(2) << "\n";
//  std::cout << "redundant_joint: " << redundantJoint << "\n";


  // Convert to Eigen
  Eigen::Matrix4f desiredPose(Eigen::Matrix4f::Zero());
  // Position
  for (unsigned int i=0; i<3; ++i)  desiredPose(i,3) = _targetTrans.getTranslation().at(i);
  desiredPose(3,3) = 1.0;
  // Orientation
  mt::Matrix3x3 rot = _targetTrans.getRotation().getMatrix();
  for (unsigned int i=0; i<3; ++i)
      for (unsigned int j=0; j<3; ++j)
          desiredPose(i,j) = rot[i][j];

  YumiKinematics YumiKinSolver;
  std::vector< std::vector<double> > yumiIkSolutions;
  yumiIkSolutions = YumiKinSolver.AnalyticalIKSolver(desiredPose, redundantJoint);

//  std::cout << "Yumi IK solutions: " << yumiIkSolutions.size() << "\n";

  // Solve IK
  if (yumiIkSolutions.size() > 0) {

    // Select a solution
    std::vector<double> ikSolution = yumiIkSolutions[0];

    std::cout << "Yumi IK solution: ";
    for (unsigned int i=0; i<ikSolution.size(); ++i)  std::cout << ikSolution[i] << " ";
    std::cout << std::endl;

    bool solutionOK = true;
    for (unsigned int i=0; solutionOK && (i<ikSolution.size()); ++i){
      // f != f will be true if f is NaN
      solutionOK = !(ikSolution[i] != ikSolution[i]);
    }
    if (!solutionOK){
      cout << "Inverse kinematics solution has a NaN value" << endl;
      return false;
    }

    // Store the selection solution


//      double control [6];
//      UR5_controls(control,_result);
//      cout << "Joint values are:" << endl;
//      for (int j = 0; j < 6; j++) {
//          cout << "  theta" << j+1 << "=" << _result[j]
//               << " (" << control[j] << ")" << endl;
//      }
//      return true;

      return false; // TEST TEST TEST
  } else {
    cout << "Inverse kinematics failed" << endl;
    return false;
  }
}

bool IvKinYumi::setParameters(){
  try{

      // Get parameters
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

      it = _parameters.find("Redundant joint" );
      if(it != _parameters.end()){
          _redundantJoint = it->second;
      }else
        return false;

      // Set target position
      _targetTrans.setTranslation(mt::Point3(_eulPos.at(0), _eulPos.at(1), _eulPos.at(2)));
      _targetTrans.setRotation(mt::Rotation(_eulPos.at(5), _eulPos.at(4), _eulPos.at(3)));

      for( int i = 0; i < 3; i++)   _target.at(i) = _targetTrans.getTranslation().at(i);
      for( int i = 3; i < 7; i++)   _target.at(i) = _targetTrans.getRotation().at(i-3);

      // Set redundant joint
      if(_target.size() > 7)    _target.at(7) = _redundantJoint;

    }catch(...){
      return false;
    }
    return true;
}

void IvKinYumi::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
  (void)masterconf;//unused
  (void)maintainSameWrist;//unused

  //loads the target: the tcp transform
  _target.clear();
  for(unsigned i =0; i< target.size(); i++)     _target.push_back(target.at(i));
}
