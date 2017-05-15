
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

#include <kautham/problem/robot.h>
#include <kautham/util/libkin/ivkinyumi.h>
#include "YumiKinematics.h"


IvKinYumi::IvKinYumi(Robot* const rob): Kautham::InverseKinematic(rob){

  _target.resize(8);  // This contains the pose and quaternion as a vector
  _target[6] = 1.0; // w component of the quaternion
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

  std::cout << "Yumi IK ----------------------------------------------------------" << std::endl;

  std::cout << "_target: " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << " "
                           << _target.at(3) << " " << _target.at(4) << " " << _target.at(5) << " "
                           << _target.at(6) << " " << _target.at(7) << " " << std::endl;

  // Set target pose
  _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1), _target.at(2)));
  _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));

  // Set redundant joint
  double redundantJoint = 0.0;
  if (_target.size() > 7)   redundantJoint = _target.at(7);


  std::cout << "_targetTrans p: " << _targetTrans.getTranslation().at(0) << " "
                                  << _targetTrans.getTranslation().at(1) << " "
                                  << _targetTrans.getTranslation().at(2) << "\n";
  std::cout << "_targetTrans rot:\n";
  std::cout << "  " << _targetTrans.getRotation().getMatrix()[0][0] << " " << _targetTrans.getRotation().getMatrix()[0][1] << " " << _targetTrans.getRotation().getMatrix()[0][2] << std::endl;
  std::cout << "  " << _targetTrans.getRotation().getMatrix()[1][0] << " " << _targetTrans.getRotation().getMatrix()[1][1] << " " << _targetTrans.getRotation().getMatrix()[1][2] << std::endl;
  std::cout << "  " << _targetTrans.getRotation().getMatrix()[2][0] << " " << _targetTrans.getRotation().getMatrix()[2][1] << " " << _targetTrans.getRotation().getMatrix()[2][2] << std::endl;
  std::cout << "redundant_joint: " << redundantJoint << "\n";


  // Desired pose
  //  Convert to Eigen
  Eigen::Matrix4f desiredPoseInRobotRef(Eigen::Matrix4f::Identity());
  //   Position
  for (unsigned int i=0; i<3; ++i)  desiredPoseInRobotRef(i,3) = _targetTrans.getTranslation().at(i);
  //   Orientation
  mt::Matrix3x3 tmp_rot = _targetTrans.getRotation().getMatrix();
  for (unsigned int i=0; i<3; ++i)  for (unsigned int j=0; j<3; ++j)    desiredPoseInRobotRef(i,j) = tmp_rot[i][j];

  std::cout << "yumi desired pose: " << std::endl;
  std::cout << desiredPoseInRobotRef << std::endl;

  // Transform to right arm reference
  Eigen::Matrix4f bodyToRightArmTF(Eigen::Matrix4f::Identity());
  //   Position
  bodyToRightArmTF(0,3) = 0.05355;
  bodyToRightArmTF(1,3) = -0.0725;
  bodyToRightArmTF(2,3) = 0.41492;
  //   Orientation
  tmp_rot = mt::Rotation(-2.3180,-0.5716,-0.9781).getMatrix();
  for (unsigned int i=0; i<3; ++i)  for (unsigned int j=0; j<3; ++j)    bodyToRightArmTF(i,j) = tmp_rot[i][j];

//  std::cout << "Link1:\n pos:" << bodyToRightArmTF(0,3) << " " << bodyToRightArmTF(1,3) << " " << bodyToRightArmTF(2,3) << std::endl;
//  std::cout << " rot: " << std::endl;
//  std::cout << bodyToRightArmTF(0,0) << " " << bodyToRightArmTF(0,1) << " " << bodyToRightArmTF(0,2) << std::endl;
//  std::cout << bodyToRightArmTF(1,0) << " " << bodyToRightArmTF(1,1) << " " << bodyToRightArmTF(1,2) << std::endl;
//  std::cout << bodyToRightArmTF(2,0) << " " << bodyToRightArmTF(2,1) << " " << bodyToRightArmTF(2,2) << std::endl;


  Eigen::Matrix4f desiredPoseInArmRef( bodyToRightArmTF.inverse() * desiredPoseInRobotRef );

//  std::cout << "desiredPoseInArmRef:\n pos:" << desiredPoseInArmRef(0,3) << " " << desiredPoseInArmRef(1,3) << " " << desiredPoseInArmRef(2,3) << std::endl;
//  std::cout << " rot: " << std::endl;
//  std::cout << desiredPoseInArmRef(0,0) << " " << desiredPoseInArmRef(0,1) << " " << desiredPoseInArmRef(0,2) << std::endl;
//  std::cout << desiredPoseInArmRef(1,0) << " " << desiredPoseInArmRef(1,1) << " " << desiredPoseInArmRef(1,2) << std::endl;
//  std::cout << desiredPoseInArmRef(2,0) << " " << desiredPoseInArmRef(2,1) << " " << desiredPoseInArmRef(2,2) << std::endl;


  desiredPoseInArmRef = desiredPoseInRobotRef;  // TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

  // Set position in mm for yumi ik algorithm
  for (unsigned int i=0;i<3;++i)    desiredPoseInArmRef(i,3) *= 1000;

  // Redundant configuration needs to be denormalized TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
  double denormalized_redundantJoint = redundantJoint;  // TEST TEST TEST TEST TEST
//  double denormalized_redundantJoint = XXXX + XXXX * redundantJoint;    // TODO TODO TODO TODO TODO TODO TODO

  // Declare solver
  YumiKinematics* YumiKinSolver;

  // Get all possible 16 analitic ik configurations
  std::vector< std::vector<double> > yumiAnalyticalIkSolutions = YumiKinSolver->AnalyticalIKSolver(desiredPoseInArmRef, denormalized_redundantJoint);

  //std::cout << "Yumi IK solutions: " << yumiAnalyticalIkSolutions.size() << std::endl; // PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT

  // Solve IK
  if (yumiAnalyticalIkSolutions.size() > 0) {

    // Select the configuration with the closest TCP to the desired configuration
    bool non_nan_config = false;    // There is at least one config without a NaN value
    double min_err = 1000000000;    // Super-high value
    Eigen::VectorXf minAnalyticalIkSolution(yumiAnalyticalIkSolutions[0].size());
    for(unsigned int i=0; i<yumiAnalyticalIkSolutions.size(); i++)
    {
        Eigen::VectorXf ikAnalyticalSolution(yumiAnalyticalIkSolutions[i].size());

        for(unsigned int j=0; j<yumiAnalyticalIkSolutions[i].size(); j++)    ikAnalyticalSolution(j) = yumiAnalyticalIkSolutions[i][j];

        std::cout << "yumiAnalyticalIkSolutions[i] = "; // PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
        for(unsigned int j=0; j<ikAnalyticalSolution.size(); j++)   std::cout << ikAnalyticalSolution(j) << " ";
        std::cout << std::endl;

        // Look for NaN values
        bool solutionOK = true;
        for (unsigned int i=0; solutionOK && (i<ikAnalyticalSolution.size()); ++i){
          // f != f will be true if f is NaN or -NaN
          solutionOK = !(ikAnalyticalSolution[i] != ikAnalyticalSolution[i]);
        }

        if (solutionOK){    // If solution has no NaN
            non_nan_config = true;

            // Desired pose
            Eigen::Vector3f pd(desiredPoseInArmRef(0,3), desiredPoseInArmRef(1,3), desiredPoseInArmRef(2,3));
            Eigen::Matrix3f Rd;
            Rd << desiredPoseInArmRef(0,0), desiredPoseInArmRef(0,1), desiredPoseInArmRef(0,2),
                  desiredPoseInArmRef(1,0), desiredPoseInArmRef(1,1), desiredPoseInArmRef(1,2),
                  desiredPoseInArmRef(2,0), desiredPoseInArmRef(2,1), desiredPoseInArmRef(2,2);
            Eigen::Quaternionf uqd(Rd);

            // Pose of the analitical solution
            Eigen::Matrix4f currentPose = YumiKinSolver->ForwardKinematics(ikAnalyticalSolution);

            Eigen::Vector3f pe(currentPose(0,3), currentPose(1,3), currentPose(2,3));
            Eigen::Matrix3f Re;
            Re << currentPose(0,0), currentPose(0,1), currentPose(0,2),
                  currentPose(1,0), currentPose(1,1), currentPose(1,2),
                  currentPose(2,0), currentPose(2,1), currentPose(2,2);
            Eigen::Quaternionf uqe(Re);

            // Error
            //  Position
            Eigen::Vector3f errorP(pd-pe);

            //  Orientation
            Eigen::Vector3f u1(uqd.x(),uqd.y(),uqd.z());
            Eigen::Vector3f u2(uqe.x(),uqe.y(),uqe.z());
            Eigen::Matrix3f m;
            m <<        0, -uqd.z(),  uqd.y(),
                    uqd.z(),        0, -uqd.x(),
                    -uqd.y(),  uqd.x(),        0;
            Eigen::Vector3f errorO(uqe.w()*u1-uqd.w()*u2-m*u2);

            //  Total
            double e = (errorO.squaredNorm()+errorP.squaredNorm())/2.0;

            // Keep minimum error solution
            if (e < min_err){
                min_err = e;
                minAnalyticalIkSolution = ikAnalyticalSolution;
            }
        } // if (solutionOK)
    }

    // If all analytic configurations have NaN values there is no solution for the IK problem
    if (!non_nan_config){
      std::cout << "Inverse kinematics solution has a NaN value" << std::endl;
      return false;
    }

    // The configuration to feed the numerical solver is ok - PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
    std::cout << "Yumi IK analytical solution: ";
    for (unsigned int i=0; i<minAnalyticalIkSolution.size(); ++i)  std::cout << minAnalyticalIkSolution[i] << " ";
    std::cout << std::endl;

    // Solve the ik using a numerical approach starting with the minimum error configuration
    Eigen::VectorXf ikSolution(YumiKinSolver->NumericalIKSolver(desiredPoseInArmRef, minAnalyticalIkSolution, 0.001, 10000));

    // The final configuration - PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
    std::cout << "Yumi IK analytical solution: ";
    for (unsigned int i=0; i<ikSolution.size(); ++i)  std::cout << ikSolution[i] << " ";
    std::cout << std::endl;

    // Do the forward kinematics for verification - PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
    Eigen::Matrix4f verif_pose = YumiKinSolver->ForwardKinematics(ikSolution);
    for (unsigned int i=0; i<3; ++i)    verif_pose(i,3) /= 1000.0;
    std::cout << "verif_pose:\n pos:" << verif_pose(0,3) << " " << verif_pose(1,3) << " " << verif_pose(2,3) << std::endl;
    std::cout << " rot: " << std::endl;
    std::cout << verif_pose(0,0) << " " << verif_pose(0,1) << " " << verif_pose(0,2) << std::endl;
    std::cout << verif_pose(1,0) << " " << verif_pose(1,1) << " " << verif_pose(1,2) << std::endl;
    std::cout << verif_pose(2,0) << " " << verif_pose(2,1) << " " << verif_pose(2,2) << std::endl;


    // Store the selection solution
    std::vector<KthReal> tmp(7);
    for (unsigned int i = 0; i<7; ++i)  tmp.at(i) = ikSolution[i];
    _robConf.setRn(tmp);

    return true;
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

      std::cout << "--> _target: ";
      for (unsigned int i=0; i<_target.size(); ++i) std::cout << _target.at(i) << " ";
      std::cout << std::endl;

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
