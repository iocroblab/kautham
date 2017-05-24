
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
  tmp_rot = mt::Rotation(-2.3180,-0.5682,-0.9781).getMatrix();
  for (unsigned int i=0; i<3; ++i)  for (unsigned int j=0; j<3; ++j)    bodyToRightArmTF(i,j) = tmp_rot[i][j];

  // Transform to left arm reference
  Eigen::Matrix4f bodyToLeftArmTF(Eigen::Matrix4f::Identity());
  //   Position
  bodyToLeftArmTF(0,3) = 0.05355;
  bodyToLeftArmTF(1,3) = 0.0725;
  bodyToLeftArmTF(2,3) = 0.41492;
  //   Orientation
  tmp_rot = mt::Rotation(2.3180,-0.5716,-0.9781).getMatrix();
  for (unsigned int i=0; i<3; ++i)  for (unsigned int j=0; j<3; ++j)    bodyToLeftArmTF(i,j) = tmp_rot[i][j];

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

  // Redundant configuration needs to be denormalized
  double denormalized_redundantJoint = redundantJoint;  // TEST TEST TEST TEST TEST
//  double denormalized_redundantJoint = -2.94088 + redundantJoint * 5.88176;    // TODO TODO TODO TODO TODO TODO TODO

  // Declare solver
  YumiKinematics* YumiKinSolver;

  // Get all possible 16 analitic ik configurations
  std::vector< std::vector<double> > yumiAnalyticalIkSolutions = YumiKinSolver->AnalyticalIKSolver(desiredPoseInArmRef, denormalized_redundantJoint);

  //std::cout << "Yumi IK solutions: " << yumiAnalyticalIkSolutions.size() << std::endl; // PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT


  // DK testing
  Eigen::VectorXf dkConfig(7);
  Eigen::Matrix4f dkPose;

//  // Ik yumi
//  dkConfig << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << M_PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, M_PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, 0.0, M_PI/2.0, 0.0, 0.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, 0.0, 0.0, M_PI/2.0, 0.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, 0.0, 0.0, 0.0, M_PI/2.0, 0.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, 0.0, 0.0, 0.0, 0.0, M_PI/2.0, 0.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;

//  dkConfig << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI/2.0;
//  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
//  std::cout << ">> q = " << dkConfig.transpose() << std::endl << dkPose << std::endl;


  // Kautham yumi
  mt::Transform* dkYumiTCPPose, dkYumiRightShoulderPose;
  dkYumiRightShoulderPose.setRotation(mt::Rotation(-2.3180,-0.5682,-0.9781));
  dkYumiRightShoulderPose.setTranslation(mt::Point3(0.05355, -0.0725, 0.41492));
  mt::Transform dkYumiPose;

  Eigen::VectorXf dkYumiConfig(8);

//  dkYumiConfig << 0.0, 0.0, 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, 0.0;
//  std::cout << "n links / joints: " << _robot->getNumLinks() << " / " << _robot->getNumJoints() << std::endl;
//  for (unsigned int i=1; i<8; ++i)  _robot->getLink(i)->setValue(dkYumiConfig(i));
//  for (unsigned int i=1; i<8; ++i)  _robot->getLink(i)->getValue();
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  dkYumiTCPPose = _robot->getLink(7)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiTCPPose);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }


//  // dk per link
//  mt::Transform* dkYumiPose0;
//  dkYumiPose0 = _robot->getLink(0)->getTransformation();
//  dkYumiPose = *dkYumiPose0;
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose0:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose1;
//  dkYumiPose1 = _robot->getLink(1)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose1);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose1:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose2;
//  dkYumiPose2 = _robot->getLink(2)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose2);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose2:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose3;
//  dkYumiPose3 = _robot->getLink(3)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose3);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose3:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose4;
//  dkYumiPose4 = _robot->getLink(4)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose4);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose4:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose5;
//  dkYumiPose5 = _robot->getLink(5)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose5);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose5:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose6;
//  dkYumiPose6 = _robot->getLink(6)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose6);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose6:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose7;
//  dkYumiPose7 = _robot->getLink(7)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose7);
//  std::cout << "> q = " << dkYumiConfig.transpose() << std::endl;
//  std::cout << "dkYumiPose7:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }


  std::cout << "------------------------------------------------------------------" << std::endl;

  dkConfig << M_PI/3.0, -M_PI/6.0, 0.0, -M_PI/4.0, 0.0, M_PI/6.0, 0.0;
  std::cout << ">> q = " << dkConfig.transpose() << std::endl;
  for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));

//  mt::Transform* dkYumiPose0;
//  dkYumiPose0 = _robot->getLink(0)->getTransformation();
//  dkYumiPose = *dkYumiPose0;
//  std::cout << "dkYumiPose0:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose1;
//  dkYumiPose1 = _robot->getLink(1)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose1);
//  std::cout << "dkYumiPose1:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose2;
//  dkYumiPose2 = _robot->getLink(2)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose2);
//  std::cout << "dkYumiPose2:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose3;
//  dkYumiPose3 = _robot->getLink(3)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose3);
//  std::cout << "dkYumiPose3:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose4;
//  dkYumiPose4 = _robot->getLink(4)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose4);
//  std::cout << "dkYumiPose4:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose5;
//  dkYumiPose5 = _robot->getLink(5)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose5);
//  std::cout << "dkYumiPose5:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }

//  mt::Transform* dkYumiPose6;
//  dkYumiPose6 = _robot->getLink(6)->getTransformation();
//  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPose6);
//  std::cout << "dkYumiPose6:" << std::endl;
//  for (unsigned int i=0; i<3; ++i){
//    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
//    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
//  }


  // Right Arm
  mt::Transform* dkYumiPoseTCP;
  dkYumiPoseTCP = _robot->getLink(7)->getTransformation();
  dkYumiPose = dkYumiRightShoulderPose.inverse() * (*dkYumiPoseTCP);
  std::cout << "dkYumiPoseTCP:" << std::endl;
  for (unsigned int i=0; i<3; ++i){
    for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
    std::cout << dkYumiPose.getTranslation()[i] << std::endl;
  }
  dkConfig(3) = dkConfig(3) + M_PI/2.0;
  dkPose = YumiKinSolver->ForwardKinematics(dkConfig);
  std::cout << "dkPose:" << std::endl << dkPose << std::endl;


    {
        // Direct Kinematics --------------------------------------------------------
        std::cout << "DIRECT KINEMATICS" << std::endl;

        Eigen::VectorXf dkConfig(7);
//        dkConfig << M_PI/6.0, -M_PI/6.0, 0.0, -M_PI/5.0, 0.0, M_PI/6.0, 0.0;
//        dkConfig << 0.0, 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, 0.0;
        dkConfig << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        std::cout << ">> q = " << dkConfig.transpose() << std::endl;
        for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));
        Eigen::VectorXf dkYumiConfig;


        // Right Arm
        mt::Transform* dkYumiRightShoulderPose = new mt::Transform;
        dkYumiRightShoulderPose->setRotation(mt::Rotation(-2.3180,-0.5682,-0.9781));
        dkYumiRightShoulderPose->setTranslation(mt::Point3(0.05355, -0.0725, 0.41492));

        mt::Transform* dkYumiRightTCPPose = new mt::Transform;
        dkYumiRightTCPPose = _robot->getLink(7)->getTransformation();
        dkYumiPose = dkYumiRightShoulderPose->inverse() * (*dkYumiRightTCPPose);
        std::cout << "Kautham RIGHT dkYumiPoseTCP:" << std::endl;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
            std::cout << dkYumiPose.getTranslation()[i] << std::endl;
        }
        std::cout << "det = " << std::log(dkYumiPose.getRotation().getMatrix().determinant()) << std::endl;
        dkYumiConfig = dkConfig;
        dkYumiConfig(3) = dkYumiConfig(3) + M_PI/2.0;
        dkPose = YumiKinSolver->ForwardKinematics(dkYumiConfig);
        std::cout << "IK dkPose:" << std::endl << dkPose << std::endl;


        // Left Arm
        mt::Transform* dkYumiLeftShoulderPose = new mt::Transform;
        dkYumiLeftShoulderPose->setRotation(mt::Rotation(2.3180,-0.5716,0.9781));
        dkYumiLeftShoulderPose->setTranslation(mt::Point3(0.05355, 0.0725, 0.41492));

        mt::Transform* dkYumiLeftTCPPose = new mt::Transform;
        dkYumiLeftTCPPose = _robot->getLink(7)->getTransformation();
        dkYumiPose = dkYumiLeftShoulderPose->inverse() * (*dkYumiLeftTCPPose);
        std::cout << "Kautham LEFT dkYumiPoseTCP:" << std::endl;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
            std::cout << dkYumiPose.getTranslation()[i] << std::endl;
        }
        std::cout << "det = " << std::log(dkYumiPose.getRotation().getMatrix().determinant()) << std::endl;
        dkYumiConfig = dkConfig;
        dkYumiConfig(3) = dkYumiConfig(3) + M_PI/2.0;
        dkPose = YumiKinSolver->ForwardKinematics(dkYumiConfig);
        std::cout << "IK dkPose:" << std::endl << dkPose << std::endl;


        // Inverse Kinematics --------------------------------------------------------
        std::cout << "INVERSE KINEMATICS" << std::endl;

        dkConfig << M_PI/6.0, -M_PI/6.0, 0.0, -M_PI/5.0, 0.0, M_PI/6.0, 0.0;
        // Compute test pose
        for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));
        mt::Transform* ikTestPose = new mt::Transform;
        ikTestPose = _robot->getLink(7)->getTransformation();
        std::cout << "IK Test Pose:" << std::endl;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << ikTestPose->getRotation().getMatrix()[i][j] << "  ";
            std::cout << ikTestPose->getTranslation()[i] << std::endl;
        }

        // To right arm reference
        mt::Transform* ikRightTestPose = new mt::Transform;
        *ikRightTestPose = dkYumiRightShoulderPose->inverse() * (*dkYumiRightTCPPose);


        // Translate to eigen
        Eigen::Matrix4f eigIkTestPose(Eigen::Matrix4f::Identity());
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    eigIkTestPose(i,j) = ikRightTestPose->getRotation().getMatrix()[i][j];
            eigIkTestPose(i,3) =  ikRightTestPose->getTranslation()[i];
        }

        Eigen::VectorXf init_q(7);
        init_q << 0.3, 0.5, 0.0, 1.1, 0.0, -0.1, 0.0;
        init_q = dkConfig;
        init_q(3) = dkConfig(3) - M_PI/2.0;
        Eigen::VectorXf test_ikSolution(YumiKinSolver->NumericalIKSolver(eigIkTestPose, init_q, 0.01, 10000));

        Eigen::VectorXf ikKauthamSolution(test_ikSolution);
        ikKauthamSolution(3) = test_ikSolution(3) - M_PI/2.0;
        for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(ikKauthamSolution(i-1));
        mt::Transform* ikPose = new mt::Transform;
        _robot->getLink(7)->getTransformation();
        std::cout << "IK Pose:" << std::endl;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << ikPose->getRotation().getMatrix()[i][j] << "  ";
            std::cout << ikPose->getTranslation()[i] << std::endl;
        }


//        // To left arm reference
//        mt::Transform* ikLeftTestPose = new mt::Transform;
//        ikLeftTestPose = dkYumiLeftShoulderPose->inverse() * (*dkYumiLeftTCPPose);

    }




  return false;


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

//        // Saturate joint values
//        // This is real Yumi convention
//        if      (ikAnalyticalSolution(0) < -2.94088)    ikAnalyticalSolution(0) = -2.94088;
//        else if (ikAnalyticalSolution(0) >  2.94088)    ikAnalyticalSolution(0) =  2.94088;

//        if      (ikAnalyticalSolution(1) < -2.50455)    ikAnalyticalSolution(1) = -2.50455;
//        else if (ikAnalyticalSolution(1) > 0.759218)    ikAnalyticalSolution(1) = 0.759218;

//        if      (ikAnalyticalSolution(2) < -2.94088)    ikAnalyticalSolution(2) = -2.94088;
//        else if (ikAnalyticalSolution(2) >  2.94088)    ikAnalyticalSolution(2) =  2.94088;

//        if      (ikAnalyticalSolution(3) < -2.15548)    ikAnalyticalSolution(3) = -2.15548;
//        else if (ikAnalyticalSolution(3) >  1.39626)    ikAnalyticalSolution(3) =  1.39626;

//        if      (ikAnalyticalSolution(4) < -5.06145)    ikAnalyticalSolution(4) = -5.06145;
//        else if (ikAnalyticalSolution(4) >  5.06145)    ikAnalyticalSolution(4) =  5.06145;

//        if      (ikAnalyticalSolution(5) < -1.53589)    ikAnalyticalSolution(5) = -1.53589;
//        else if (ikAnalyticalSolution(5) >  2.40855)    ikAnalyticalSolution(5) =  2.40855;

//        if      (ikAnalyticalSolution(6) <  -3.9968)    ikAnalyticalSolution(6) =  -3.9968;
//        else if (ikAnalyticalSolution(6) >   3.9968)    ikAnalyticalSolution(6) =   3.9968;

//        std::cout << "yumiAnalyticalIkSolutions[i] = "; // PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
//        for(unsigned int j=0; j<ikAnalyticalSolution.size(); j++)   std::cout << ikAnalyticalSolution(j) << " ";
//        std::cout << std::endl;

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

    // TEST POSE
    Eigen::VectorXf test_q(Eigen::VectorXf::Zero(7));
    test_q << 2.1, 0.3, -1.5, 0.0, -1.1, 1.9, 3.0;
    Eigen::Matrix4f test_pose = YumiKinSolver->ForwardKinematics(test_q);
    test_q << 0.3, 0.5, 0.0, 1.1, 0.0, -0.1, 0.0;
    Eigen::VectorXf test_ikSolution(YumiKinSolver->NumericalIKSolver(test_pose, test_q, 0.01, 10000));
    return false;


    // Set angles between -pi and pi
    for (unsigned int i=0; i<minAnalyticalIkSolution.size(); ++i){
        while (minAnalyticalIkSolution(i) < -PI)     minAnalyticalIkSolution(i) += 2.0*PI;
        while (minAnalyticalIkSolution(i) >  PI)     minAnalyticalIkSolution(i) -= 2.0*PI;
    }

    // Saturate joint values
    // This is real Yumi convention
    if      (minAnalyticalIkSolution(0) < -2.94088)    minAnalyticalIkSolution(0) = -2.94088;
    else if (minAnalyticalIkSolution(0) >  2.94088)    minAnalyticalIkSolution(0) =  2.94088;

    if      (minAnalyticalIkSolution(1) < -2.50455)    minAnalyticalIkSolution(1) = -2.50455;
    else if (minAnalyticalIkSolution(1) > 0.759218)    minAnalyticalIkSolution(1) = 0.759218;

    if      (minAnalyticalIkSolution(2) < -2.94088)    minAnalyticalIkSolution(2) = -2.94088;
    else if (minAnalyticalIkSolution(2) >  2.94088)    minAnalyticalIkSolution(2) =  2.94088;

    if      (minAnalyticalIkSolution(3) < -2.15548)    minAnalyticalIkSolution(3) = -2.15548;
    else if (minAnalyticalIkSolution(3) >  1.39626)    minAnalyticalIkSolution(3) =  1.39626;

    if      (minAnalyticalIkSolution(4) < -5.06145)    minAnalyticalIkSolution(4) = -5.06145;
    else if (minAnalyticalIkSolution(4) >  5.06145)    minAnalyticalIkSolution(4) =  5.06145;

    if      (minAnalyticalIkSolution(5) < -1.53589)    minAnalyticalIkSolution(5) = -1.53589;
    else if (minAnalyticalIkSolution(5) >  2.40855)    minAnalyticalIkSolution(5) =  2.40855;

    if      (minAnalyticalIkSolution(6) <  -3.9968)    minAnalyticalIkSolution(6) =  -3.9968;
    else if (minAnalyticalIkSolution(6) >   3.9968)    minAnalyticalIkSolution(6) =   3.9968;


    // Solve the ik using a numerical approach starting with the minimum error configuration
    Eigen::VectorXf ikSolution(YumiKinSolver->NumericalIKSolver(desiredPoseInArmRef, minAnalyticalIkSolution, 0.001, 10000));

    // The final configuration - PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT
    std::cout << "Yumi IK analytical solution: ";
    for (unsigned int i=0; i<ikSolution.size(); ++i)  std::cout << ikSolution[i] << " ";
    std::cout << std::endl;

    // Normalize configuration
    ikSolution(0) = ( ikSolution(0) - (-2.94088) )/(  2.94088 - (-2.94088) );
    ikSolution(1) = ( ikSolution(0) - (-2.50455) )/( 0.759218 - (-2.50455) );
    ikSolution(2) = ( ikSolution(0) - (-2.94088) )/(  2.94088 - (-2.94088) );
    ikSolution(3) = ( ikSolution(0) - (-2.15548) )/(  1.39626 - (-2.15548) );
    ikSolution(4) = ( ikSolution(0) - (-5.06145) )/(  5.06145 - (-5.06145) );
    ikSolution(5) = ( ikSolution(0) - (-1.53589) )/(  2.40855 - (-1.53589) );
    ikSolution(6) = ( ikSolution(0) -  (-3.9968) )/(   3.9968 -  (-3.9968) );

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
