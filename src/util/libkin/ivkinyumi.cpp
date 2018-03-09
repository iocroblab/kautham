
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
#include <iomanip>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <kautham/problem/robot.h>
#include <kautham/util/libkin/ivkinyumi.h>
#include "YumiKinematics.h"


IvKinYumi::IvKinYumi(Robot* const rob, const bool use_left_arm): Kautham::InverseKinematic(rob){

  _target.resize(8);  // This contains the pose and quaternion as a vector
  _target[6] = 1.0; // w component of the quaternion
  _eulPos.resize(6);
  _robConf.setRn(7);

  _redundantJoint = 0.0;

  _use_left_arm = use_left_arm;

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

    // AUXILIARY FUNCTIONS

    //  Auxiliar lambda function for pose ploting
    auto plot_pose = [](const mt::Transform* pose, const char* pose_name) {
        std::cout << "-- " << pose_name << " ------------------------" << std::endl;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << std::setw(12) << pose->getRotation().getMatrix()[i][j] << "  ";
            std::cout << "     " << std::setw(12) << pose->getTranslation()[i] << std::endl;
        }
    };

    //  Auxiliar lambda function to convert pose from mt to eigen
    auto mt_to_Eigen_pose = [](const mt::Transform* pose) {
        Eigen::Matrix4f eigenPose(Eigen::Matrix4f::Identity());
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    eigenPose(i,j) = pose->getRotation().getMatrix()[i][j];
            eigenPose(i,3) =  pose->getTranslation()[i];
        }
        return eigenPose;
    };


    std::cout << "Yumi IK ----------------------------------------------------------" << std::endl;

    std::cout << "Yumi arm is " << (this->arm_is_left() ? "left" : "right") << std::endl;

    std::cout << "_target (pos / quat): " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << " "
              << _target.at(3) << " " << _target.at(4) << " " << _target.at(5) << " "
              << _target.at(6) << " " << _target.at(7) << " " << std::endl;

    // Set target pose
    _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1), _target.at(2)));
    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));
    plot_pose(&_targetTrans,"_target TF");

//    // Set target pose      // TEST
//    _targetTrans.setTranslation(mt::Point3(0.5, -0.2, 0.4));    // TEST
//    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));
//    plot_pose(&_targetTrans,"_target TF");

    // Set redundant joint
    double redundantJoint = 0.0;
    if (_target.size() > 7)   redundantJoint = -2.94088 + 5.88176 * _target.at(7);    // Denormalize
    std::cout << "Redundat joint = " << redundantJoint << std::endl;


    // Shoulder pose in Yumi Frame
    mt::Transform* shoulder_YumiFrame = new mt::Transform;
    if (arm_is_left()){
        shoulder_YumiFrame->setRotation(mt::Rotation(2.3180,-0.5716,0.9781));
        shoulder_YumiFrame->setTranslation(mt::Point3(0.05355, 0.0725, 0.41492));
    }
    else{
        shoulder_YumiFrame->setRotation(mt::Rotation(-2.3180,-0.5682,-0.9781));
        shoulder_YumiFrame->setTranslation(mt::Point3(0.05355, -0.0725, 0.41492));
    }
//    plot_pose(shoulder_YumiFrame,"yumiShoulderPose_YumiFrame");


    std::cout << "INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    //  Desired TCP pose in Yumi frame
    mt::Transform desired_TCP_YumiFrame = _targetTrans;
    //  TEST
    bool test = true;
    test = false;
    Eigen::VectorXf q_test(7);
    if (test){
        q_test << M_PI/6.0, -M_PI/6.0, M_PI/4.0, -M_PI/5.0, 0.0, M_PI/6.0, -M_PI/3.0;
        std::cout << " q_test: " << q_test.transpose() << std::endl;

        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(q_test(i-1));
        desired_TCP_YumiFrame = *_robot->getLink(7)->getTransformation();

        // Test DK
        YumiKinematics* yumiKinSolver = new YumiKinematics();
        Eigen::VectorXf q_dkTest(q_test);
        Eigen::Matrix4f dk_test_pose(mt_to_Eigen_pose(shoulder_YumiFrame) * yumiKinSolver->ForwardKinematics(q_dkTest));
        std::cout << "dk_test_pose desired_TCP_YumiFrame" << std::endl << dk_test_pose << std::endl;
        // Test DK
    }
//    plot_pose(&desired_TCP_YumiFrame,"desired_TCP_YumiFrame");

    // Desired TCP pose in shoulder frame
    mt::Transform desired_TCP_ShoulderFrame = shoulder_YumiFrame->inverse() * desired_TCP_YumiFrame;
//    plot_pose(&desired_TCP_ShoulderFrame,"desired_TCP_ShoulderFrame");


    // Solve inverse kinematics -------------------------------

    //  Generate an initial configuration
    Eigen::VectorXf init_q(7);
    init_q << 0.0, 0.0, 0.0, M_PI/2, 0.0, -0.0, 0.0;
    if (test){
        init_q = q_test;
        init_q(2) = init_q(2) + M_PI/8;
        std::cout << " init_q: " << init_q.transpose() << std::endl;
    }

    Eigen::VectorXf ikSolution(7);
    YumiKinematics* yumiKinSolver = new YumiKinematics();
    bool ik_solved = yumiKinSolver->solveIK(mt_to_Eigen_pose(&desired_TCP_ShoulderFrame), init_q, 2, redundantJoint, ikSolution, false);

    // Final TCP pose
    for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(ikSolution(i-1));
    mt::Transform ikResult_TCP_YumiFrame = *_robot->getLink(7)->getTransformation();
    plot_pose(&ikResult_TCP_YumiFrame,"ikResult_TCP_YumiFrame");

    mt::Scalar y, p, r;
    ikResult_TCP_YumiFrame.getRotation().getYpr(y, p, r);
    std::cout << "ypr " << y << " " << p << " " << r << std::endl;

//    // TEST
//    // Shoulder pose in Yumi Frame
//    Eigen::VectorXf ikTestConfig(ikSolution);
//    std::cout << "ikSolution = " << ikSolution.transpose() << std::endl;
////        Eigen::Matrix4f ikResult_TCP_YumiFrame_2(mt_to_Eigen_pose(shoulder_YumiFrame) * yumiKinSolver->ForwardKinematics(ikTestConfig));
//    std::cout << "yumiKinSolver->ForwardKinematics(ikTestConfig)" << std::endl << yumiKinSolver->ForwardKinematics(ikTestConfig) << std::endl;
//    std::cout << "ikResult_TCP_YumiFrame_2" << std::endl << ikResult_TCP_YumiFrame_2 << std::endl;
//    // TEST


    // Store the selection solution to kautham
    std::vector<KthReal> qn(7);
//    qn.at(0) = ( ikSolution(0) - (-2.94088) )/(  2.94088 - (-2.94088) );
//    qn.at(1) = ( ikSolution(1) - (-2.50455) )/( 0.759218 - (-2.50455) );
//    qn.at(2) = ( ikSolution(2) - (-2.94088) )/(  2.94088 - (-2.94088) );
//    qn.at(3) = ( ikSolution(3) - (-2.15548) )/(  1.39626 - (-2.15548) );
//    qn.at(4) = ( ikSolution(4) - (-5.06145) )/(  5.06145 - (-5.06145) );
//    qn.at(5) = ( ikSolution(5) - (-1.53589) )/(  2.40855 - (-1.53589) );
//    qn.at(6) = ( ikSolution(6) -  (-3.9968) )/(   3.9968 -  (-3.9968) );
    for (unsigned int i = 0; i<7; ++i)  qn.at(i) = ikSolution[i];
    _robConf.setRn(qn);

    if (!ik_solved)     cout << "Inverse kinematics failed" << endl;

    return ik_solved;
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
