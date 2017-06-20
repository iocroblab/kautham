
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
#include <kautham/util/libkin/ivkinkukalwr.h>
//#include "YumiKinematics.h"


IvKinKukaLWR::IvKinKukaLWR(Robot* const rob): Kautham::InverseKinematic(rob){

  _target.resize(8);  // This contains the pose and quaternion as a vector
  _target[6] = 1.0; // w component of the quaternion
  _eulPos.resize(6);
  _robConf.setRn(7);

  addParameter("Px", _eulPos.at(0));
  addParameter("Py", _eulPos.at(1));
  addParameter("Pz", _eulPos.at(2));
  addParameter("Rx", _eulPos.at(3));
  addParameter("Ry", _eulPos.at(4));
  addParameter("Rz", _eulPos.at(5));

  addParameter("Redundant joint", _redundantJoint);
}

IvKinKukaLWR::~IvKinKukaLWR(){

}

bool IvKinKukaLWR::solve(){

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


    std::cout << "KUKA LWR IK ----------------------------------------------------------" << std::endl;

    std::cout << "_target (pos / quat): " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << " "
              << _target.at(3) << " "     << _target.at(4) << " " << _target.at(5) << " "
              << _target.at(6) << " "     << _target.at(7) << " " << std::endl;

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
    if (_target.size() > 7)   redundantJoint = _target.at(7);
    std::cout << "Redundat joint = " << redundantJoint << std::endl;


    std::cout << "INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    //  Desired TCP pose
    mt::Transform desired_TCP = _targetTrans;
    //  TEST
    bool test = true;
//    test = false;
    Eigen::VectorXf q_test(7);
    if (test){
        q_test << M_PI/6.0, -M_PI/6.0, M_PI/4.0, -M_PI/5.0, 0.0, M_PI/6.0, -M_PI/3.0;
        std::cout << " q_test: " << q_test.transpose() << std::endl;

        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(q_test(i-1));
        desired_TCP = *_robot->getLink(7)->getTransformation();
    }
//    plot_pose(&desired_TCP_,"desired_TCP");



    // Solve inverse kinematics ---------------------------------------

    //  From:
    //  "Analisis cinemático de robots manipuladores redundantes: aaplicacion a los robots Kuka LWR 4+ y ABB Yumi"
    //    Isiah Zaplana, Josep-Arnau Claret y Luis Basañez
    //  2017

    float px = desired_TCP.getTranslation()[0];
    float py = desired_TCP.getTranslation()[1];
    float pz = desired_TCP.getTranslation()[2];


    //  Solver for position +++++++++

    //   Compute q(3)'s
    float a_3 = ( px*px + py*py + (pz-310.0)*(pz-310.0) - 312100.0 )/312000.0;
    std::vector<float> q3s;
    float tmp = sqrt(1.0 - a_3*a_3);
    q3s.push_back( atan2(+tmp, a_3) );
    q3s.push_back( atan2(-tmp, a_3) );

    //   Compute q(0)'s
    float s2 = sin(redundantJoint);
    std::vector<float> q0s;
    for (unsigned int i=0; i<2; ++i){
        // For each q(3)
        float s3 = sin(q3s[i]);
        float tmp = sqrt( px*px + py*py - 390.0*390.0*s2*s2*s3*s3 );
        float atanxy = atan2(py, px);
        q0s.push_back( atan2(390.0*s2*s3, +tmp) - atanxy );
        q0s.push_back( atan2(390.0*s2*s3, -tmp) - atanxy );
    }

    //   Compute q(1)'s
    float c2 = cos(redundantJoint);
    std::vector<float> q1s;
    for (unsigned int i=0; i<2; ++i){
        // For each q(3)
        float s3 = sin(q3s[i]);
        float c3 = cos(q3s[i]);
        float a_4 = ( 390.0*c3 + 400.0 );
        float b_4 = - 390.0*c2*s3;
        float c_4 = pz - 310.0;
        float tmp = sqrt( a_4*a_4 + b_4*b_4 + c_4*c_4 );
        float atanab = atan2(a_4, b_4);
        q1s.push_back( atan2(c_4, +tmp) - atanab );
        q1s.push_back( atan2(c_4, -tmp) - atanab );
    }

    // Store 'wrist' configurations in a list
    std::vector< std::vector<float> > q0123s;
    for (unsigned int i=0; i<4; ++i){
        std::vector<float> qArm(4);
        qArm[0] = q0s[i];
        for (unsigned int i=0; i<4; ++i){
            qArm[1] = q1s[i];
            qArm[2] = redundantJoint;
            for (unsigned int i=0; i<2; ++i){
                qArm[3] = q3s[i];
                q0123s.push_back(qArm);
            }
        }
    }


    //  Solver for rotation +++++++++
    mt::Rotation R = _targetTrans.getRotation();

    std::vector< std::vector<float> > q456s;
    for (unsigned int i=0; i<q0123s.size(); ++i){
        std::vector<float> qArm(q0123s[i]);
        for (unsigned int j=0; j<4; ++j)    _robot->getLink(j)->setValue(qArm[j]);

        //   Compute wrist rotation matrix
        mt::Rotation R0 = _robot->getLink(0)->getTransformation()->getRotation();
        mt::Rotation R1 = _robot->getLink(1)->getTransformation()->getRotation();
        mt::Rotation R2 = _robot->getLink(2)->getTransformation()->getRotation();
        mt::Rotation R3 = _robot->getLink(3)->getTransformation()->getRotation();

        mt::Rotation R456 = R3.inverse() * R2.inverse() * R1.inverse() * R0.inverse() * R;

//        float m11 = R456.getMatrix()[0][0];
//        float m12 = R456.getMatrix()[0][1];
        float m13 = R456.getMatrix()[0][2];
        float m21 = R456.getMatrix()[1][0];
        float m22 = R456.getMatrix()[1][1];
        float m23 = R456.getMatrix()[1][2];
//        float m31 = R456.getMatrix()[2][0];
//        float m32 = R456.getMatrix()[2][1];
        float m33 = R456.getMatrix()[2][2];

        float tmp = sqrt( m13*m13 + m33*m33 );

        // First set
        float q5_1 = atan2( +tmp, m23 );
        float s5_1 = sin(q5_1);
        float q4_1 = atan2(  m33/s5_1, -m13/s5_1 );
        float q6_1 = atan2( -m22/s5_1,  m21/s5_1 );
        std::vector<float> qWrist_1;
        qWrist_1.push_back(q4_1);
        qWrist_1.push_back(q5_1);
        qWrist_1.push_back(q6_1);
        q456s.push_back(qWrist_1);

        // Second set
        float q5_2 = atan2( -tmp, m23 );
        float s5_2 = sin(q5_2);
        float q4_2 = atan2(  m33/s5_2, -m13/s5_2 );
        float q6_2 = atan2( -m22/s5_2,  m21/s5_2 );
        std::vector<float> qWrist_2;
        qWrist_2.push_back(q4_2);
        qWrist_2.push_back(q5_2);
        qWrist_2.push_back(q6_2);
        q456s.push_back(qWrist_2);
    }


    // Join the position and orientation set of joints
    std::vector< std::vector<float> > solutionSet;
    for (unsigned int i=0; i<q0123s.size(); ++i){
        for (unsigned int j=0; j<q456s.size(); ++j){
            std::vector<float> config(7);
            config[0] = q0123s[i][0];
            config[1] = q0123s[i][1];
            config[2] = q0123s[i][2];
            config[3] = q0123s[i][3];
            config[4] =  q456s[j][0];
            config[5] =  q456s[j][1];
            config[6] =  q456s[j][2];
            solutionSet.push_back(config);
        }
    }





    //  Generate an initial configuration
    Eigen::VectorXf init_q(7);
    init_q << 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0;
    if (test){
        init_q = q_test;
        std::cout << " init_q: " << init_q.transpose() << std::endl;
    }

    Eigen::VectorXf ikSolution(7);
//    YumiKinematics* yumiKinSolver;
//    bool ik_solved = yumiKinSolver->solveIK(mt_to_Eigen_pose(&desired_TCP_ShoulderFrame), init_q, 2, redundantJoint, ikSolution, false);
    bool ik_solved = false;

    // Final TCP pose
    for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(ikSolution(i-1));
    mt::Transform ikResult_TCP = *_robot->getLink(7)->getTransformation();
    plot_pose(&ikResult_TCP,"ikResult_TCP");

    mt::Scalar y, p, r;
    ikResult_TCP.getRotation().getYpr(y, p, r);
    std::cout << "ypr " << y << " " << p << " " << r << std::endl;

//    // TEST
//    Eigen::VectorXf ikTestConfig(ikSolution);
//    std::cout << "ikSolution = " << ikSolution.transpose() << std::endl;
////        Eigen::Matrix4f ikResult_TCP_2(yumiKinSolver->ForwardKinematics(ikTestConfig));
//    std::cout << "yumiKinSolver->ForwardKinematics(ikTestConfig)" << std::endl << yumiKinSolver->ForwardKinematics(ikTestConfig) << std::endl;
//    std::cout << "ikResult_TCP_2" << std::endl << ikResult_TCP_2 << std::endl;
//    // TEST


    // Store the selection solution to kautham
    std::vector<KthReal> qn(7);
    for (unsigned int i = 0; i<7; ++i)  qn.at(i) = ikSolution[i];
    _robConf.setRn(qn);

    if (!ik_solved)     cout << "Inverse kinematics failed" << endl;

    return ik_solved;
}


bool IvKinKukaLWR::setParameters(){
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

void IvKinKukaLWR::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
  (void)masterconf;//unused
  (void)maintainSameWrist;//unused

  //loads the target: the tcp transform
  _target.clear();
  for(unsigned i =0; i< target.size(); i++)     _target.push_back(target.at(i));
}
