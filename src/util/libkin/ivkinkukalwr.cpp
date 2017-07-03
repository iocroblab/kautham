
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

    //  Auxiliar lambda function for pose ploting
    auto plot_rot = [](const mt::Rotation* rot, const char* rot_name) {
        std::cout << "-- " << rot_name << " ------------------------" << std::endl;
        mt::Matrix3x3 rot_mat = rot->getMatrix();
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    std::cout << std::setw(12) << rot_mat[i][j] << "  ";
            std::cout << std::endl;
        }
    };


    std::cout << "KUKA LWR IK ----------------------------------------------------------" << std::endl;

    std::cout << "_target (pos / quat): " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << " "
              << _target.at(3) << " "     << _target.at(4) << " " << _target.at(5) << " "
              << _target.at(6) << " "     << _target.at(7) << " " << std::endl;

    // Set target pose
    _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1), _target.at(2)));
    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));
//    plot_pose(&_targetTrans,"_target TF");

    // Set redundant joint
    double redundantJoint = 0.0;
    if (_target.size() > 7)   redundantJoint = -2.96705972839 + _target.at(7)*(2.96705972839 + 2.96705972839);
//    std::cout << "Redundat joint = " << redundantJoint << std::endl;


//    // Set target pose    // TEST TEST TEST TEST
//    _targetTrans.setTranslation(mt::Point3(0.0, 0.0, 0.878));
//    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));
//    plot_pose(&_targetTrans,"_target TF");
//    redundantJoint = 0.0;
//    std::cout << "Redundat joint = " << redundantJoint << std::endl;





    std::cout << "INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    //  From:
    //  "Analisis cinemático de robots manipuladores redundantes: aaplicacion a los robots Kuka LWR 4+ y ABB Yumi"
    //    Isiah Zaplana, Josep-Arnau Claret y Luis Basañez
    //    Revista Iberoamericana de Automática e Informática Industrial RIAI
    //  2017

    //  Desired TCP pose
    mt::Transform desired_TCP = _targetTrans;
    plot_pose(&desired_TCP,"desired_TCP");
    std::cout << "Redundat joint = " << redundantJoint << std::endl;

    mt::Transform last_link;
    last_link.setTranslation(mt::Point3(0.0, 0.0, 0.078));


    // Solve inverse kinematics ---------------------------------------

    mt::Transform desired_Wrist_TF = desired_TCP * last_link.inverse();
//    plot_pose(&desired_Wrist_TF,"Wrist_TF");

    float px = desired_Wrist_TF.getTranslation()[0];
    float py = desired_Wrist_TF.getTranslation()[1];
    float pz = desired_Wrist_TF.getTranslation()[2];
//    std::cout << "px py pz =  " << px << " " << py << " " << pz << std::endl;


    //  Solver for position ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    //   q(2) is the redundant joint - q3 in the paper notation
    float s3 = sin(redundantJoint);
    float c3 = cos(redundantJoint);
//    std::cout << "q3s = " << redundantJoint << std::endl;


    //   Compute q(3)'s - q4 in the paper notation
    float a_4 = ( px*px + py*py + (pz-0.310)*(pz-0.310) - 0.312100 )/0.312000;
    std::vector<float> q4s;
    float tmp = sqrt(1.0 - a_4*a_4);
    q4s.push_back( atan2(+tmp, a_4) );
    q4s.push_back( atan2(-tmp, a_4) );
//    std::cout << "q4s = ";    for (unsigned int i=0; i<q4s.size(); ++i)   std::cout << q4s[i] << " "; std::cout << std::endl;


    //   Compute q(0)'s - q1 in the paper notation
    std::vector<float> q1s;
    for (unsigned int i=0; i<2; ++i){

        // For each q(3)
        float s4;
        if (i < 2)      s4 = sin(q4s[0]);
        else            s4 = sin(q4s[1]);

        // Compute q1
        float tmp = 0.0;
        tmp = sqrt( px*px + py*py - 0.390*0.390*s3*s3*s4*s4 );
        // Adding 0.0 to turn signed -0.0 to 0.0
        float atan_yx = atan2(py + 0.0, -px + 0.0);
        q1s.push_back( atan2(0.390*s3*s4 + 0.0, +tmp + 0.0) - atan_yx );
        q1s.push_back( atan2(0.390*s3*s4 + 0.0, -tmp + 0.0) - atan_yx );

    }
//    std::cout << "q1s = ";    for (unsigned int i=0; i<q1s.size(); ++i)   std::cout << q1s[i] << " "; std::cout << std::endl;


    //   Compute q(1)'s - q2 in the paper notation
    std::vector<float> q2s;
    for (unsigned int i=0; i<4; ++i){

        // For each q(3)
        float s1 = sin(q1s[i]);
        float c1 = cos(q1s[i]);
        float s4, c4;
        if (i < 2){
            s4 = sin(q4s[0]);
            c4 = cos(q4s[0]);
        }
        else{
            s4 = sin(q4s[1]);
            c4 = cos(q4s[1]);
        }

        // Compute q2
        float a_2 = 0.390*c4 + 0.400;
        float b_2 = 0.390*c3*s4;
        float c_2 = px*c1 + py*s1;
        float tmp = sqrt( a_2*a_2 + b_2*b_2 - c_2*c_2 );
        // Adding 0.0 to turn signed -0.0 to 0.0
        float atan_ab = atan2(a_2 + 0.0, b_2 + 0.0);
        q2s.push_back( atan2(c_2 + 0.0, -tmp + 0.0) - atan_ab );
    }
//    std::cout << "q2s = ";    for (unsigned int i=0; i<q2s.size(); ++i)   std::cout << q2s[i] << " "; std::cout << std::endl;


    // Store 'wrist' configurations in a list
    std::vector< std::vector<float> > q1234s;
    for (unsigned int i=0; i<2; ++i){
        std::vector<float> qArm(4);

        qArm[2] = redundantJoint;
        qArm[3] = q4s[i];

        // For each q4
        qArm[0] = q1s[2*i];
        qArm[1] = q2s[2*i];
        q1234s.push_back(qArm);

        qArm[0] = q1s[2*i+1];
        qArm[1] = q2s[2*i+1];
        q1234s.push_back(qArm);
    }

//    // Test position results
//    std::cout << "Testing IK positions : " << q1234s.size() << std::endl;
//    for (unsigned int i=0; i<q1234s.size(); ++i){
//        Eigen::VectorXf tmp_cfg(7);
//        for (unsigned int j=0; j<4; ++j)    tmp_cfg(j) = q1234s[i][j];
//        for (unsigned int j=4; j<7; ++j)    tmp_cfg(j) = 0.0;

////        // Ploting each position configuration
////        std::cout << "pos cfg " << i << ":  ";
////        for (unsigned int j=0; j<7; ++j)    std::cout << tmp_cfg(j) << " ";
////        std::cout << std::endl;

//        // Setting positions to Kautham convention:
//        //  q_KAUTHAM(1) = q_ISIAH(1) - PI/2
//        //  q_KAUTHAM(i) = q_ISIAH(i)         i \in {0, 2, 3, 4, 5, 6}
//        tmp_cfg(1) = tmp_cfg(1) - M_PI/2.0;

//        // Ploting each position configuration
//        std::cout << "pos cfg " << i << ":  ";
//        for (unsigned int j=0; j<7; ++j)    std::cout << tmp_cfg(j) << " ";
//        std::cout << std::endl;

////        // DK
////        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(tmp_cfg(i-1));
////        mt::Transform Wrist_TF = *_robot->getLink(7)->getTransformation();
////        plot_pose(&Wrist_TF,"Wrist_TF");
//    }


    //  Solver for rotation +++++++++
    mt::Rotation R = desired_Wrist_TF.getRotation();

    std::vector< std::vector<float> > q567s;
    for (unsigned int i=0; i<4; ++i){

        //   Compute wrist rotation matrix
        std::vector<float> qArm(7);
        qArm[0] = q1234s[i][0];
        qArm[1] = q1234s[i][1] - M_PI/2.0;   // To Kautham convention
        qArm[2] = q1234s[i][2];
        qArm[3] = q1234s[i][3];
        qArm[4] = 0.0;
        qArm[5] = 0.0;
        qArm[6] = 0.0;

        for (unsigned int j=1; j<5; ++j)    _robot->getLink(j)->setValue(qArm[j-1]);
        mt::Rotation R4 = _robot->getLink(4)->getTransformation()->getRotation();

        mt::Rotation zRot;
        zRot.setYpr(M_PI, 0.0, 0.0);

        mt::Rotation R4b = R4 * zRot;
        mt::Rotation R567b = R4b.inverse() * R;

        mt::Matrix3x3 R567_rotmat = R567b.getMatrix();
//        float m11 = R567_rotmat[0][0];
//        float m12 = R567_rotmat[0][1];
        float m13 = R567_rotmat[0][2];
        float m21 = R567_rotmat[1][0];
        float m22 = R567_rotmat[1][1];
        float m23 = R567_rotmat[1][2];
//        float m31 = R567_rotmat[2][0];
//        float m32 = R567_rotmat[2][1];
        float m33 = R567_rotmat[2][2];

        float tmp = sqrt( m13*m13 + m33*m33 );

        // First set
        float q6_1 = atan2( +tmp + 0.0, m23 + 0.0 );
        float s6_1 = sin(q6_1);
        float q5_1 = atan2( -m33/s6_1 + 0.0, m13/s6_1 + 0.0 );
        float q7_1 = atan2( -m22/s6_1 + 0.0, m21/s6_1 + 0.0 );
        std::vector<float> qWrist_1;
        qWrist_1.push_back(q5_1);
        qWrist_1.push_back(q6_1);
        qWrist_1.push_back(q7_1);
        q567s.push_back(qWrist_1);

        // Second set
        float q6_2 = atan2( -tmp + 0.0, m23 + 0.0 );
        float s6_2 = sin(q6_2);
        float q5_2 = atan2( -m33/s6_2 + 0.0, m13/s6_2 + 0.0 );
        float q7_2 = atan2( -m22/s6_2 + 0.0, m21/s6_2 + 0.0 );
        std::vector<float> qWrist_2;
        qWrist_2.push_back(q5_2);
        qWrist_2.push_back(q6_2);
        qWrist_2.push_back(q7_2);
        q567s.push_back(qWrist_2);
    }
//    std::cout << "q567s obtained: " << q567s.size() << std::endl;
//    for (unsigned int i=0; i<q567s.size(); ++i){
//        for (unsigned int j=0; j<3; ++j)    std::cout << q567s[i][j] << " ";
//        std::cout << std::endl;
//    }


    // Join the position and orientation set of joints
//    std::cout << "q1234s / q567s " << q1234s.size() << " " << q567s.size() << std::endl;
    std::vector< std::vector<float> > rawConfigSet;
    for (unsigned int i=0; i<4; ++i){

        std::vector<float> config(7);

        config[0] = q1234s[i][0];
        config[1] = q1234s[i][1] - M_PI/2.0;   // To Kautham convention
        config[2] = q1234s[i][2];
        config[3] = q1234s[i][3];

        // First orientation cover
        config[4] =  q567s[2*i][0];
        config[5] =  q567s[2*i][1];
        config[6] =  q567s[2*i][2];
        rawConfigSet.push_back(config);

        // Second orientation cover
        config[4] =  q567s[2*i+1][0];
        config[5] =  q567s[2*i+1][1];
        config[6] =  q567s[2*i+1][2];
        rawConfigSet.push_back(config);
    }
//    std::cout << "Solutions obtained: " << solutionSet.size() << std::endl;
//    for (unsigned int i=0; i<solutionSet.size(); ++i){
//        for (unsigned int j=0; j<7; ++j)    std::cout << solutionSet[i][j] << " ";
//        std::cout << std::endl;
//    }

    // Test IK results
//    std::cout << "Testing IK positions : " << rawConfigSet.size() << std::endl;
//    for (unsigned int i=0; i<rawConfigSet.size(); ++i){
//        Eigen::VectorXf tmp_cfg(7);
//        for (unsigned int j=0; j<7; ++j)    tmp_cfg(j) = rawConfigSet[i][j];

//        // Ploting each position configuration
//        std::cout << "pos cfg " << i << ":  ";
//        for (unsigned int j=0; j<7; ++j)    std::cout << tmp_cfg(j) << " ";
//        std::cout << std::endl;

//        // DK
//        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(tmp_cfg(i-1));
//        mt::Transform TCP_TF = *_robot->getLink(7)->getTransformation();
//        plot_pose(&TCP_TF,"TCP_TF");

//        mt::Scalar y, p, r;
//        TCP_TF.getRotation().getYpr(y, p, r);
//        std::cout << "ypr " << y << " " << p << " " << r << std::endl;
//    }

    // For each obtained solution, keep only that are not within limits and that do not have NaN values
    std::vector< std::vector<float> > solutionSet;
    for (unsigned int i=0; i<rawConfigSet.size(); ++i){

        std::vector<float> cfg(rawConfigSet[i]);

        bool joint_value_changed = false;

        for (unsigned int j=0; !joint_value_changed && j<7; ++j){
            float joint_value = cfg[j];
            if ( joint_value != joint_value )      joint_value_changed = true;
            else{
                float new_value = joint_value;
                this->setJointInLimit(j, new_value);

                float threshold = 0.001;
                if ( fabs( new_value - joint_value ) > threshold )      joint_value_changed = true;
            }
        }

        if (!joint_value_changed)   solutionSet.push_back(cfg);
    }
    std::cout << solutionSet.size() << " solutions found!" << std::endl;

    bool ik_solved = false;
    Eigen::VectorXf ikSolution(7);
    if ( solutionSet.size() == 0 )  ik_solved = false;
    else{
        // Pick the first configuration in the list
        for (unsigned int j=0; j<7; ++j)    ikSolution[j] = solutionSet[0][j];

        // Test solution
        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(ikSolution(i-1));
        mt::Transform ikResult_TCP = *_robot->getLink(7)->getTransformation();
        plot_pose(&ikResult_TCP,"Solution TCP");

        ik_solved = true;
    }

    std::cout << "End INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;


//    // TEST TEST TEST TEST TEST TEST
////    ikSolution(0) = M_PI/2.0;
////    ikSolution(1) = 0.0;
////    ikSolution(2) = 0.0;
////    ikSolution(3) = 0.0;
////    ikSolution(4) = 0.0;
////    ikSolution(5) = 0.0;
////    ikSolution(6) = 0.0;
//    ikSolution(0) = 1.34;
//    ikSolution(1) = -0.56;
//    ikSolution(2) = 1.28;
//    ikSolution(3) = 0.28;
//    ikSolution(4) = -1.13;
//    ikSolution(5) = 1.94;
//    ikSolution(6) = 2.1;
////    for (unsigned int i=0; i<7; ++i)    ikSolution(i) = 0.5;


    // PLOT results -----------------------------------------------------------------------
    std::cout << " ikSolution: " << ikSolution.transpose() << std::endl;

    // Final robot poses
    for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(ikSolution(i-1));
//    for (unsigned int i=0; i<7; ++i){
//        mt::Transform ikpartial_TCP = *_robot->getLink(i)->getTransformation();
//        plot_pose(&ikpartial_TCP,"partial_TF");
//    }
    mt::Transform ikResult_TCP = *_robot->getLink(7)->getTransformation();
    plot_pose(&ikResult_TCP,"ikResult_TCP");
//    mt::Scalar y, p, r;
//    ikResult_TCP.getRotation().getYpr(y, p, r);
//    std::cout << "ypr " << y << " " << p << " " << r << std::endl;


    if (!ik_solved){
        cout << "Inverse kinematics failed:" << endl;
        cout << "Try another value for the redundant joint and set the desired pose within the reachable workspace." << endl;

        std::vector<KthReal> qn(7);
        for (unsigned int i = 0; i<7; ++i){
            _robot->getLink(i+1)->setValue(0.0);
            qn.at(i) = 0.0;
        }
        _robConf.setRn(qn);

        return false;
    }

    // Store the selection solution to kautham
    std::vector<KthReal> qn(7);
    for (unsigned int i = 0; i<7; ++i)  qn.at(i) = ikSolution[i];
    _robConf.setRn(qn);

    return true;
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


void IvKinKukaLWR::setJointInLimit(const unsigned i, float &value){

    if      ( value < this->getLimit(i,0) )   value = this->getLimit(i,0);
    else if ( value > this->getLimit(i,1) )   value = this->getLimit(i,1);

    return;
}


void IvKinKukaLWR::setJointsInLimits(Eigen::VectorXf &joints){
    for (unsigned int i=0; i<7; ++i)    this->setJointInLimit(i, joints(i));
}


float IvKinKukaLWR::normalizeJoint(const unsigned i, const float value){
    return ( value - this->getLimit(i,0) )/( this->getLimit(i,1) - this->getLimit(i,0) );
}

Eigen::VectorXf IvKinKukaLWR::normalizeJoints(const Eigen::VectorXf joints){

    Eigen::VectorXf normalized_jnts(7);

    for (unsigned int i=0; i<7; ++i)
        normalized_jnts(i) = this->normalizeJoint(i, joints(i));

    return normalized_jnts;
}


float IvKinKukaLWR::denormalizeJoint(const unsigned i, const float normalized_value){
    return ( this->getLimit(i,0) + normalized_value*( this->getLimit(i,1) - this->getLimit(i,0) ) );
}

Eigen::VectorXf IvKinKukaLWR::denormalizeJoints(const Eigen::VectorXf normalized_joints){

    Eigen::VectorXf jnts(7);

    for (unsigned int i=0; i<7; ++i)
        jnts(i) = this->denormalizeJoint(i, normalized_joints(i));

    return jnts;
}


float IvKinKukaLWR::getLimit(const unsigned int i, bool high_limit){

    if  ( !high_limit )     return *(_robot->getLink(i+1)->getLimits(1));
    else                    return *(_robot->getLink(i+1)->getLimits(0));
}
