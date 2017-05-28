
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

    auto mt_to_Eigen_pose = [](const mt::Transform* pose) {
        Eigen::Matrix4f eigenPose;
        for (unsigned int i=0; i<3; ++i){
            for (unsigned int j=0; j<3; ++j)    eigenPose(i,j) = pose->getRotation().getMatrix()[i][j];
            eigenPose(i,3) =  pose->getTranslation()[i];
        }
        return eigenPose;
    };



    std::cout << "Yumi IK ----------------------------------------------------------" << std::endl;

    std::cout << "Yumi arm is " << (this->arm_is_left() ? "left" : "right") << std::endl;

    std::cout << "_target: " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << " "
              << _target.at(3) << " " << _target.at(4) << " " << _target.at(5) << " "
              << _target.at(6) << " " << _target.at(7) << " " << std::endl;

    // Set target pose
    _targetTrans.setTranslation(mt::Point3(_target.at(0), _target.at(1), _target.at(2)));
    _targetTrans.setRotation(mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6) ));
    plot_pose(&_targetTrans,"_targetTrans");

    // Set redundant joint
    double redundantJoint = 0.0;
    if (_target.size() > 7)   redundantJoint = -2.94088 + 5.88176 * _target.at(7);    // Denormalize
    std::cout << "Redundat joint = " << redundantJoint << std::endl;


    // Declare solver
    YumiKinematics* YumiKinSolver;


    {
        // Shoulder pose in Yumi Frame
        mt::Transform* yumiShoulderPose_YumiFrame = new mt::Transform;
        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(0.0);
        yumiShoulderPose_YumiFrame = _robot->getLink(1)->getTransformation();
        plot_pose(yumiShoulderPose_YumiFrame,"yumiShoulderPose_YumiFrame");

        // Yumi arm TCP pose to Gripper Pose TF
        mt::Transform* yumiArmTCPToGripperTF = new mt::Transform;
        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(0.0);
        *yumiArmTCPToGripperTF = _robot->getLink(7)->getTransformation()->inverse() * _robot->getLastLinkTransform();
        plot_pose(yumiArmTCPToGripperTF,"yumiArmTCPToGripperTF");


        // Inverse Kinematics --------------------------------------------------------
        std::cout << "INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

        // Desired pose --------------------------------------------------------------

        //  Desired gripper pose in Yumi frame
        mt::Transform* desired_YumiGripperPose_YumiFrame = new mt::Transform;
        *desired_YumiGripperPose_YumiFrame = _targetTrans;
        plot_pose(desired_YumiGripperPose_YumiFrame,"desired_YumiGripperPose_YumiFrame");

        //  Desired TCP pose in Yumi frame
        mt::Transform* desired_YumiTCPPose_YumiFrame = new mt::Transform;
        *desired_YumiTCPPose_YumiFrame = _targetTrans * yumiArmTCPToGripperTF->inverse();

        //  TEST
        Eigen::VectorXf q_test(7);
        q_test << M_PI/6.0, -M_PI/6.0, 0.0, -M_PI/5.0, 0.0, M_PI/6.0, 0.0;
        for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(q_test(i-1));
        desired_YumiTCPPose_YumiFrame = _robot->getLink(7)->getTransformation();

        plot_pose(desired_YumiTCPPose_YumiFrame,"desired_YumiTCPPose_YumiFrame");

        // Desired TCP pose in shoulder frame
        mt::Transform* desired_YumiTCPPose_ShoulderFrame = new mt::Transform;
        *desired_YumiTCPPose_ShoulderFrame = yumiShoulderPose_YumiFrame->inverse() * (*desired_YumiTCPPose_YumiFrame);
        std::cout << "IK Test desiredYumiTCPPose_ShoulderFrame:" << std::endl;
//        plot_pose(desired_YumiTCPPose_ShoulderFrame,"desired_YumiTCPPose_ShoulderFrame");


        // Solve inverse kinematics -------------------------------
        //  Generate an initial configuration
        Eigen::VectorXf init_q(7);
        init_q << 0.0, 0.0, 0.0, -M_PI/2, 0.0, -0.0, 0.0;
        //  Use numerical method
        float max_iterations = 100;
        Eigen::VectorXf ikSolution(7);
        bool ik_solved = YumiKinSolver->NumericalIKSolver(mt_to_Eigen_pose(desired_YumiTCPPose_ShoulderFrame),
                                                          init_q, 0.0001, max_iterations,
                                                          ikSolution);

        // Final gripper pose
        ikSolution(3) = ikSolution(3) - M_PI/2.0;
        for (unsigned int i=1; i<8; ++i)    _robot->getLink(i)->setValue(ikSolution(i-1));
        mt::Transform* ikResult_YumiGripperPose_YumiFrame = new mt::Transform;
        ikResult_YumiGripperPose_YumiFrame = _robot->getLink(7)->getTransformation();
        plot_pose(ikResult_YumiGripperPose_YumiFrame,"ikResult_YumiGripperPose_YumiFrame");
        mt::Scalar y, p, r;
        ikResult_YumiGripperPose_YumiFrame->getRotation().getYpr(y, p, r);
        std::cout << "ypr " << y << " " << p << " " << r << std::endl;

        // Store the selection solution to kautham
        std::vector<KthReal> tmpik(7);
        tmpik.at(0) = ( ikSolution(0) - (-2.94088) )/(  2.94088 - (-2.94088) );
        tmpik.at(1) = ( ikSolution(1) - (-2.50455) )/( 0.759218 - (-2.50455) );
        tmpik.at(2) = ( ikSolution(2) - (-2.94088) )/(  2.94088 - (-2.94088) );
        tmpik.at(3) = ( ikSolution(3) - (-2.15548) )/(  1.39626 - (-2.15548) );
        tmpik.at(4) = ( ikSolution(4) - (-5.06145) )/(  5.06145 - (-5.06145) );
        tmpik.at(5) = ( ikSolution(5) - (-1.53589) )/(  2.40855 - (-1.53589) );
        tmpik.at(6) = ( ikSolution(6) -  (-3.9968) )/(   3.9968 -  (-3.9968) );
        //      for (unsigned int i = 0; i<7; ++i)  tmpik.at(i) = test_ikSolution[i];
        for (unsigned int i = 0; i<7; ++i)  tmpik.at(i) = 0.5;
        _robConf.setRn(tmpik);

        return ik_solved;
    }












    // Body to shoulder transformations

    //  Right Arm
    mt::Transform* dkYumiRightShoulderPose = new mt::Transform;
    dkYumiRightShoulderPose->setRotation(mt::Rotation(-2.3180,-0.5682,-0.9781));
    dkYumiRightShoulderPose->setTranslation(mt::Point3(0.05355, -0.0725, 0.41492));

    mt::Transform* yumiArmTCPToGripperTF = new mt::Transform;
    *yumiArmTCPToGripperTF = _robot->getLink(7)->getTransformation()->inverse() * _robot->getLastLinkTransform();

    //  Left Arm
    mt::Transform* dkYumiLeftShoulderPose = new mt::Transform;
    dkYumiLeftShoulderPose->setRotation(mt::Rotation(2.3180,-0.5716,0.9781));
    dkYumiLeftShoulderPose->setTranslation(mt::Point3(0.05355, 0.0725, 0.41492));

    *yumiArmTCPToGripperTF = _robot->getLink(7)->getTransformation()->inverse() * _robot->getLastLinkTransform();



    //  // Direct Kinematics --------------------------------------------------------
    //  std::cout << "DIRECT KINEMATICS TEST +++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    //  Eigen::VectorXf dkConfig(7);
    //  //        dkConfig << M_PI/6.0, -M_PI/6.0, 0.0, -M_PI/5.0, 0.0, M_PI/6.0, 0.0;
    //  //        dkConfig << 0.0, 0.0, 0.0, -M_PI/2.0, 0.0, 0.0, 0.0;
    //  dkConfig << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    //  std::cout << ">> q = " << dkConfig.transpose() << std::endl;
    //  for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));
    //  Eigen::VectorXf dkYumiConfig;
    //  Eigen::Matrix4f dkPose;
    //  mt::Transform dkYumiPose;


    //  mt::Transform* dkYumiRightTCPPose = new mt::Transform;
    //  dkYumiRightTCPPose = _robot->getLink(7)->getTransformation();
    //  dkYumiPose = dkYumiRightShoulderPose->inverse() * (*dkYumiRightTCPPose);
    //  std::cout << "Kautham RIGHT dkYumiPoseTCP:" << std::endl;
    //  for (unsigned int i=0; i<3; ++i){
    //      for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
    //      std::cout << dkYumiPose.getTranslation()[i] << std::endl;
    //  }
    //  std::cout << "det = " << std::log(dkYumiPose.getRotation().getMatrix().determinant()) << std::endl;
    //  dkYumiConfig = dkConfig;
    //  dkYumiConfig(3) = dkYumiConfig(3) + M_PI/2.0;
    //  dkPose = YumiKinSolver->ForwardKinematics(dkYumiConfig);
    //  std::cout << "IK dkPose:" << std::endl << dkPose << std::endl;


    //  mt::Transform* dkYumiLeftTCPPose = new mt::Transform;
    //  dkYumiLeftTCPPose = _robot->getLink(7)->getTransformation();
    //  dkYumiPose = dkYumiLeftShoulderPose->inverse() * (*dkYumiLeftTCPPose);
    //  std::cout << "Kautham LEFT dkYumiPoseTCP:" << std::endl;
    //  for (unsigned int i=0; i<3; ++i){
    //      for (unsigned int j=0; j<3; ++j)    std::cout << dkYumiPose.getRotation().getMatrix()[i][j] << "  ";
    //      std::cout << dkYumiPose.getTranslation()[i] << std::endl;
    //  }
    //  std::cout << "det = " << std::log(dkYumiPose.getRotation().getMatrix().determinant()) << std::endl;
    //  dkYumiConfig = dkConfig;
    //  dkYumiConfig(3) = dkYumiConfig(3) + M_PI/2.0;
    //  dkPose = YumiKinSolver->ForwardKinematics(dkYumiConfig);
    //  std::cout << "IK dkPose:" << std::endl << dkPose << std::endl;

    // Inverse Kinematics --------------------------------------------------------
    std::cout << "INVERSE KINEMATICS ++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    Eigen::VectorXf init_q(7);
    Eigen::Matrix4f eigIkTestPose(Eigen::Matrix4f::Identity());
    Eigen::VectorXf test_ikSolution, ikKauthamSolution;
    mt::Transform* ikPose = new mt::Transform;
    mt::Transform* ikTestPose = new mt::Transform;
    bool ik_solved = false;
    float max_iterations = 0;

    Eigen::VectorXf dkConfig(7);
    dkConfig << M_PI/6.0, +M_PI/6.0, 0.0, -M_PI/5.0, 0.0, M_PI/6.0, 0.0;
    init_q << 0.0, 0.0, 0.0, -M_PI/2, 0.0, -0.0, 0.0;

    // To right arm reference ---------------------------------------
    // Compute test pose
    for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));
    ikTestPose = _robot->getLink(7)->getTransformation();
    std::cout << "IK RightTest Pose: -------------------------------" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikTestPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikTestPose->getTranslation()[i] << std::endl;
    }
    *ikTestPose = _targetTrans * yumiArmTCPToGripperTF->inverse();
    std::cout << "IK RightTest Pose: -------------------------------" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikTestPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikTestPose->getTranslation()[i] << std::endl;
    }

    mt::Transform* ikRightTestPose = new mt::Transform;
    *ikRightTestPose = dkYumiRightShoulderPose->inverse() * (*ikTestPose);
    //        std::cout << "IK Test ikRightTestPose:" << std::endl;
    //        for (unsigned int i=0; i<3; ++i){
    //            for (unsigned int j=0; j<3; ++j)    std::cout << ikRightTestPose->getRotation().getMatrix()[i][j] << "  ";
    //            std::cout << ikRightTestPose->getTranslation()[i] << std::endl;
    //        }

    // Translate to eigen
    eigIkTestPose = Eigen::Matrix4f::Identity();
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    eigIkTestPose(i,j) = ikRightTestPose->getRotation().getMatrix()[i][j];
        eigIkTestPose(i,3) =  ikRightTestPose->getTranslation()[i];
    }

    //        init_q = dkConfig;
    //      init_q(3) = dkConfig(3) + M_PI/2.0;
    max_iterations = 100;
    ik_solved = YumiKinSolver->NumericalIKSolver(eigIkTestPose, init_q, 0.0001, max_iterations, test_ikSolution);

    ikKauthamSolution = test_ikSolution;
    ikKauthamSolution(3) = test_ikSolution(3) - M_PI/2.0;
    for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(ikKauthamSolution(i-1));
    ikPose = _robot->getLink(7)->getTransformation();
    std::cout << "IK Right Arm Final Pose:" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikPose->getTranslation()[i] << std::endl;
    }

    // Store the selection solution
    std::vector<KthReal> tmpik(7);
    tmpik.at(0) = ( test_ikSolution(0) - (-2.94088) )/(  2.94088 - (-2.94088) );
    tmpik.at(1) = ( test_ikSolution(1) - (-2.50455) )/( 0.759218 - (-2.50455) );
    tmpik.at(2) = ( test_ikSolution(2) - (-2.94088) )/(  2.94088 - (-2.94088) );
    tmpik.at(3) = ( test_ikSolution(3) - (-2.15548) )/(  1.39626 - (-2.15548) );
    tmpik.at(4) = ( test_ikSolution(4) - (-5.06145) )/(  5.06145 - (-5.06145) );
    tmpik.at(5) = ( test_ikSolution(5) - (-1.53589) )/(  2.40855 - (-1.53589) );
    tmpik.at(6) = ( test_ikSolution(6) -  (-3.9968) )/(   3.9968 -  (-3.9968) );
    //      for (unsigned int i = 0; i<7; ++i)  tmpik.at(i) = test_ikSolution[i];
    for (unsigned int i = 0; i<7; ++i)  tmpik.at(i) = 0.5;
    _robConf.setRn(tmpik);

    return ik_solved;


    // To left arm reference ---------------------------------------
    // Compute test pose
    for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(dkConfig(i-1));
    ikTestPose = _robot->getLink(7)->getTransformation();
    std::cout << "IK Left Test Pose: -------------------------------" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikTestPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikTestPose->getTranslation()[i] << std::endl;
    }
    *ikTestPose = _targetTrans;
    std::cout << "IK Left Test Pose: -------------------------------" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikTestPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikTestPose->getTranslation()[i] << std::endl;
    }

    mt::Transform* ikLeftTestPose = new mt::Transform;
    *ikLeftTestPose = dkYumiLeftShoulderPose->inverse() * (*ikTestPose);
    //        std::cout << "IK Test ikLeftTestPose:" << std::endl;
    //        for (unsigned int i=0; i<3; ++i){
    //            for (unsigned int j=0; j<3; ++j)    std::cout << ikLeftTestPose->getRotation().getMatrix()[i][j] << "  ";
    //            std::cout << ikLeftTestPose->getTranslation()[i] << std::endl;
    //        }

    // Translate to eigen
    eigIkTestPose = Eigen::Matrix4f::Identity();
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    eigIkTestPose(i,j) = ikLeftTestPose->getRotation().getMatrix()[i][j];
        eigIkTestPose(i,3) =  ikLeftTestPose->getTranslation()[i];
    }

    //        init_q = dkConfig;
    init_q(3) = dkConfig(3) + M_PI/2.0;
    max_iterations = 30;
    ik_solved = YumiKinSolver->NumericalIKSolver(eigIkTestPose, init_q, 0.0001, max_iterations, test_ikSolution);

    ikKauthamSolution = test_ikSolution;
    ikKauthamSolution(3) = test_ikSolution(3) - M_PI/2.0;
    for (unsigned int i=1; i<8; ++i)      _robot->getLink(i)->setValue(ikKauthamSolution(i-1));
    ikPose = _robot->getLink(7)->getTransformation();
    std::cout << "IK Left Arm Final Pose:" << std::endl;
    for (unsigned int i=0; i<3; ++i){
        for (unsigned int j=0; j<3; ++j)    std::cout << ikPose->getRotation().getMatrix()[i][j] << "  ";
        std::cout << ikPose->getTranslation()[i] << std::endl;
    }


    return false;



    // Solve IK

    // Get all possible 16 analitic ik configurations
    Eigen::Matrix4f desiredPoseInArmRef;
    std::vector< std::vector<float> > yumiAnalyticalIkSolutions = YumiKinSolver->AnalyticalIKSolver(desiredPoseInArmRef, redundantJoint);
    //std::cout << "Yumi IK solutions: " << yumiAnalyticalIkSolutions.size() << std::endl; // PLOT PLOT PLOT PLOT PLOT PLOT PLOT PLOT

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
        Eigen::VectorXf test_ikSolution(7);
        max_iterations = 100;
        bool ik_solved = YumiKinSolver->NumericalIKSolver(test_pose, test_q, 0.01, max_iterations, test_ikSolution);
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
        Eigen::VectorXf ikSolution(7);
        max_iterations = 100;
        ik_solved = YumiKinSolver->NumericalIKSolver(desiredPoseInArmRef, minAnalyticalIkSolution, 0.001, max_iterations, ikSolution);

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
