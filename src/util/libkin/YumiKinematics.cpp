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

/* Author: Isiah Zaplana, Aliakbar Akbari, Muhayyudin, Josep-Arnau Claret */

// This library follows the work in:
//  "Analisis cinemático de robots manipuladores redundantes: aaplicacion a los robots Kuka LWR 4+ y ABB Yumi"
//    Isiah Zaplana, Josep-Arnau Claret y Luis Basañez
//    Revista Iberoamericana de Automática e Informática Industrial RIAI
//  2017


#include <stdlib.h>     /* srand, rand */
#include <iostream>
#include <math.h>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "YumiKinematics.h"


#define IK_NUMERICAL_INITIAL_Q 0
#define IK_NUMERICAL_RND_INITIAL_QS 1
#define IK_ANALYTICAL_Q  2


std::vector< std::vector<float> > YumiKinematics::AnalyticalIKSolver(const Eigen::Matrix4f Pose, const float theta3)
{
    std::vector< std::vector<float> > SolutionSet1;
    std::vector< std::vector<float> > SolutionSet2;
    std::vector< std::vector<float> > Solutions;
    //  std::cout<<"Computing Theta4"<<std::endl;
    SolutionSet1 = ComputeTheta4(Pose, theta3,  0);
    //   std::cout<<"Computing Theta4"<<std::endl;
    SolutionSet2 = ComputeTheta4(Pose, theta3,  1);
    //    std::cout<<"computed Theta4"<<std::endl;


    for(unsigned int i=0;i<SolutionSet1.size();i++){
        SolutionSet1[i][3] = SolutionSet1[i][3] - M_PI/2.0;   // Remove the offset from the inner IK kin model to the Kautham library kin model
        Solutions.push_back(SolutionSet1[i]);
    }

    for(unsigned int i=0;i<SolutionSet2.size();i++){
        SolutionSet2[i][3] = SolutionSet2[i][3] - M_PI/2.0;   // Remove the offset from the inner IK kin model to the Kautham library kin model
        Solutions.push_back(SolutionSet2[i]);
    }

    return Solutions;
}

std::vector< std::vector<float> > YumiKinematics::ComputeTheta4(const Eigen::Matrix4f Pose, const float theta3, const unsigned int byn)
{
    std::vector< std::vector<float> > SolutionSet1;
    std::vector< std::vector<float> > SolutionSet2;
    std::vector< std::vector<float> > Solutions;
    float theta4;

    // Original
//    float a=130014.5;
//    float b=-41836.5;
//    float c= Pose(0,3)*Pose(0,3)+Pose(1,3)*Pose(1,3)+(Pose(2,3)-166)*(Pose(2,3)-166)-136757.75;

//    float a=130014.5;
//    float b=-41836.5;
////    float c= Pose(0,3)*Pose(0,3)+Pose(1,3)*Pose(1,3)+(Pose(2,3)-166)*(Pose(2,3)-166)-136757.75;
//    float c= 1000.0*Pose(0,3)*1000.0*Pose(0,3)+1000.0*Pose(1,3)*1000.0*Pose(1,3)+(1000.0*Pose(2,3)-166)*(1000.0*Pose(2,3)-166)-136757.75;    // To mm

    float a = 125518.22;   // TODO
    float b = 41112.36;   // TODO
    float c = 1000.0*Pose(0,3)*1000.0*Pose(0,3)+1000.0*Pose(1,3)*1000.0*Pose(1,3)+(1000.0*Pose(2,3)-100)*(1000.0*Pose(2,3)-100)-132098.93;    // TODO

    if(byn==0)
    {
        theta4= atan2(c,sqrt(a*a+b*b-c*c))-atan2(a,b);
        //    std::cout<<"Computing first Theta4 0 "<<theta4<<std::endl;
        SolutionSet1 = ComputeTheta2(Pose, theta3, theta4,0);
        //        std::cout<<"Computing SolutionSet1 0 "<<std::endl;
        //            for(int i=0; i<SolutionSet1.size(); i++)
        //            {
        //                for(int j=0; j<SolutionSet1[i].size(); j++)
        //                {
        //                std::cout<<SolutionSet1[i][j]<<" ";
        //                }
        //                std::cout<<std::endl;
        //            }
        SolutionSet2 = ComputeTheta2(Pose, theta3, theta4,1);
        //     std::cout<<"Computed Theta4"<<std::endl;

    }
    if(byn==1)
    {
        theta4= atan2(c,-sqrt(a*a+b*b-c*c))-atan2(a,b);
        //  std::cout<<"value of c, a, b"<<c<<a<<b<<std::endl;
        //    std::cout<<"value of theta4 1 "<<theta4<<std::endl;
        SolutionSet1 = ComputeTheta2(Pose, theta3, theta4,0);
        SolutionSet2 = ComputeTheta2(Pose, theta3, theta4,1);
    }

    for(unsigned int i=0;i<SolutionSet1.size();i++)
        Solutions.push_back(SolutionSet1[i]);

    for(unsigned int i=0;i<SolutionSet2.size();i++)
        Solutions.push_back(SolutionSet2[i]);
    return Solutions;

}

std::vector< std::vector<float> > YumiKinematics::ComputeTheta2(const Eigen::Matrix4f Pose, const float theta3, const float theta4, const unsigned int byn)
{
    std::vector< std::vector<float> > SolutionSet1;
    std::vector< std::vector<float> > SolutionSet2;
    std::vector< std::vector<float> > Solutions;
    float theta2;

    // Original
//    float a=251.5+265*cos(theta4)-40.5*sin(theta4);
//    float b=40.5*cos(theta3)-40.5*cos(theta3)*cos(theta4)-265*cos(theta3)*sin(theta4);
//    float c=Pose(2,3)-166;

//    float a=251.5+265*cos(theta4)-40.5*sin(theta4);
//    float b=40.5*cos(theta3)-40.5*cos(theta3)*cos(theta4)-265*cos(theta3)*sin(theta4);
////    float c=Pose(2,3)-166;
//    float c=1000.0*Pose(2,3)-166;  // To mm

    float a=251.56+256.0*cos(theta4)+40.5*sin(theta4);
    float b=-40.5*cos(theta3)+40.5*cos(theta3)*cos(theta4)-256.0*cos(theta3)*sin(theta4);
    float c=1000.0*Pose(2,3)-100;  // To mm


    //  std::cout<<"Computing Theta2 a "<<a<<std::endl;
    //  std::cout<<"Computing Theta2 b "<<b<<std::endl;
    //  std::cout<<"Computing Theta2 c "<<c<<std::endl;

    if(byn==0)
    {
        theta2=atan2(c,sqrt(a*a+b*b-c*c))-atan2(a,b);
        //    std::cout<<"Computing Theta2 0 "<<theta2<<std::endl;
        SolutionSet1 = ComputeTheta1(Pose,theta2,theta3,theta4,0);
        //      std::cout<<"Computing Theta1"<<std::endl;
        SolutionSet2 = ComputeTheta1(Pose,theta2,theta3,theta4,1);
        //     std::cout<<"Computedc Theta1"<<std::endl;

    }
    if(byn==1)
    {
        theta2=atan2(c,-sqrt(a*a+b*b-c*c))-atan2(a,b);
        //    std::cout<<"value of a, b"<<" /// " <<a<<" /// "<<b<<std::endl;
        //   std::cout<<"value of theta2"<<theta4<<std::endl;
        SolutionSet1 = ComputeTheta1(Pose,theta2,theta3,theta4,0);
        SolutionSet2 = ComputeTheta1(Pose,theta2,theta3,theta4,1);
    }
    for(unsigned int i=0;i<SolutionSet1.size();i++)
        Solutions.push_back(SolutionSet1[i]);

    for(unsigned int i=0;i<SolutionSet2.size();i++)
        Solutions.push_back(SolutionSet2[i]);
    return Solutions;
}

std::vector< std::vector<float> > YumiKinematics::ComputeTheta1(const Eigen::Matrix4f Pose, const float theta2, const float theta3, const float theta4, const unsigned int byn)
{
    std::vector<float> Solution1;
    std::vector<float> Solution2;
    std::vector< std::vector<float> > Solutions;
    float theta1;

    // Original
//    float a=40.5*cos(theta2)*cos(theta3)-251.5*sin(theta2)-265*cos(theta4)*sin(theta2)+40.5*sin(theta2)*sin(theta4)-40.5*cos(theta2)*
//            cos(theta3)*cos(theta4)-265*cos(theta2)*cos(theta3)*sin(theta4);
//    float b=265*sin(theta3)*sin(theta4)-40.5*sin(theta3)+40.5*sin(theta3)*cos(theta4);

//    float a=40.5*cos(theta2)*cos(theta3)-251.5*sin(theta2)-265*cos(theta4)*sin(theta2)+40.5*sin(theta2)*sin(theta4)-40.5*cos(theta2)*
//            cos(theta3)*cos(theta4)-265*cos(theta2)*cos(theta3)*sin(theta4);
//    float b=265*sin(theta3)*sin(theta4)-40.5*sin(theta3)+40.5*sin(theta3)*cos(theta4);

    float a=-40.5*cos(theta2)*cos(theta3)-251.56*sin(theta2)-256.0*cos(theta4)*sin(theta2)-40.5*sin(theta2)*sin(theta4)+40.5*cos(theta2)*
            cos(theta3)*cos(theta4)-256.0*cos(theta2)*cos(theta3)*sin(theta4);
    float b=256.0*sin(theta3)*sin(theta4)+40.5*sin(theta3)-40.5*sin(theta3)*cos(theta4);

    //   std::cout<<"value of theta1"<<" /// " <<a<<" /// "<<b<<std::endl;

    float c=Pose(0,3)*1000.0;  // In mm
    if(byn==0)
    {
        theta1=atan2(c,sqrt(a*a+b*b-c*c))-atan2(a,b);
        //      std::cout<<"first value of a, b"<<" /// " <<a<<" /// "<<b<<std::endl;

        //     std::cout<<"Computing first theta "<<theta1<<std::endl;
        Solution1 = ComputeRotation(Pose,  theta1, theta2, theta3,  theta4, 0);
        // std::cout<<"Computing rotation"<<std::endl;
        Solution2 = ComputeRotation(Pose,  theta1, theta2, theta3,  theta4, 1);
        // std::cout<<"Computed rotation"<<std::endl;
    }
    if(byn==1)
    {
        theta1=atan2(c,-sqrt(a*a+b*b-c*c))-atan2(a,b);
        //      std::cout<<"value of theta1"<<theta1<<std::endl;

        //std::cout<<"Computing rotation"<<std::endl;
        Solution1 = ComputeRotation(Pose,  theta1, theta2, theta3,  theta4, 0);
        // std::cout<<"Computing rotation"<<std::endl;
        Solution2 = ComputeRotation(Pose,  theta1, theta2, theta3,  theta4, 1);
        // std::cout<<"Computed rotation"<<std::endl;
    }
    Solutions.push_back(Solution1);
    Solutions.push_back(Solution2);

    return Solutions;
}

std::vector<float>  YumiKinematics::ComputeRotation(const Eigen::Matrix4f Pose, const float theta1, const float theta2, const float theta3, const float theta4, const unsigned int byn)
{
    //  std::cout<<"Entered in Rot function"<<std::endl;
    //  std::cout<<"Theta1 theta2 theta3 "<<theta1<<theta2<<theta3<<theta4<<std::endl;

    std::vector<float> q;
    Eigen::Matrix3f M;
    Eigen::Matrix3f R;
    Eigen::Matrix3f Rpos;

    //std::cout<<"Reading Pose"<<std::endl;
    Rpos(0,0) = Pose(0,0); Rpos(0,1) = Pose(0,1); Rpos(0,2) = Pose(0,2);
    Rpos(1,0) = Pose(1,0); Rpos(1,1) = Pose(1,1); Rpos(1,2) = Pose(1,2);
    Rpos(2,0) = Pose(2,0); Rpos(2,1) = Pose(2,1); Rpos(2,2) = Pose(2,2);
    //std::cout<<"Reading Pose"<<std::endl;

    R(0,0) = cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)-cos(theta4)*sin(theta1)*sin(theta3)-cos(theta1)*sin(theta2)*sin(theta4);
    R(0,1) = cos(theta1)*cos(theta4)*sin(theta3)-sin(theta1)*sin(theta2)*sin(theta4)+cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1);
    R(0,2) = cos(theta2)*sin(theta4)+cos(theta3)*cos(theta4)*sin(theta2);
    //std::cout<<"Reading Pose"<<std::endl;

    R(1,0) = sin(theta1)*sin(theta3)*sin(theta4)-cos(theta1)*cos(theta4)*sin(theta2)-cos(theta1)*cos(theta2)*cos(theta3)*sin(theta4);
    R(1,1) = -cos(theta4)*sin(theta1)*sin(theta2)-cos(theta1)*sin(theta3)*sin(theta4)-cos(theta2)*cos(theta3)*sin(theta1)*sin(theta4);
    R(1,2) = cos(theta2)*cos(theta4)-cos(theta3)*sin(theta2)*sin(theta4);
    //std::cout<<"Reading Pose"<<std::endl;

    R(2,0) = cos(theta3)*sin(theta1)+cos(theta1)*cos(theta2)*sin(theta3);
    R(2,1) = cos(theta2)*sin(theta1)*sin(theta3)-cos(theta1)*cos(theta3);
    R(2,2) = sin(theta2)*sin(theta3);
    //std::cout<<"Reading Pose"<<std::endl;

    M = R*Rpos;
    //  std::cout<<M<<std::endl;
    if(byn==0)
    {
        float theta6 = atan2(sqrt(M(0,2)*M(0,2)+M(2,2)*M(2,2)),M(1,2));
        float theta7 = atan2(M(2,2)/sin(theta6),-(M(0,2))/sin(theta6));
        float theta5 = atan2(M(1,1)/sin(theta6),M(1,0)/sin(theta6));
        q.push_back(theta1); q.push_back(theta2); q.push_back(theta3); q.push_back(theta4);
        q.push_back(theta5); q.push_back(theta6); q.push_back(theta7);
    }
    if(byn==1)
    {
        float theta6 = atan2(-sqrt(M(2,2)*M(2,2)+M(0,2)*M(0,2)),M(1,2));
        float theta7 = atan2(M(2,2)/sin(theta6),-(M(0,2))/sin(theta6));
        float theta5 = atan2(M(1,1)/sin(theta6),M(1,0)/sin(theta6));
        //   std::cout<<"Theta6 theta7 theta5 "<<theta6<<theta7<<theta5<<std::endl;
        q.push_back(theta1); q.push_back(theta2); q.push_back(theta3); q.push_back(theta4);
        q.push_back(theta5); q.push_back(theta6); q.push_back(theta7);
    }

    return q;

}

Eigen::MatrixXf YumiKinematics::Jacobian(const Eigen::VectorXf Q)
{
    Eigen::VectorXf q(Q);
    q(3) = q(3) + M_PI/2.0;   // Add the offset from the Kautham kin model to the inner IK library kin model

    Eigen::Matrix4f A1,A2,A3,A4,A5,A6,A7;
//    A1 << cos(q(0)), -sin(q(0)), 0, 0, sin(q(0)), cos(q(0)), 0, 0, 0, 0, 1, 166, 0, 0, 0, 1;
//    A2 << cos(q(1)), -sin(q(1)), 0, 30, 0, 0, -1, 0, sin(q(1)), cos(q(1)), 0, 0, 0, 0, 0, 1;
//    A3 << cos(q(2)), -sin(q(2)), 0, -30, 0, 0, 1, 0.2515e3, -sin(q(2)), -cos(q(2)), 0, 0, 0, 0, 0, 1;
//    A4 << cos(q(3)), -sin(q(3)), 0, 40.5, 0, 0, -1, 0, sin(q(3)), cos(q(3)), 0, 0, 0, 0, 0, 1;
//    A5 << cos(q(4)), -sin(q(4)), 0, -40.5, 0, 0, 1, 265, -sin(q(4)), -cos(q(4)), 0, 0, 0, 0, 0, 1;
//    A6 << cos(q(5)), -sin(q(5)), 0, 27, 0, 0, -1, 0, sin(q(5)), cos(q(5)), 0, 0, 0, 0, 0, 1;
//    A7 << cos(q(6)), -sin(q(6)), 0, -27, 0, 0, 1, 36, -sin(q(6)), -cos(q(6)), 0, 0, 0, 0, 0, 1;

    A1 << cos(q(0)), -sin(q(0)), 0, 0,       sin(q(0)), cos(q(0)), 0, 0,        0, 0, 1, 0,                        0, 0, 0, 1;
    A2 << cos(q(1)), -sin(q(1)), 0, 30,      0, 0, -1, 0,                       sin(q(1)), cos(q(1)), 0, 100,      0, 0, 0, 1;
    A3 << cos(q(2)), -sin(q(2)), 0, -30,     0, 0, 1, 172.83,                   -sin(q(2)), -cos(q(2)), 0, 0,      0, 0, 0, 1;
    A4 << -sin(q(3)), -cos(q(3)), 0, -41.88, 0, 0, -1, 0,                       cos(q(3)), -sin(q(3)), 0, 78.73,   0, 0, 0, 1;
    A5 << 0, 0, 1, 164.61,                   -cos(q(4)), sin(q(4)), 0, -40.5,   -sin(q(4)), -cos(q(4)), 0, 0,      0, 0, 0, 1;
    A6 << cos(q(5)), -sin(q(5)), 0, -27,     0, 0, -1, 0,                       sin(q(5)), cos(q(5)), 0, 100.39,   0, 0, 0, 1;
    A7 << cos(q(6)), -sin(q(6)), 0, 27,      0, 0, 1, 29,                       -sin(q(6)), -cos(q(6)), 0, 0,      0, 0, 0, 1;

    Eigen::Matrix4f T1 = A1;
    Eigen::Matrix4f T2 = T1*A2;
    Eigen::Matrix4f T3 = T2*A3;
    Eigen::Matrix4f T4 = T3*A4;
    Eigen::Matrix4f T5 = T4*A5;
    Eigen::Matrix4f T6 = T5*A6;
    Eigen::Matrix4f T7 = T6*A7;

    // Set in metres
    for (unsigned int i=0; i<3; ++i){
        T1(i,3) /= 1000.0;
        T2(i,3) /= 1000.0;
        T3(i,3) /= 1000.0;
        T4(i,3) /= 1000.0;
        T5(i,3) /= 1000.0;
        T6(i,3) /= 1000.0;
        T7(i,3) /= 1000.0;
    }

    Eigen::Vector3f o1(T1(0,3),T1(1,3),T1(2,3));
    Eigen::Vector3f z1(T1(0,2),T1(1,2),T1(2,2));
    Eigen::Vector3f o2(T2(0,3),T2(1,3),T2(2,3));
    Eigen::Vector3f z2(T2(0,2),T2(1,2),T2(2,2));
    Eigen::Vector3f o3(T3(0,3),T3(1,3),T3(2,3));
    Eigen::Vector3f z3(T3(0,2),T3(1,2),T3(2,2));
    Eigen::Vector3f o4(T4(0,3),T4(1,3),T4(2,3));
    Eigen::Vector3f z4(T4(0,2),T4(1,2),T4(2,2));
    Eigen::Vector3f o5(T5(0,3),T5(1,3),T5(2,3));
    Eigen::Vector3f z5(T5(0,2),T5(1,2),T5(2,2));
    Eigen::Vector3f o6(T6(0,3),T6(1,3),T6(2,3));
    Eigen::Vector3f z6(T6(0,2),T6(1,2),T6(2,2));
    Eigen::Vector3f o7(T7(0,3),T7(1,3),T7(2,3));
    Eigen::Vector3f z7(T7(0,2),T7(1,2),T7(2,2));

    Eigen::Vector3f J1 = z1.cross(o7-o1);
    Eigen::Vector3f J2 = z2.cross(o7-o2);
    Eigen::Vector3f J3 = z3.cross(o7-o3);
    Eigen::Vector3f J4 = z4.cross(o7-o4);
    Eigen::Vector3f J5 = z5.cross(o7-o5);
    Eigen::Vector3f J6 = z6.cross(o7-o6);
    Eigen::Vector3f J7 = z7.cross(o7-o7);

    Eigen::MatrixXf J(6,7);
    J(0,0) = J1(0); J(0,1) = J2(0); J(0,2) = J3(0); J(0,3) = J4(0); J(0,4) = J5(0); J(0,5) = J6(0); J(0,6) = J7(0);
    J(1,0) = J1(1); J(1,1) = J2(1); J(1,2) = J3(1); J(1,3) = J4(1); J(1,4) = J5(1); J(1,5) = J6(1); J(1,6) = J7(1);
    J(2,0) = J1(2); J(2,1) = J2(2); J(2,2) = J3(2); J(2,3) = J4(2); J(2,4) = J5(2); J(2,5) = J6(2); J(2,6) = J7(2);
    J(3,0) = z1(0); J(3,1) = z2(0); J(3,2) = z3(0); J(3,3) = z4(0); J(3,4) = z5(0); J(3,5) = z6(0); J(3,6) = z7(0);
    J(4,0) = z1(1); J(4,1) = z2(1); J(4,2) = z3(1); J(4,3) = z4(1); J(4,4) = z5(1); J(4,5) = z6(1); J(4,6) = z7(1);
    J(5,0) = z1(2); J(5,1) = z2(2); J(5,2) = z3(2); J(5,3) = z4(2); J(5,4) = z5(2); J(5,5) = z6(2); J(5,6) = z7(2);
    return J;
}


bool YumiKinematics::NumericalIKSolver(const Eigen::Matrix4f desiredPose, const Eigen::VectorXf qIni,
                                       const float threshold, float& max_iterations,
                                       Eigen::VectorXf& qResult,
                                       const bool use_joint_saturation)
{
    //  Auxiliar lambda function for error norm computation
    auto computeNormError = [](const Eigen::VectorXf errorP, const Eigen::VectorXf errorO) {
//        return (std::sqrt(errorO.squaredNorm())+std::sqrt(errorP.squaredNorm()))/2.0;
//        return std::sqrt( errorP(0)*errorP(0)+errorP(1)*errorP(1)+errorP(2)*errorP(2) + errorO(0)*errorO(0)+errorO(1)*errorO(1)+errorO(2)*errorO(2) );
        return ( (1.0/17.45329444)*std::sqrt(errorO.squaredNorm()) + std::sqrt(errorP.squaredNorm()) )/2.0;    // 0.001 m ~= 1º (0.01745329444 rad)
    };


    // Desired pose
    Eigen::Vector3f pd(desiredPose(0,3), desiredPose(1,3), desiredPose(2,3));
    Eigen::Matrix3f Rd;
    Rd << desiredPose(0,0), desiredPose(0,1), desiredPose(0,2),
          desiredPose(1,0), desiredPose(1,1), desiredPose(1,2),
          desiredPose(2,0), desiredPose(2,1), desiredPose(2,2);
    Eigen::Quaternionf uqd(Rd);
//    std::cout << "   IK Num --> IK Desired Pose:" << std::endl << desiredPose << std::endl;

    // Initial configuration
    Eigen::VectorXf q(qIni), past_q(qIni);
//    std::cout << "   IK Num --> IK Initial q: " << q.transpose() << std::endl;
//    std::cout << "   IK Num --> IK Initial Pose:" << std::endl << this->ForwardKinematics(q) << std::endl;

    Eigen::Matrix4f currentPose = this->ForwardKinematics(q);
    Eigen::Vector3f pe;
    pe << currentPose(0,3), currentPose(1,3), currentPose(2,3);
    Eigen::Matrix3f Re;
    Re << currentPose(0,0), currentPose(0,1), currentPose(0,2),
          currentPose(1,0), currentPose(1,1), currentPose(1,2),
          currentPose(2,0), currentPose(2,1), currentPose(2,2);
    Eigen::Quaternionf uqe(Re);

    // Error pose
    Eigen::Vector3f errorP(pd-pe);
    Eigen::Vector3f ud(uqd.x(),uqd.y(),uqd.z());
    Eigen::Vector3f ue(uqe.x(),uqe.y(),uqe.z());
    Eigen::Matrix3f m;
    m <<        0, -uqd.z(),  uqd.y(),
          uqd.z(),        0, -uqd.x(),
         -uqd.y(),  uqd.x(),        0;
    Eigen::Vector3f errorO(uqe.w()*ud-uqd.w()*ue-m*ue);

    Eigen::VectorXf error(6);
    error << errorP(0), errorP(1), errorP(2), errorO(0), errorO(1), errorO(2);

    float past_e = 100000;
    float e = computeNormError(errorP, errorO);


    // Loop
    unsigned int n_it = 0;
    while ( (e > threshold)
            && (n_it < max_iterations)
            && ( fabs(past_e - e) > threshold/5.0 )
            ){

//        std::cout << "     IK Num --> Loop: q_ini: " << q.transpose() << std::endl;

        past_q = q;
        Eigen::MatrixXf J(this->Jacobian(q));
        Eigen::MatrixXf pinvJ(this->pseudoInverse(J));
        q = q + 0.5 * pinvJ * error;
        if (use_joint_saturation)   this->setJointsInLimits(q);

        // Next pose -------------------
        Eigen::Matrix4f nextPose = this->ForwardKinematics(q);
//        std::cout << "     IK Num --> Loop: IK nextPose:" << std::endl << nextPose << std::endl;

        pe << nextPose(0,3), nextPose(1,3), nextPose(2,3);
        Re << nextPose(0,0), nextPose(0,1), nextPose(0,2),
              nextPose(1,0), nextPose(1,1), nextPose(1,2),
              nextPose(2,0), nextPose(2,1), nextPose(2,2);
        uqe = Eigen::Quaternionf(Re);

        // Error pose ---------------------
        errorP = pd-pe;

        ud << uqd.x(),uqd.y(),uqd.z();
        ue << uqe.x(),uqe.y(),uqe.z();
        m <<        0, -uqd.z(),  uqd.y(),
              uqd.z(),        0, -uqd.x(),
             -uqd.y(),  uqd.x(),        0;
        errorO = uqe.w()*ud-uqd.w()*ue-m*ue;

        //  Error vector
        error << errorP(0), errorP(1), errorP(2), errorO(0), errorO(1), errorO(2);

        // Norm error computation
        past_e = e;
        e = computeNormError(errorP, errorO);

        ++n_it;

//        std::cout << n_it << "     IK Num --> Loop: e = " << e << std::endl;
    }
    max_iterations = n_it;

#ifdef VERBOSE_ON
    std::cout << "   IK Num --> IK numerical " << ((e < threshold) ? "" : "NOT ") << "SOLVED in " << n_it << " iterations"
              << " : e = " << e << ((e < threshold) ? " < " : " > ") << threshold << std::endl;
#endif

//    std::cout << "   IK Num --> IK Final q: " << q.transpose() << std::endl;
//    std::cout << "   IK Num --> IK Final Pose:" << std::endl << ForwardKinematics(q) << std::endl;

    qResult = q;

    if ( e < threshold )    return true;
    else                    return false;
}

Eigen::Matrix4f YumiKinematics::ForwardKinematics(const Eigen::VectorXf Q)
{
    Eigen::VectorXf q(Q.size());
    q(0)=Q(0);
    q(1)=Q(1);
    q(2)=Q(2);
    q(3)=Q(3) + M_PI/2.0;   // Add the offset from the Kautham kin model to the inner IK library kin model
    q(4)=Q(4);
    q(5)=Q(5);
    q(6)=Q(6);

    Eigen::Matrix4f A1,A2,A3,A4,A5,A6,A7;

//    A1 << cos(q(0)), -sin(q(0)), 0, 0, sin(q(0)), cos(q(0)), 0, 0, 0, 0, 1, 166, 0, 0, 0, 1;
//    A2 << cos(q(1)), -sin(q(1)), 0, 30, 0, 0, -1, 0, sin(q(1)), cos(q(1)), 0, 0, 0, 0, 0, 1;
//    A3 << cos(q(2)), -sin(q(2)), 0, -30, 0, 0, 1, 0.2515e3, -sin(q(2)), -cos(q(2)), 0, 0, 0, 0, 0, 1;
//    A4 << cos(q(3)), -sin(q(3)), 0, 40.5, 0, 0, -1, 0, sin(q(3)), cos(q(3)), 0, 0, 0, 0, 0, 1;
//    A5 << cos(q(4)), -sin(q(4)), 0, -40.5, 0, 0, 1, 265, -sin(q(4)), -cos(q(4)), 0, 0, 0, 0, 0, 1;
//    A6 << cos(q(5)), -sin(q(5)), 0, 27, 0, 0, -1, 0, sin(q(5)), cos(q(5)), 0, 0, 0, 0, 0, 1;
//    A7 << cos(q(6)), -sin(q(6)), 0, -27, 0, 0, 1, 36, -sin(q(6)), -cos(q(6)), 0, 0, 0, 0, 0, 1;

    A1 << cos(q(0)), -sin(q(0)), 0, 0,       sin(q(0)), cos(q(0)), 0, 0,        0, 0, 1, 0,                        0, 0, 0, 1;
    A2 << cos(q(1)), -sin(q(1)), 0, 30,      0, 0, -1, 0,                       sin(q(1)), cos(q(1)), 0, 100,      0, 0, 0, 1;
    A3 << cos(q(2)), -sin(q(2)), 0, -30,     0, 0, 1, 172.83,                   -sin(q(2)), -cos(q(2)), 0, 0,      0, 0, 0, 1;
    A4 << -sin(q(3)), -cos(q(3)), 0, -41.88, 0, 0, -1, 0,                       cos(q(3)), -sin(q(3)), 0, 78.73,   0, 0, 0, 1;
    A5 << 0, 0, 1, 164.61,                   -cos(q(4)), sin(q(4)), 0, -40.5,   -sin(q(4)), -cos(q(4)), 0, 0,      0, 0, 0, 1;
    A6 << cos(q(5)), -sin(q(5)), 0, -27,     0, 0, -1, 0,                       sin(q(5)), cos(q(5)), 0, 100.39,   0, 0, 0, 1;
    A7 << cos(q(6)), -sin(q(6)), 0, 27,      0, 0, 1, 29,                       -sin(q(6)), -cos(q(6)), 0, 0,      0, 0, 0, 1;

    Eigen::Matrix4f T1 = A1;
    Eigen::Matrix4f T2 = T1*A2;
    Eigen::Matrix4f T3 = T2*A3;
    Eigen::Matrix4f T4 = T3*A4;
    Eigen::Matrix4f T5 = T4*A5;
    Eigen::Matrix4f T6 = T5*A6;
    Eigen::Matrix4f T7 = T6*A7;

//    std::cout << "Partial TFs" << std::endl;
//    std::cout << "T1\n" << T1 << std::endl;
//    std::cout << "T2\n" << T2 << std::endl;
//    std::cout << "T3\n" << T3 << std::endl;
//    std::cout << "T4\n" << T4 << std::endl;
//    std::cout << "T5\n" << T5 << std::endl;
//    std::cout << "T6\n" << T6 << std::endl;
//    std::cout << "T7\n" << T7 << std::endl;


    // Return in metres
    for (unsigned int i=0; i<3; ++i)    T7(i,3) /= 1000.0;

    return T7;
}


void YumiKinematics::setJointsInLimits(Eigen::VectorXf& q){

    double tol = 0.0001;

    if      ( q(0) < -2.94088+tol )     q(0) = -2.94088+tol;
    else if ( q(0) >  2.94088-tol )     q(0) =  2.94088-tol;

    if      ( q(1) < -2.50455+tol )     q(1) = -2.50455+tol;
    else if ( q(1) > 0.759218-tol )     q(1) = 0.759218-tol;

    if      ( q(2) < -2.94088+tol )     q(2) = -2.94088+tol;
    else if ( q(2) >  2.94088-tol )     q(2) =  2.94088-tol;

    if      ( q(3) < -2.15548+tol )     q(3) = -2.15548+tol;
    else if ( q(3) >  1.39626-tol )     q(3) =  1.39626-tol;

    if      ( q(4) < -5.06145+tol )     q(4) = -5.06145+tol;
    else if ( q(4) >  5.06145-tol )     q(4) =  5.06145-tol;

    if      ( q(5) < -1.53589+tol )     q(5) = -1.53589+tol;
    else if ( q(5) >  2.40855-tol )     q(5) =  2.40855-tol;

    if      ( q(6) <  -3.9968+tol )     q(6) =  -3.9968+tol;
    else if ( q(6) >   3.9968-tol )     q(6) =   3.9968-tol;

    return;
}


bool YumiKinematics::solveIK(const Eigen::Matrix4f desiredPose,
                             const Eigen::VectorXf q_initial_num_IK,
                             const unsigned int alg_type,
                             const float redundantJoint,
                             Eigen::VectorXf& qResult,
                             const bool use_joint_saturation)
{
    bool ik_solved = false;

    if (alg_type == IK_NUMERICAL_INITIAL_Q){
        // Use initial configuration 'q_initial_num_IK' for one numerical ik

        if ( q_initial_num_IK.size() == 7 ){
            float max_iterations = 50;
            ik_solved = this->NumericalIKSolver(desiredPose, q_initial_num_IK, 0.001, max_iterations, qResult, use_joint_saturation);
        }
    }

    else if (alg_type == IK_NUMERICAL_RND_INITIAL_QS){
        // Generate 'n_max_init_q' random initial configurations
#ifdef VERBOSE_ON
        std::cout << " Numerical IK using multiple random initial configurations:" << std::endl;
#endif

        unsigned int i =0;
        unsigned int n_max_init_q = 10;
        for (i=0; !ik_solved && i<n_max_init_q; ++i){

            // Initial configuration
            Eigen::VectorXf q_initial(this->generateRandomConfiguration());     // In IK convention

            if (this->solveIK(desiredPose, q_initial, IK_NUMERICAL_INITIAL_Q, redundantJoint, qResult, use_joint_saturation))
                ik_solved = this->configurationInLimits(qResult);
        }

#ifdef VERBOSE_ON
        if (ik_solved)  std::cout << " IK SOLVED after " << i << " initial configurations: configuration is within limits." << std::endl;
        else            std::cout << " IK NOT SOLVED after " << i << " initial configurations: no configuration within limits." << std::endl;
#endif
    }

    else if (alg_type == IK_ANALYTICAL_Q){
        // Use analytical IK to feed the numerical IK
#ifdef VERBOSE_ON
        std::cout << " Analytical IK to feed the numerical IK:" << std::endl;
#endif
        std::vector< std::vector<float> > yumiAnalyticalIkSolutions = this->AnalyticalIKSolver(desiredPose, redundantJoint);

        if (yumiAnalyticalIkSolutions.size() > 0) {

            // Select the configuration with the closest TCP to the desired configuration
            bool at_least_one_non_NaN_config = false;    // There is at least one config without a NaN value
            double min_err = 1000000000;    // Super-high value
            Eigen::VectorXf minAnalyticalIkSolution(yumiAnalyticalIkSolutions[0].size());
            std::vector< Eigen::VectorXf > validSolutions;
            std::vector< float > validSolutions_errors;
            for(unsigned int i=0; i<yumiAnalyticalIkSolutions.size(); i++)
            {
                // For each solution of the analytical IK
                Eigen::VectorXf analyticalIk_q(yumiAnalyticalIkSolutions[i].size());
                for(unsigned int j=0; j<yumiAnalyticalIkSolutions[i].size(); j++)    analyticalIk_q(j) = yumiAnalyticalIkSolutions[i][j];

//                this->setJointsInLimits(analyticalIk_q);

                // Look for NaN values
                bool solution_has_no_NaN = true;
                for (unsigned int i=0; solution_has_no_NaN && (i<analyticalIk_q.size()); ++i){
                    // f != f will be true if f is NaN or -NaN
                    solution_has_no_NaN = !(analyticalIk_q[i] != analyticalIk_q[i]);
                }

                if (solution_has_no_NaN){
                    at_least_one_non_NaN_config = true;

                    validSolutions.push_back(analyticalIk_q);

                    // Desired pose
                    Eigen::Vector3f pd(desiredPose(0,3), desiredPose(1,3), desiredPose(2,3));
                    Eigen::Matrix3f Rd;
                    Rd << desiredPose(0,0), desiredPose(0,1), desiredPose(0,2),
                          desiredPose(1,0), desiredPose(1,1), desiredPose(1,2),
                          desiredPose(2,0), desiredPose(2,1), desiredPose(2,2);
                    Eigen::Quaternionf uqd(Rd);

                    // Pose of the analitical solution
                    Eigen::Matrix4f currentPose = this->ForwardKinematics(analyticalIk_q);

                    Eigen::Vector3f pe(currentPose(0,3), currentPose(1,3), currentPose(2,3));
                    Eigen::Matrix3f Re;
                    Re << currentPose(0,0), currentPose(0,1), currentPose(0,2),
                          currentPose(1,0), currentPose(1,1), currentPose(1,2),
                          currentPose(2,0), currentPose(2,1), currentPose(2,2);
                    Eigen::Quaternionf uqe(Re);

                    // Error
                    Eigen::Vector3f errorP(pd-pe);
                    Eigen::Vector3f u1(uqd.x(),uqd.y(),uqd.z());
                    Eigen::Vector3f u2(uqe.x(),uqe.y(),uqe.z());
                    Eigen::Matrix3f m;
                    m <<       0, -uqd.z(),  uqd.y(),
                         uqd.z(),        0, -uqd.x(),
                         uqd.y(),  uqd.x(),        0;
                    Eigen::Vector3f errorO(uqe.w()*u1-uqd.w()*u2-m*u2);

                    //  Total
                    double e = (errorO.squaredNorm()+errorP.squaredNorm())/2.0;
                    validSolutions_errors.push_back(e);

                    // Keep minimum error solution
                    if (e < min_err){
                        min_err = e;
                        minAnalyticalIkSolution = analyticalIk_q;
                    }
                } // if (solution_has_no_NaN)
            }

            // If all analytic configurations have NaN values there is no solution for the IK problem
            if (!at_least_one_non_NaN_config){
#ifdef VERBOSE_ON
                std::cout << "Inverse kinematics solution has a NaN value" << std::endl;
#endif
                return false;
            }

            // Use the initial configuration from the analytical Ik to feed the numerical solver
            ik_solved = this->solveIK(desiredPose, minAnalyticalIkSolution, IK_NUMERICAL_INITIAL_Q, redundantJoint, qResult, use_joint_saturation);
            bool config_in_limits = configurationInLimits(qResult);

#ifdef VERBOSE_ON
            if   (ik_solved){
                if (config_in_limits)   std::cout << " IK SOLVED using the analytical IK" << std::endl;
                else                    std::cout << " IK NOT SOLVED: The analytical IK converges but the final configuration is not in limits. The numerical Ik with multiple initial random configurations will be executed." << std::endl;
            }
            else                        std::cout << " IK NOT SOLVED using the analytical IK. The numerical Ik with multiple initial random configurations will be executed." << std::endl;
#endif

            // If Ik not solved, use multiple random initial configurations
            if ( !ik_solved || !config_in_limits )      ik_solved = this->solveIK(desiredPose, minAnalyticalIkSolution, IK_NUMERICAL_RND_INITIAL_QS, redundantJoint, qResult, use_joint_saturation);

            // Analytical IK Test
            for (unsigned int i=0; i<7; ++i)    qResult(i) = validSolutions[0](i);
        }
    }

    else    this->solveIK(desiredPose, q_initial_num_IK, IK_NUMERICAL_RND_INITIAL_QS, redundantJoint, qResult, use_joint_saturation);

    return ik_solved;
}


Eigen::VectorXf YumiKinematics::generateRandomConfiguration(){

    Eigen::VectorXf q(7);
    q(0) = -2.94088            + ((float) rand() / RAND_MAX) * (  2.94088 - (-2.94088) );
    q(1) = -2.50455            + ((float) rand() / RAND_MAX) * ( 0.759218 - (-2.50455) );
    q(2) = -2.94088            + ((float) rand() / RAND_MAX) * (  2.94088 - (-2.94088) );
    q(3) = -2.15548            + ((float) rand() / RAND_MAX) * (  1.39626 - (-2.15548) );
    q(4) = -5.06145            + ((float) rand() / RAND_MAX) * (  5.06145 - (-5.06145) );
    q(5) = -1.53589            + ((float) rand() / RAND_MAX) * (  2.40855 - (-1.53589) );
    q(6) =  -3.9968            + ((float) rand() / RAND_MAX) * (   3.9968 - ( -3.9968) );
    return q;
}


bool YumiKinematics::configurationInLimits(const Eigen::VectorXf q){

//    std::cout << " valid q? " << q.transpose() << std::endl;

    double tol = 0.0001;

    if      ( q(0) < -2.94088-tol )     return false;
    else if ( q(0) >  2.94088+tol )     return false;

    if      ( q(1) < -2.50455-tol )     return false;
    else if ( q(1) > 0.759218+tol )     return false;

    if      ( q(2) < -2.94088-tol )     return false;
    else if ( q(2) >  2.94088+tol )     return false;

    if      ( q(3) < -2.15548-tol )     return false;
    else if ( q(3) >  1.39626+tol )     return false;

    if      ( q(4) < -5.06145-tol )     return false;
    else if ( q(4) >  5.06145+tol )     return false;

    if      ( q(5) < -1.53589-tol )     return false;
    else if ( q(5) >  2.40855+tol )     return false;

    if      ( q(6) <  -3.9968-tol )     return false;
    else if ( q(6) >   3.9968+tol )     return false;

    return true;
}

