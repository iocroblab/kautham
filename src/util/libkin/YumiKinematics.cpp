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


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include <iostream>
#include <math.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "YumiKinematics.h"


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


    for(unsigned int i=0;i<SolutionSet1.size();i++)
        Solutions.push_back(SolutionSet1[i]);

    for(unsigned int i=0;i<SolutionSet2.size();i++)
        Solutions.push_back(SolutionSet2[i]);
    return Solutions;
}

std::vector< std::vector<float> > YumiKinematics::ComputeTheta4(const Eigen::Matrix4f Pose, const float theta3, const unsigned int byn)
{
    std::vector< std::vector<float> > SolutionSet1;
    std::vector< std::vector<float> > SolutionSet2;
    std::vector< std::vector<float> > Solutions;
    float theta4;
    float a=130014.5;
    float b=-41836.5;
//    float c= Pose(0,3)*Pose(0,3)+Pose(1,3)*Pose(1,3)+(Pose(2,3)-166)*(Pose(2,3)-166)-136757.75;
    float c= 1000.0*Pose(0,3)*1000.0*Pose(0,3)+1000.0*Pose(1,3)*1000.0*Pose(1,3)+(1000.0*Pose(2,3)-166)*(1000.0*Pose(2,3)-166)-136757.75;    // To mm
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
    float a=251.5+265*cos(theta4)-40.5*sin(theta4);
    float b=40.5*cos(theta3)-40.5*cos(theta3)*cos(theta4)-265*cos(theta3)*sin(theta4);
//    float c=Pose(2,3)-166;
    float c=1000.0*Pose(2,3)-166;  // To mm


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
    float a=40.5*cos(theta2)*cos(theta3)-251.5*sin(theta2)-265*cos(theta4)*sin(theta2)+40.5*sin(theta2)*sin(theta4)-40.5*cos(theta2)*
            cos(theta3)*cos(theta4)-265*cos(theta2)*cos(theta3)*sin(theta4);
    float b=265*sin(theta3)*sin(theta4)-40.5*sin(theta3)+40.5*sin(theta3)*cos(theta4);
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
                                       Eigen::VectorXf& qResult)
{
    // Initial configuration
    Eigen::VectorXf q(qIni);

    // Desired pose
    Eigen::Vector3f pd(desiredPose(0,3), desiredPose(1,3), desiredPose(2,3));
    Eigen::Matrix3f Rd;
    Rd << desiredPose(0,0), desiredPose(0,1), desiredPose(0,2),
          desiredPose(1,0), desiredPose(1,1), desiredPose(1,2),
          desiredPose(2,0), desiredPose(2,1), desiredPose(2,2);
    Eigen::Quaternionf uqd(Rd);
    std::cout << "-- > IK Desired Pose:" << std::endl << desiredPose << std::endl;

    // Initial pose
    Eigen::VectorXf qIniPlot(qIni);
    qIniPlot(3) = qIniPlot(3) + M_PI/2.0;
    std::cout << "-- > IK Initial Pose:" << std::endl << ForwardKinematics(qIniPlot) << std::endl;

    float e = 10000000000000000000.0;
    float past_e = 0;
    unsigned int n_it = 0;
    while ( (e > threshold) && (n_it < max_iterations) && (fabs(past_e - e) > threshold)  )
    {
        past_e = e;

//        std::cout << "q_ini: " << q.transpose() << std::endl;

        Eigen::Matrix4f current_Pose = ForwardKinematics(q);
//        std::cout << "-- > IK Current_Pose:" << std::endl << current_Pose << std::endl;

        // Current pose
        Eigen::Vector3f pe(current_Pose(0,3), current_Pose(1,3), current_Pose(2,3));
        Eigen::Matrix3f Re;
        Re << current_Pose(0,0), current_Pose(0,1), current_Pose(0,2),
              current_Pose(1,0), current_Pose(1,1), current_Pose(1,2),
              current_Pose(2,0), current_Pose(2,1), current_Pose(2,2);
        Eigen::Quaternionf uqe(Re);

        // Error
        //  Position
        Eigen::Vector3f errorP(pd-pe);

        //  Orientation
        Eigen::Vector3f ud(uqd.x(),uqd.y(),uqd.z());
        Eigen::Vector3f ue(uqe.x(),uqe.y(),uqe.z());
        Eigen::Matrix3f m;
        m <<        0, -uqd.z(),  uqd.y(),
              uqd.z(),        0, -uqd.x(),
             -uqd.y(),  uqd.x(),        0;
        Eigen::Vector3f errorO(uqe.w()*ud-uqd.w()*ue-m*ue);

        //  Total
        Eigen::VectorXf error(6);
        error << errorP(0), errorP(1), errorP(2), errorO(0), errorO(1), errorO(2);

//        q = q + 0.2 * pseudoInverse(Jacobian(q)) * error;
        q = q + 1.2 * pseudoInverse(Jacobian(q)) * error;

//        std::cout << "q:     " << q.transpose() << std::endl;
        setJointsInLimits(q);
//        std::cout << "q_sat: " << q.transpose() << std::endl;

        // Error for algorithm convergence
//        e = (std::sqrt(errorO.squaredNorm())+std::sqrt(errorP.squaredNorm()))/2.0;
//        e = std::sqrt(error.squaredNorm());
        // 0.001 m ~= 1º --> 0.001 m ~= 0.01745329444 rad
        e = ( std::sqrt(errorO.squaredNorm())+17.45329444*std::sqrt(errorP.squaredNorm()) )/2.0;

        ++n_it;
//        std::cout << n_it << " - yumi ik - e = ( ep, eo ): "
//                  << e << " = ( " << std::sqrt(errorP.squaredNorm()) << ", " << std::sqrt(errorO.squaredNorm()) << ")" << std::endl;
    }
    qResult = q;
    max_iterations = n_it;

    std::cout << "-- > IK " << ((e < threshold) ? "" : "NOT ") << "SOLVED in " << n_it << " iterations"
              << " : e = " << e << ((e < threshold) ? " < " : " > ") << threshold << std::endl;
    std::cout << "-- > IK Final Pose:" << std::endl << this->ForwardKinematics(qResult) << std::endl;

    if ( e < threshold )    return true;
    else                    return false;
}

Eigen::Matrix4f YumiKinematics::ForwardKinematics(const Eigen::VectorXf Q)
{
    Eigen::VectorXf q(Q.size());
    q(0)=Q(0); q(1)=Q(1); q(2)=Q(2); q(3)=Q(3); q(4)=Q(4); q(5)=Q(5); q(6)=Q(6);

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

    if      ( q(0) < -2.94088 )     q(0) = -2.94088;
    else if ( q(0) >  2.94088 )     q(0) =  2.94088;

    if      ( q(1) < -2.50455 )     q(1) = -2.50455;
    else if ( q(1) > 0.759218 )     q(1) = 0.759218;

    if      ( q(2) < -2.94088 )     q(2) = -2.94088;
    else if ( q(2) >  2.94088 )     q(2) =  2.94088;

    if      ( q(3) < -2.15548 )     q(3) = -2.15548;
    else if ( q(3) >  1.39626 )     q(3) =  1.39626;

//    if      ( q(4) < -5.06145 )     q(4) = -5.06145;
//    else if ( q(4) >  5.06145 )     q(4) =  5.06145;
    if      ( q(4) < -5.06145 + M_PI/2.0 )      q(4) = -5.06145 + M_PI/2.0;
    else if ( q(4) >  5.06145 + M_PI/2.0 )      q(4) =  5.06145 + M_PI/2.0;

    if      ( q(5) < -1.53589 )     q(5) = -1.53589;
    else if ( q(5) >  2.40855 )     q(5) =  2.40855;

    if      ( q(6) <  -3.9968 )     q(6) =  -3.9968;
    else if ( q(6) >   3.9968 )     q(6) =   3.9968;

    return;
}

#endif
#endif
