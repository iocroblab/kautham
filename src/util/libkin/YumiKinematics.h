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

/* Author: Isiah Zaplana, Aliakbar Akbari, Muhayyudin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#if !defined(_YumiKinematics_H)
#define _YumiKinematics_H

#include <istream>
#include <vector>

#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>

class YumiKinematics
{
public:
    YumiKinematics();
    std::vector< std::vector<double> > AnalyticalIKSolver(Eigen::Matrix4f Pose, double theta3);
    std::vector< std::vector<double> > ComputeTheta4(Eigen::Matrix4f Pose, double theta3, int byn);
    std::vector< std::vector<double> > ComputeTheta2(Eigen::Matrix4f Pose, double theta3, double theta4,int byn);
    std::vector< std::vector<double> > ComputeTheta1(Eigen::Matrix4f Pose, double theta2, double theta3, double theta4,int byn);
    std::vector<double>  ComputeRotation(Eigen::Matrix4f Pose, double theta1, double theta2, double theta3, double theta4,int byn);
    Eigen::MatrixXf  Jacobian(Eigen::VectorXf Q);
    Eigen::VectorXf NumericalIKSolver(Eigen::Matrix4f desire_Pose, Eigen::Matrix4f current_Pose, Eigen::VectorXf qini, double threshold, double e);
    template<typename _Matrix_Type_>  _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
    Eigen::Matrix4f ForwardKinematics(Eigen::VectorXf Q);
};

#endif
#endif
#endif
