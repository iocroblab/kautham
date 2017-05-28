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

    std::vector< std::vector<float> > AnalyticalIKSolver(const Eigen::Matrix4f Pose, const float theta3);

    std::vector< std::vector<float> > ComputeTheta4(const Eigen::Matrix4f Pose, const float theta3, const unsigned int byn);

    std::vector< std::vector<float> > ComputeTheta2(const Eigen::Matrix4f Pose, const float theta3, const float theta4, const unsigned int byn);

    std::vector< std::vector<float> > ComputeTheta1(const Eigen::Matrix4f Pose, const float theta2, const float theta3, const float theta4, const unsigned int byn);

    std::vector<float>  ComputeRotation(const Eigen::Matrix4f Pose, const float theta1, const float theta2, const float theta3, const float theta4, const unsigned int byn);

    Eigen::MatrixXf  Jacobian(const Eigen::VectorXf Q);

    bool NumericalIKSolver(const Eigen::Matrix4f desiredPose, const Eigen::VectorXf qIni, const float threshold, float &max_iterations, Eigen::VectorXf &qResult);

    template<typename _Matrix_Type_>  _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, float epsilon = std::numeric_limits<float>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        float tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    Eigen::Matrix4f ForwardKinematics(const Eigen::VectorXf Q);

    void setJointsInLimits(Eigen::VectorXf& q);
};

#endif
#endif
#endif
