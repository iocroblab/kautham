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



#if !defined(_IVKINKUKALWR_H)
#define _IVKINKUKALWR_H


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <kautham/util/libkin/inversekinematic.h>


/** \addtogroup IK
 *  @{
 */

class IvKinKukaLWR:public Kautham::InverseKinematic{
public:
    IvKinKukaLWR(Robot* const rob);
    ~IvKinKukaLWR();
    INVKINECLASS type() {   return KUKA_LWR;  }
    string       name() {   return "KUKA_LWR";  }
    bool solve();
    bool setParameters();
    void setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist);
    bool arm_is_left(){    return _use_left_arm;  }
private:
    IvKinKukaLWR();
    vector<KthReal> _eulPos;
    double          _redundantJoint;
    double          _result[6];
    bool            _use_left_arm;

    void setJointInLimit(const unsigned i, float &value);
    void setJointsInLimits(Eigen::VectorXf &joints);
    float normalizeJoint(const unsigned i, const float value);
    Eigen::VectorXf normalizeJoints(const Eigen::VectorXf joints);
    float denormalizeJoint(const unsigned i, const float normalized_value);
    Eigen::VectorXf denormalizeJoints(const Eigen::VectorXf normalized_joints);
    float getLimit(const unsigned int i, bool high_limit);
};

/** @}   end of Doxygen module */

#endif // _IVKINKUKALWR_H
