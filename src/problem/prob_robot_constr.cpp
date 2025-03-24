/*************************************************************************\
  Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Pol Ramon Canyameres, Jan Rosell */

#include <iostream>
#include <iomanip>

#include <kautham/problem/prob_robot_constr.hpp>

namespace Kautham {

    /* RobotProblemConstraint Class Methods */

    RobotProblemConstraint::RobotProblemConstraint(std::string _id, std::string _type) {
        this->id_ = _id;
        this->type_ = _type;
        this->enabled_ = true;
        this->origin_ = Eigen::AffineCompact3d::Identity();
    }

    void RobotProblemConstraint::printRobProbConstraintInfo() const {
        std::cout << "\tIdentifier: " << id_ << std::endl;
        std::cout << "\tType: " << type_ << std::endl;
        std::cout << "\tEnabled: " << (enabled_ ? "Yes" : "No") << std::endl;
        std::cout << "\tReference frame entity: " << reference_frame_entity_ << std::endl;
        std::cout << "\tReference frame link: " << reference_frame_link_ << std::endl;

        std::cout << "\tOrigin transformation matrix:\n";
        std::cout << std::fixed << std::setprecision(4); // Set fixed precision for floating-point numbers
        for (int i = 0; i < origin_.rows(); ++i) {
            std::cout << "\t\t";
            for (int j = 0; j < origin_.cols(); ++j) {
                std::cout << origin_(i, j) << " ";
            }
            std::cout << std::endl;
        }
        
        if (type_ == "orientation_ur5") {   // Change to tcp_orientation when robot agnostic
            std::cout << "\tTarget orientation (w, x, y, z): "
                    << target_orientation_.w() << ", " 
                    << target_orientation_.x() << ", "
                    << target_orientation_.y() << ", "
                    << target_orientation_.z() << std::endl;
        } else {
            std::cout << "\tGeometric parameters:" << std::endl;
            std::cout << "\t\tLength: " << geo_params_.length << std::endl;
            std::cout << "\t\tWidth: " << geo_params_.width << std::endl;
            std::cout << "\t\tHeight: " << geo_params_.height << std::endl;
            std::cout << "\t\tRadius: " << geo_params_.radius << std::endl;
        }
        
        std::cout << "\tApplied to joints (the order matters): " << std::endl;
        for (const auto& joint : constrained_joints_) {
            std::cout << "\t\tJoint[" << joint.second << "]: " << joint.first << std::endl;
        }
    }


    bool RobotProblemConstraint::associateNewJoint(const std::string& _joint_name, const uint _joint_index) {
        // Check if the joint is already associated
        for (const auto& joint : this->constrained_joints_) {
            if (joint.first == _joint_name || joint.second == _joint_index) {
                return false; // Joint name or index already exists
            }
        }
    
        // If not already associated, add the new joint
        this->constrained_joints_.push_back(std::make_pair(_joint_name, _joint_index));
        return true; // Successfully associated
    }

    void RobotProblemConstraint::setReferenceFrame(const std::string& _entity_id, const std::string& _link_name) {
        this->reference_frame_entity_ = _entity_id;
        this->reference_frame_link_ = _link_name;
    }

    void RobotProblemConstraint::setOrigin(const double x, const double y, const double z, const double quat_x, const double quat_y, const double quat_z, const double quat_w) {
        this->origin_ = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);
    }
    
    void RobotProblemConstraint::setOrigin(const double x, const double y, const double z, const double roll, const double pitch, const double yaw) {
        // Set translation
        this->origin_.translation() = Eigen::Vector3d(x, y, z);

        // Set rotation using Euler angles (roll, pitch, yaw)
        this->origin_.linear() = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    void RobotProblemConstraint::setTargetOrientation(const double quat_x, const double quat_y, const double quat_z, const double quat_w) {
        this->target_orientation_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);
    }

    void RobotProblemConstraint::setTargetOrientation(const Eigen::Quaterniond& _quat) {
        this->target_orientation_ = _quat;
    }

}