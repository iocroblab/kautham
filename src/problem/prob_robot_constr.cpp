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
#include <set>
#include <map>

#include <kautham/problem/prob_robot_constr.hpp>

namespace Kautham {

    /* RobotProblemConstraint Class Methods */

    RobotProblemConstraint::RobotProblemConstraint(std::string _id, std::string _type) {
        this->id_ = _id;
        this->type_ = _type;
        this->enabled_ = true;
        this->origin_ = Eigen::AffineCompact3d::Identity();
        this->allowed_volume_region_ = AllowedVolumeRegion::Inside;
    }

    void RobotProblemConstraint::printRobProbConstraintInfo() const {
        std::cout << "\tIdentifier: " << id_ << std::endl;
        std::cout << "\tType: " << type_ << std::endl;
        std::cout << "\tIntersection group: " << (intersection_group_.empty() ? "None" : intersection_group_) << std::endl;
        std::cout << "\tEnabled: " << (enabled_ ? "Yes" : "No") << std::endl;
        std::cout << "\tTarget link: " << target_link_ << std::endl;
        
        if (type_ == "arm_orientation") {
            std::cout << "\tTarget orientation (w, x, y, z): "
                    << target_orientation_.w() << ", " 
                    << target_orientation_.x() << ", "
                    << target_orientation_.y() << ", "
                    << target_orientation_.z() << std::endl;
            std::cout << "\tFreeMovementAxes:\n";
            std::cout << "\t\tx: " << (free_movement_axes_.x() ? "Free" : "Restricted") << std::endl;
            std::cout << "\t\ty: " << (free_movement_axes_.y() ? "Free" : "Restricted") << std::endl;
            std::cout << "\t\tz: " << (free_movement_axes_.z() ? "Free" : "Restricted") << std::endl;
            std::cout << "\tTolerance:\n";
            std::cout << "\t\tValue (rad): " << tolerance_value_ << std::endl;
            std::cout << "\t\tVariable: " << (tolerance_variable_ ? "Yes" : "No") << std::endl;
            std::cout << "\t\tGradient (rad/m): " << tolerance_grandient_ << std::endl;
        } else {
            std::cout << "\tGeometric parameters:" << std::endl;
            std::cout << "\t\tLength (m): " << geo_params_.length << std::endl;
            std::cout << "\t\tWidth (m): " << geo_params_.width << std::endl;
            std::cout << "\t\tHeight (m): " << geo_params_.height << std::endl;
            std::cout << "\t\tRadius (m): " << geo_params_.radius << std::endl;

            std::cout << "\tAllowed region: ";
            switch (allowed_volume_region_) {
                case AllowedVolumeRegion::Inside:
                    std::cout << "Inside";
                    break;
                case AllowedVolumeRegion::Outside:
                    std::cout << "Outside";
                    break;
                default:
                    std::cout << "Unknown";
                    break;
            }
            std::cout << std::endl;
            
            std::cout << "\tReference frame entity: " << reference_frame_entity_ << std::endl;
            std::cout << "\tReference frame link: " << reference_frame_link_ << std::endl;
    
            std::cout << "\tOrigin transformation matrix:\n";
            std::cout << std::fixed << std::setprecision(4);
            for (int i = 0; i < origin_.rows(); ++i) {
                std::cout << "\t\t";
                for (int j = 0; j < origin_.cols(); ++j) {
                    std::cout << origin_(i, j) << " ";
                }
                std::cout << std::endl;
            }
        }
        
        std::cout << "\tApplied to joints (the order matters): " << std::endl;
        for (const auto& joint : constrained_joints_) {
            std::cout << "\t\tJoint[" << joint.second << "]: " << joint.first << std::endl;
        }
    }

    void RobotProblemConstraint::assignIntersectionGroups(const std::vector<std::shared_ptr<RobotProblemConstraint>>& _constraints) {
        // Map from set of joint names to all instances with that set
        std::map<std::set<std::string>, std::vector<std::shared_ptr<RobotProblemConstraint>>> groups;

        for (auto& constraint : _constraints) {
            std::set<std::string> joint_names;
            for (const auto& p : constraint->getConstrainedJoints()) {
                joint_names.insert(p.first);
            }
            groups[joint_names].push_back(constraint);
        }

        // Assign a group name to each group (only if more than one instance in the group)
        int group_counter = 1;
        for (auto& entry : groups) {
            if (entry.second.size() > 1) { // Only assign group if more than one instance
                std::string group_name = "intersection_group_" + std::to_string(group_counter++);
                for (auto& constraint : entry.second) {
                    constraint->setIntersectionGroup(group_name);
                }
            }
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