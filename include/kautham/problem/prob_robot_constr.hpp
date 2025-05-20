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


#if !defined(_PROBLEM_ROBOT_CONSTRAINT_HPP)
#define _PROBLEM_ROBOT_CONSTRAINT_HPP


#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace Kautham {

    /** \addtogroup Problem
     *  @{
     */
    

    //! Class RobotProblemConstraint stores the definition of the Robot constraints defeined in the problem XML file:
    class RobotProblemConstraint {
        public:
            //!< Defines all the geometric params that can be used to define any geometric representation:
            struct GeometricParams {
                double length;
                double width;
                double height;
                double radius;
            };

            RobotProblemConstraint(std::string _id, std::string _type); //!<  Constructor

            void printRobProbConstraintInfo() const;

            static void assignIntersectionGroups(const std::vector<std::shared_ptr<RobotProblemConstraint>>& _constraints);

            // SET METHODS MAIN:
            bool associateNewJoint(const std::string& _joint_name, const uint _joint_index);
            inline void setTargetLink(const std::string& _target_link){target_link_ = _target_link;}
            inline void setEnabledStatus(const bool _status){enabled_ = _status;}
            inline void setIntersectionGroup(const std::string& _intersection_group){intersection_group_ = _intersection_group;}

            // SET METHODS ORIENTATION:
            void setTargetOrientation(const double quat_x, const double quat_y, const double quat_z, const double quat_w);
            void setTargetOrientation(const Eigen::Quaterniond& _quat);
            inline void setToleranceInfo(const double _value, const bool _variable, const double _gradient){tolerance_value_ = _value; tolerance_variable_ = _variable; tolerance_grandient_ = _gradient;}
            inline void setFreeMovementAxes(const bool free_x, const bool free_y, const bool free_z){free_movement_axes_.x() = static_cast<double>(free_x); free_movement_axes_.y() = static_cast<double>(free_y); free_movement_axes_.z() = static_cast<double>(free_z);}

            // SET METHODS GEOMETRIC:
            void setReferenceFrame(const std::string& _entity_id, const std::string& _link_name);
            void setOrigin(const double x, const double y, const double z, const double quat_x, const double quat_y, const double quat_z, const double quat_w);
            void setOrigin(const double x, const double y, const double z, const double roll, const double pitch, const double yaw);
            inline void setGeometricParamLength(const double _length){geo_params_.length = _length;}
            inline void setGeometricParamWidth(const double _width){geo_params_.width = _width;}
            inline void setGeometricParamHeight(const double _height){geo_params_.height = _height;}
            inline void setGeometricParamRadius(const double _radius){geo_params_.radius = _radius;}

            // GET METHODS MAIN:
            inline std::string getConstraintId() const {return id_;}
            inline std::string getConstraintType() const {return type_;}
            inline bool isConstraintOperative() const {return enabled_;}
            inline std::vector<std::pair<std::string, uint>> getConstrainedJoints() const {return constrained_joints_;}
            inline std::pair<std::string, uint> getLastConstrainedJoint() const {return constrained_joints_.back();} // Return the last element
            inline std::string getTargetLink() const {return target_link_;}
            inline std::string getIntersectionGroup() const {return intersection_group_;}

            // GET METHODS ORIENTATION:
            inline Eigen::Quaterniond getTargetOrientation() const {return target_orientation_;}
            inline double getToleranceValue() const {return tolerance_value_;}
            inline bool isToleranceVariable() const {return tolerance_variable_;}
            inline double getToleranceGradient() const {return tolerance_grandient_;}
            inline Eigen::Vector3d getFreeMovementAxes() const {return free_movement_axes_;}

            // GET METHODS GEOMETRIC:
            inline std::string getReferenceFrameEntity() const {return reference_frame_entity_;}
            inline std::string getReferenceFrameLink() const {return reference_frame_link_;}
            inline  Eigen::AffineCompact3d getReferencedFrameOrigin() const {return origin_;}
            inline double getGeometricParamLength() const {return geo_params_.length;}
            inline double getGeometricParamWidth() const {return geo_params_.width;}
            inline double getGeometricParamHeight() const {return geo_params_.height;}
            inline double getGeometricParamRadius() const {return geo_params_.radius;}

        private:
            // MAIN:
            std::string id_;  //!< Unique identifier.
            std::string type_;  //!< TCP Orientation or Geometric (box, sphere, cylinder, cone, ...).
            std::vector<std::pair<std::string, uint>> constrained_joints_;  //!< Related the joint name with the URDF kinematic position.
            bool enabled_;  //!< A constraint could be defined, but not used.
            std::string target_link_;  //!< The frame where the target is set.
            std::string intersection_group_; //!< Unique identifier for the group of constraints that share the same joints (i.e., intersect) with this constraint within the robot. Empty means without group.

            // ORIENTATION:
            Eigen::Quaterniond target_orientation_; //!< The desired orientation to maintain.
            double tolerance_value_;    //!< The tolerance of the constraint.
            bool tolerance_variable_;  //!< If the tolerance is fixed or variable in function of how far is the target_link from the goal.
            double tolerance_grandient_;    //!< The gradient in rad/m if the tolerance is variable.
            Eigen::Vector3d free_movement_axes_; //!< (x,y,z) bool axes, that indicates which axis will be free to move (when the constraint is used).
            
            // GEOMETRIC:
            GeometricParams geo_params_;    //!< Used to store the parameters that describes the geometries.
            std::string reference_frame_entity_;  //!< To which kautham entity (Robot || Obstacle || kautham world) is referenced the constraint.
            std::string reference_frame_link_;  //!< Which link of the reference is used (Robot -> Joint or Link; Obstacle -> Link; kautham world -> Empty).
            Eigen::AffineCompact3d origin_; //!< Transformation of the geometric constraint wrt the reference frame link.
    };

  /** @}   end of Doxygen module "Problem" */
}

#endif  //_PROBLEM_ROBOT_CONSTRAINT_HPP