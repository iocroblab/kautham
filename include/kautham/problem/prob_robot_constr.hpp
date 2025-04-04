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

            // Set methods:
            bool associateNewJoint(const std::string& _joint_name, const uint _joint_index);
            void setReferenceFrame(const std::string& _entity_id, const std::string& _link_name);
            void setOrigin(const double x, const double y, const double z, const double quat_x, const double quat_y, const double quat_z, const double quat_w);
            void setOrigin(const double x, const double y, const double z, const double roll, const double pitch, const double yaw);
            inline void setGeometricParamLength(const double _length){geo_params_.length = _length;}
            inline void setGeometricParamWidth(const double _width){geo_params_.width = _width;}
            inline void setGeometricParamHeight(const double _height){geo_params_.height = _height;}
            inline void setGeometricParamRadius(const double _radius){geo_params_.radius = _radius;}
            void setTargetOrientation(const double quat_x, const double quat_y, const double quat_z, const double quat_w);
            void setTargetOrientation(const Eigen::Quaterniond& _quat);
            inline void setOrientationLink(const std::string _orilink){orientation_link_ = _orilink;}

            // Get methods:
            inline std::string getConstraintId() const {return id_;}
            inline std::string getConstraintType() const {return type_;}
            inline std::vector<std::pair<std::string, uint>> getConstrainedJoints() const {return constrained_joints_;}
            inline Eigen::Quaterniond getTargetOrientation() const {return target_orientation_;}
            inline std::string getOrientationLink() const {return orientation_link_;}
            inline std::string getReferenceFrameEntity() const {return reference_frame_entity_;}
            inline  Eigen::AffineCompact3d getReferencedFrameOrigin() const {return origin_;}
            inline double getGeometricParamLength() const {return geo_params_.length;}
            inline double getGeometricParamWidth() const {return geo_params_.width;}
            inline double getGeometricParamHeight() const {return geo_params_.height;}
            inline double getGeometricParamRadius() const {return geo_params_.radius;}

        private:
            std::string id_;  //!< Unique identifier.
            std::string type_;  //!< TCP Orientation or Geometric (box, sphere, cylinder, cone, ...).
            std::vector<std::pair<std::string, uint>> constrained_joints_;  //!< Related the joint name with the URDF kinematic position.
            bool enabled_;  //!< A constraint could be defined, but not used.
            std::string reference_frame_entity_;  //!< To which kautham entity (Robot || Obstacle || kautham world) is referenced the contraint.
            std::string reference_frame_link_;  //!< Which link of the reference is used (Robot -> Joint or Link; Obstacle -> Link; kautham world -> Empty).
            Eigen::AffineCompact3d origin_; //!< Transformation of the geometric constraint wrt the reference frame link.
            GeometricParams geo_params_;    //!< Used to store the used params.
            Eigen::Quaterniond target_orientation_; //!< Used only when TCP Orientation constraint is used.
            std::string orientation_link_;  //!< Used only when TCP Orientation constraint is used.
    };



  /** @}   end of Doxygen module "Problem" */
}

#endif  //_PROBLEM_ROBOT_CONSTRAINT_HPP