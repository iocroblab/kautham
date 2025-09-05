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

/* Author: Pol Ramon Canyameres */

#include <string>
#include <vector>

#include <Inventor/nodes/SoSeparator.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/joint.h> // Include this for Joint::Type
#include <kautham/problem/odeelement.h>

namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

    // Process the URDF (or XACRO) from the given file path:
    class KauthamParserURDF {
        public:
            KauthamParserURDF(const std::string& _urdf_file_path);    //!< Class constructor.

            inline bool isLoaded() const { return load_success; }

            // Returns the robot name:
            std::string getRobotName() const;
            
            std::shared_ptr<const urdf::Link> getRobotRootLink() const;

            // Return the joint names:
            std::vector<std::string> getJointNames() const;

            // Returns the number of links:
            size_t getNumLinks() const;

            // Returns the links in joint order (skipping the base link):
            std::vector<std::shared_ptr<urdf::Joint>> getOrderedJoints() const;

            // Returns a link by name:
            std::shared_ptr<urdf::Link> getLink(const std::string& name) const;

            void fillODEElementFromURDF(
                const std::shared_ptr<const urdf::Link>& _link,
                const std::shared_ptr<const urdf::Joint>& _joint,
                Kautham::ode_element& ode_);

            SoSeparator* createSoSeparatorFromURDF(
                const std::string& type, // "visual" or "collision"
                const std::shared_ptr<const urdf::Link>& link,
                const std::string& mesh_dir
            );                
                
        private:
            std::shared_ptr<urdf::ModelInterface> robot_model;
            bool load_success = false;
            
            bool is_xacro(const std::string& _filename);
            std::string read_file(const std::string& _filename);
            std::string expand_xacro(const std::string& _xacro_file);

            template<typename ElementPtr>
            SoSeparator* createSoSeparatorFromURDFElement(
                const ElementPtr& element, 
                const std::string& mesh_dir
            );
    };

/** @}   end of Doxygen module "Problem" */
}
