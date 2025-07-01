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

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>    // std::runtime_error
#include <filesystem>   // std::filesystem
#include <cstring>      // strerror
#include <cerrno>       // errno
#include <unistd.h>     // pipe, fork, dup2, execlp, read, close, _exit
#include <sys/wait.h>   // waitpid, WIFEXITED, WEXITSTATUS
#include <queue>
#include <set>
#include <type_traits>

#include <kautham/problem/urdf.hpp>
#include <kautham/util/kthutil/kauthamexception.h>
#include <kautham/problem/ivelement.h>


namespace Kautham {

    KauthamParserURDF::KauthamParserURDF(const std::string& _urdf_file_path) {
        std::string urdf_xml;
    
        if (KauthamParserURDF::is_xacro(_urdf_file_path)) {
            urdf_xml = KauthamParserURDF::expand_xacro(_urdf_file_path);
        } else {
            urdf_xml = KauthamParserURDF::read_file(_urdf_file_path);
        }
    
        this->robot_model = urdf::parseURDF(urdf_xml);
        if (!this->robot_model) {
            std::cerr << "Failed to parse URDF robot model" << std::endl;
            this->load_success = false;
        } else {
            this->load_success = true;
        }
    }


    std::string KauthamParserURDF::getRobotName() const {
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get the robot name. URDF model not loaded.");
        }
        return this->robot_model->getName();
    }

    std::shared_ptr<const urdf::Link> KauthamParserURDF::getRobotRootLink() const {
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get the robot root link. URDF model not loaded.");
        }
        return this->robot_model->getRoot();
    }

    
    std::vector<std::string> KauthamParserURDF::getJointNames() const {
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get joint names. URDF model not loaded.");
        }
        std::vector<std::string> joint_names;
        for (const auto& pair : this->robot_model->joints_) {
            joint_names.push_back(pair.first);
        }
        return joint_names;
    }    

    size_t KauthamParserURDF::getNumLinks() const {
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get the number of links. URDF model not loaded.");
        }
        return this->robot_model->links_.size();
    }

    std::vector<std::shared_ptr<urdf::Joint>> KauthamParserURDF::getOrderedJoints() const {
        std::vector<std::shared_ptr<urdf::Joint>> ordered;
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get the joints ordered. URDF not loaded");
        }
        // Start from root and follow the tree
        std::queue<std::string> to_visit;
        std::set<std::string> visited;

        to_visit.push(this->robot_model->getRoot()->name);

        while (!to_visit.empty()) {
            std::string parent = to_visit.front();
            to_visit.pop();
            for (const auto& [name, joint] : this->robot_model->joints_) {
                if (joint->parent_link_name == parent && visited.find(joint->child_link_name) == visited.end()) {
                    ordered.push_back(joint);
                    to_visit.push(joint->child_link_name);
                    visited.insert(joint->child_link_name);
                }
            }
        }
        return ordered;
    }
    
    std::shared_ptr<urdf::Link> KauthamParserURDF::getLink(const std::string& name) const {
        if (!this->robot_model) {
            throw std::runtime_error("Not possible to get the link. URDF not loaded");
        }
        auto it = this->robot_model->links_.find(name);
        if (it != this->robot_model->links_.end()) {
            return it->second;
        }
        return nullptr;
    }

    // Check for .xacro extension
    bool KauthamParserURDF::is_xacro(const std::string& _filename) {
        std::filesystem::path p(_filename);
        return p.extension() == ".xacro";
    }

    // Read file contents into a string
    std::string KauthamParserURDF::read_file(const std::string& _filename) {
        std::ifstream stream(_filename);
        if (!stream) {
            throw std::runtime_error("Failed to open file: " + _filename);
        }
        return std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    }

    // Expand xacro to URDF string without temporary files
    std::string KauthamParserURDF::expand_xacro(const std::string& _xacro_file) {
        std::filesystem::path xacro_path(_xacro_file);
        std::filesystem::path dir = xacro_path.parent_path();
        std::string old_cwd = std::filesystem::current_path().string();
        
        if (!dir.empty()) {
            std::filesystem::current_path(dir);
        }

        // Create pipe for xacro output
        int pipefd[2];
        if (pipe(pipefd) == -1) {
            throw std::runtime_error("Pipe creation failed: " + std::string(strerror(errno)));
        }

        pid_t pid = fork();
        if (pid == -1) {
            close(pipefd[0]);
            close(pipefd[1]);
            throw std::runtime_error("Fork failed: " + std::string(strerror(errno)));
        }

        if (pid == 0) { // Child process
            close(pipefd[0]);  // Close read end
            dup2(pipefd[1], STDOUT_FILENO);
            close(pipefd[1]);

            execlp("xacro", "xacro", xacro_path.filename().c_str(), nullptr);
            std::cerr << "execlp failed: " << strerror(errno) << std::endl;
            _exit(EXIT_FAILURE);
        }

        // Parent process
        close(pipefd[1]);  // Close write end
        std::string result;
        char buffer[4096];
        ssize_t count;
        
        while ((count = read(pipefd[0], buffer, sizeof(buffer))) > 0) {
            result.append(buffer, count);
        }
        if (count == -1) {
            close(pipefd[0]);
            throw std::runtime_error("Read error: " + std::string(strerror(errno)));
        }
        close(pipefd[0]);

        int status;
        waitpid(pid, &status, 0);
        std::filesystem::current_path(old_cwd);

        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            throw std::runtime_error("xacro failed with exit code: " + std::to_string(WEXITSTATUS(status)));
        }

        return result;
    }

    void KauthamParserURDF::fillODEElementFromURDF(
        const std::shared_ptr<const urdf::Link>& _link,
        const std::shared_ptr<const urdf::Joint>& _joint,
        Kautham::ode_element& ode_)
    {
        // Inertial properties:
        if (_link->inertial) {
            ode_.inertial.mass = _link->inertial->mass;

            // Inertia tensor:
            ode_.inertial.inertia.ixx = _link->inertial->ixx;
            ode_.inertial.inertia.ixy = _link->inertial->ixy;
            ode_.inertial.inertia.ixz = _link->inertial->ixz;
            ode_.inertial.inertia.iyy = _link->inertial->iyy;
            ode_.inertial.inertia.iyz = _link->inertial->iyz;
            ode_.inertial.inertia.izz = _link->inertial->izz;
    
            // Origin position (center of mass):
            ode_.inertial.origin.xyz[0] = _link->inertial->origin.position.x;
            ode_.inertial.origin.xyz[1] = _link->inertial->origin.position.y;
            ode_.inertial.origin.xyz[2] = _link->inertial->origin.position.z;
    
            // Orientation as RPY:
            double roll, pitch, yaw;
            _link->inertial->origin.rotation.getRPY(roll, pitch, yaw);
            ode_.inertial.origin.r = roll;
            ode_.inertial.origin.p = pitch;
            ode_.inertial.origin.y = yaw;
        }
    
        // Dynamics (if available)
        if (_joint && _joint->dynamics) {
            ode_.dynamics.damping = _joint->dynamics->damping;
            ode_.dynamics.friction = _joint->dynamics->friction;
        }
    
        // _Joint limits
        if (_joint && _joint->limits) {
            ode_.limit.effort = _joint->limits->effort;
            ode_.limit.velocity = _joint->limits->velocity;
            ode_.limit.lower = _joint->limits->lower;
            ode_.limit.upper = _joint->limits->upper;
        } else if (_joint && _joint->type == urdf::Joint::CONTINUOUS) {
            // Set default limits for continuous joints
            ode_.limit.lower = -2 * M_PI;
            ode_.limit.upper = 2 * M_PI;
        }
    
    }

    SoSeparator* KauthamParserURDF::createSoSeparatorFromURDF(
        const std::string& type, // "visual" or "collision"
        const std::shared_ptr<const urdf::Link>& link,
        const std::string& mesh_dir)
    {
        SoSeparator* model = new SoSeparator;
        model->ref();

        if (type == "visual") {
            if (!link->visual_array.empty()) {
                for (const auto& visual : link->visual_array) {
                    SoSeparator* part = createSoSeparatorFromURDFElement(visual, mesh_dir);
                    if (part) model->addChild(part);
                }
            } else if (link->visual) {
                SoSeparator* part = createSoSeparatorFromURDFElement(link->visual, mesh_dir);
                if (part) model->addChild(part);
            }
        } else if (type == "collision") {
            if (!link->collision_array.empty()) {
                for (const auto& collision : link->collision_array) {
                    SoSeparator* part = createSoSeparatorFromURDFElement(collision, mesh_dir);
                    if (part) model->addChild(part);
                }
            } else if (link->collision) {
                SoSeparator* part = createSoSeparatorFromURDFElement(link->collision, mesh_dir);
                if (part) model->addChild(part);
            }
        }
        return model;
    }


    template<typename ElementPtr>
    SoSeparator* KauthamParserURDF::createSoSeparatorFromURDFElement(
        const ElementPtr& element,
        const std::string& mesh_dir)
    {
        if (!element) {
            return nullptr;
        }

        SoSeparator* model = new SoSeparator;
        model->ref();
        SoSeparator* submodel = new SoSeparator;
        model->addChild(submodel);

        // === Origin (translation + rotation) ===
        SoTranslation* trans = new SoTranslation;
        trans->translation.setValue(
            (float)element->origin.position.x,
            (float)element->origin.position.y,
            (float)element->origin.position.z
        );
        submodel->addChild(trans);

        urdf::Rotation q = element->origin.rotation;
        SbRotation coinRot((float)q.x, (float)q.y, (float)q.z, (float)q.w);
        SoRotation* rot = new SoRotation;
        rot->rotation.setValue(coinRot);
        submodel->addChild(rot);

        // === Material (Only urdf::Visual has material in standard URDF) ===
        if constexpr (std::is_same_v<typename ElementPtr::element_type, urdf::Visual>) {
            if (element->material) {
                SoMaterial* mat = new SoMaterial;
                mat->diffuseColor.setValue(
                    (float)element->material->color.r,
                    (float)element->material->color.g,
                    (float)element->material->color.b
                );
                mat->transparency.setValue(1.0f - (float)element->material->color.a);
                submodel->addChild(mat);
            }
        }

        // === Geometry ===
        if (element->geometry) {
            switch (element->geometry->type) {
            case urdf::Geometry::BOX: {
                auto* box = dynamic_cast<urdf::Box*>(element->geometry.get());
                if (box) {
                    SoCube* soBox = new SoCube;
                    soBox->width.setValue((float)box->dim.x);
                    soBox->height.setValue((float)box->dim.y);
                    soBox->depth.setValue((float)box->dim.z);
                    submodel->addChild(soBox);
                }
                break;
            }
            case urdf::Geometry::CYLINDER: {
                auto* cyl = dynamic_cast<urdf::Cylinder*>(element->geometry.get());
                if (cyl) {
                    // URDF cylinder is along Z, Coin3D along Y, so rotate
                    SoRotation* cyl_rot = new SoRotation;
                    cyl_rot->rotation.setValue(SbVec3f(1,0,0), (float)M_PI_2);
                    submodel->addChild(cyl_rot);

                    SoCylinder* soCyl = new SoCylinder;
                    soCyl->radius.setValue((float)cyl->radius);
                    soCyl->height.setValue((float)cyl->length);
                    submodel->addChild(soCyl);
                }
                break;
            }
            case urdf::Geometry::SPHERE: {
                auto* sph = dynamic_cast<urdf::Sphere*>(element->geometry.get());
                if (sph) {
                    SoSphere* soSph = new SoSphere;
                    soSph->radius.setValue((float)sph->radius);
                    submodel->addChild(soSph);
                }
                break;
            }
            case urdf::Geometry::MESH: {
                auto* mesh = dynamic_cast<urdf::Mesh*>(element->geometry.get());
                if (mesh) {
                    SoScale* sca = new SoScale;
                    sca->scaleFactor.setValue(
                        (float)mesh->scale.x,
                        (float)mesh->scale.y,
                        (float)mesh->scale.z
                    );
                    submodel->addChild(sca);

                    SoSeparator* meshNode = readFile(mesh_dir + mesh->filename);
                    if (meshNode) {
                        submodel->addChild(meshNode);
                    } else {
                        std::cerr << "Failed to load mesh: " << mesh_dir + mesh->filename << std::endl;
                    }
                }
                break;
            }
            default:
                break;
            }
        }
        return model;
    }

}