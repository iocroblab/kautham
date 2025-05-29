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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#if !defined(_KAUTHAMSHELL_H)
#define _KAUTHAMSHELL_H

#include <vector>
#include <string>
#include <map>
#include <Eigen/Geometry>

// Must be included here, because if not, the code that use kauthamShell must include them, because public methods depends on Robot and RobConf.
#include <kautham/sampling/robconf.h>
#include <kautham/problem/robot.h>


namespace Kautham {
    
    // class Robot;
    // class RobConf;

    class kauthamshell {

        public:
            kauthamshell();//! Constructor
            ~kauthamshell(); //! Destructor
            bool problemOpened(); //! Informs wether there is a problem opened;
            void closeProblem();
            bool openProblem(std::istream *inputfile, std::vector<std::string> def_path = std::vector<std::string>());
            bool openProblem(std::string problemfilename, std::vector<std::string> def_path);
            bool checkCollision(std::vector<double> smpcoords, bool *collisionFree, std::string *msg, std::pair<std::pair<int, std::string>, std::pair<int, int> > *colliding_elements = NULL);
            bool setRobotsConfig(std::vector<double> smpcoords);
            bool setRobotsConfig(std::vector<double> smpcoords, std::vector<RobConf> &config);
            bool setObstaclesConfig(std::vector<double> smpcoords);
            bool setQuery(std::vector<double> init, std::vector<double> goal);
            bool setQuery(std::vector<std::string> control_names, std::vector<double> init, std::vector<double> goal);
            bool setInit(std::vector<double> init);
            bool setGoal(std::vector<double> goal);
            bool setInitObs(std::vector<double> initObs);
            bool clearSampleSet();
            bool setRobControls(std::istream* inputfile, std::vector<double> init, std::vector<double> goal);
            bool setRobControls(std::string controlsFile, std::vector<double> init, std::vector<double> goal);
            bool setRobControlsNoQuery(std::string inputfile);
            bool setDefaultRobControls(std::vector<double> init, std::vector<double> goal);
            bool setObsControls(std::istream* inputfile, std::vector<double> initObs);
            bool setObsControls(std::string controlsFile, std::vector<double> initObs);
            bool setFixedObsControls();
            bool setPlannerByName(std::string name);
            bool setPlanner(std::istream* inputfile);
            bool setPlanner(std::string problemfilename);
            bool setPlannerParameter(std::string parameter, std::string value);
            bool solve(std::ostream &graphVizPlannerDataFile);
            bool getPath(std::ostream &path);
            bool getPath(std::vector<std::vector<double>> &path, std::vector<std::string> &requested_joint_names, bool _only_controlled=false);
            bool computeTrajecotry(std::vector<std::vector<double>> &path, std::vector<double> &desired_max_velocity, std::vector<double> &desired_max_acceleration, double max_path_deviation, double freq, std::vector<std::vector<double>> &traj_positions, std::vector<std::vector<double>> &traj_velocities, std::vector<double> & traj_time_from_start);
            bool getProblemMaxJointVelocities(std::vector<double> &requested_max_joint_velocities, std::vector<std::string> &requested_joint_names, bool _only_controlled=false);
            bool getTrajecotry(std::vector<double> &ratio_velocity, std::vector<double> &ratio_acceleration, double max_path_deviation, double freq, std::vector<std::string> &requested_joint_names, std::vector<std::vector<double>> &traj_positions, std::vector<std::vector<double>> &traj_velocities, std::vector<double> & traj_time_from_start, bool _only_controlled=false);
            bool setInterpolatePath(bool interpolate);
            bool getInterpolatePath(bool &interpolate);
            bool solve();
            double getLastPlanComputationTime();
            int getNumEdges();
            int getNumVertices();
            bool connect(std::vector<double> smpcoords1, std::vector<double> smpcoords2);
            int addRobot(std::string robFile, double scale, std::vector<double> home, std::vector<std::vector<double> > limits,
                        std::vector<std::vector<double> > mapMatrix, std::vector<double> offMatrix);
            bool removeRobot(unsigned index);
            int addObstacle(std::string obsFile, double scale, std::vector<double> home);
            bool removeObstacle(std::string obsname);
            double cumDistCheck(std::vector<double> smpcoords);
            bool attachObstacle2RobotLink(int robot, int link, std::string obs);
            bool attachObstacle2RobotLink(const std::string& _robot_name, const std::string& _robot_link_name, const std::string& _obstacle_name);
            bool detachObstacle(const std::string& obs);

            bool motionPlanner(std::vector<double> init, std::vector<double> goal, std::string root);

            bool setObstaclePos(std::string obsname, std::vector<double> pos);
            bool setObstaclePos(const std::string& _obs_name, const Eigen::AffineCompact3d& _obs_pose);
            bool getObstaclePos(std::string obsname, std::vector<double> &pos);
            bool getObstaclePos(const std::string& _obs_name, Eigen::AffineCompact3d& _obs_pose);
            
            std::map<std::string, Robot*> getObstaclesMap();
            bool findIK(int robIndx, bool armType, std::vector<double> pos, std::vector<double> conf, bool maintSameWrist, std::vector<double> *solution);
            bool setRobPos(unsigned int index, std::vector<double> pos);
            bool getRobPos(unsigned int index, std::vector<double> &pos);
            bool getRobHomePos(unsigned int index, std::vector<double> &pos);

            //  Used to set any DOF (from 0 to 1) of a robot through his name and the DOF identifier.
            bool updateRobotDOFOffsetByName(const std::string& _robot_name, const std::vector<std::pair<std::string, double>>& _dof_updates);

            int getNumRobots();
            int getNumObstacles();
            bool getRobotFileNames(std::vector<std::string> &rnames);
            bool getObstaclesNames(std::vector<std::string> &obsnames);
            bool getRobotJointNames(int r, std::vector<std::string> &jnames);
            bool getRobotIsSE3enabled(int r);
            bool getSampleConfig(const unsigned int& _index, std::vector<double>& _sample_config);

            // Kautham Public Main Constraints Methods:
            bool getConstraintIdsPerRobot(std::map<std::string,std::vector<std::string>>& _constraints_by_robot);
            bool updateConstraintTargetLink(const std::string& _robot_name, const std::string& _id, const std::string& _target_link);
            bool updateConstraintEnabled(const std::string& _robot_name, const std::string& _id, const bool& _enabled);
            // Kautham Public Orientation Constraints Methods:
            bool updateConstraintTargetOrientation(const std::string& _robot_name, const std::string& _id, const Eigen::Quaterniond& _target_orientation);
            bool updateConstraintTargetOrientation(const std::string& _robot_name, const std::string& _id, const double& qx, const double& qy, const double& qz, const double& qw);
            bool updateConstraintTolerance(const std::string& _robot_name, const std::string& _id, const double& _tolerance_value, const bool& _tolerance_variable, const double& _tolerance_grandient);
            bool updateConstraintFreeMovementAxes(const std::string& _robot_name, const std::string& _id, const bool& _free_x, const bool& _free_y, const bool& _free_z);
            // Kautham Public Geometric Constraints Methods:
            bool getConstraintGeoProperties(const std::string& _robot_name, const std::string& _id, std::string& _type, std::map<std::string, double>& _geo_params_map, Eigen::AffineCompact3d& _constraint_pose, std::string& _reference_frame_entity, std::string& _reference_frame_link_);

        private:
            void *memPtr_;

    };

}   // Namespace: Kautham

#endif  //_KAUTHAMSHELL_H
